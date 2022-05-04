#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <pins_arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <Updater.h>
#include "../WebUI/index_html.h"
#include "configuration.h"

/*
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;
*/

#define PinSchalter D1
#define PinRelais D2   
#define PinWifi D5
#define PinIndoorSensor D6
#define PinOutdoorSensor D7

const char* ssid = "tauPau";
const char *password = "Passw0rd";

const double UG = 8314.3; // J/(kmol*K) (universelle Gaskonstante)
const double mw = 18.016; // kg/kmol (Molekulargewicht des Wasserdampfes)
const double K = 273.15;  // °K (Entspricht 0°K)
const double tp = 6.1078; // hPa Triplettpunkt von Wasser bei 0,01°C

DHT_Unified dht_indoor(PinIndoorSensor, DHT22);
DHT_Unified dht_outdoor(PinOutdoorSensor, DHT22);
AsyncWebServer server(80);
bool wifimode = false;
bool fanOn = false;


#define R1 470000.0         // exact resistance of R1 (= 470 kOhm)
#define R2 22000.0          // exact resistance of R2 (= 22 kOhm)
#define ADC_STEPS 1024.0



/*
  A0 - ADC Pin
  ESP8266 has a 10-bit resolution, range of 0 to 1023
  Range of 0V to 1V, Do not provide more than 1V directly to the ADC Pin (TOUT – Pin 6)

  Voltage divider:
  ----------------
  R1= 470 kOhm
  R2=  22 kOhm
  RGes = 470 + 22 = 492 kOhm
  Umax_Tout = 1.0V

  max input voltage:
  ------------------
  Umax_in = (R1+R2) * Umax_Tout = 492/22*1 = 22.36V

  consumption (12V Batterie -> 12,8V):
  ------------------------------------
  I = U / (R1+R2) = 12,8V / 492 kOhm = 0.026 mA

  conversion factor:
  ------------------
  22.36 / 1024 = 0.0218359375 V/step

*/
float measureVoltage(){
    float inValue = analogRead(A0) / ADC_STEPS;
    return (R1+R2)/R2 * inValue * configuration.voltage_correction_factor;
}

void calcDewPoint(SSensorData *data)
{
  float a, b;
  if (data->temperature >= 0)
  {
    a = 7.5;
    b = 237.3;
  }
  else
  {
    a = 7.6;
    b = 240.7;
  }

  float sgp = 6.1078 * pow(10, (a * data->temperature) / (b + data->temperature));
  float gp = (data->humidity / 100) * sgp;
  float v = log10(gp / 6.1078);
  data->dewpoint = (b * v) / (a - v);
  data->abshumidity = 100000 * mw / UG * data->humidity / 100 * tp * pow(10, ((a * data->temperature) / (b + data->temperature))) / (data->temperature + 273.15);
}

bool readSensor(DHT_Unified *dev, SSensorData *data, float offset_temperature = 0.0, float offset_humidity = 0.0)
{
  sensors_event_t event;

  dev->temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    return false;
  }

  data->temperature = event.temperature + offset_temperature;

  dev->humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    return false;
  }
  data->humidity = event.relative_humidity + offset_humidity;

  calcDewPoint(data);
  return true;
}

bool getFanDecision(SSensorData *indoordata, SSensorData *outdoordata, int lastFanMode)
{
  if (indoordata->temperature < configuration.min_temperature_indoor)
  {
    // Inntentemperatur zu kalt, Lüfter aus , Frostschutz
    return false;
  }
  if (outdoordata->temperature < configuration.min_temperature_outdoor)
  {
    // Außentemperatur zu kalt, Lüfter aus
    return false;
  }

  if (outdoordata->humidity > configuration.max_humidity_outdoor)
  {
    // Außenfeuchtigkeit zu hoch, Lüfter aus
    return false;
  }

  if (indoordata->humidity < configuration.min_humidity_indoor)
  {
    // erst ab 55% Luftfeutigkeit in der Garage muss zwangsbelüftet werden
    return false;
  }

  float deltadew = indoordata->dewpoint - outdoordata->dewpoint;
  if (deltadew > configuration.threshold_dewpoint_on)
  {
    // Einschaltschwelle überschritte, Fan on
    return true;
  }

  if (lastFanMode == HIGH && deltadew > configuration.threshold_dewpoint_off)
  {
    // Auschaltschwelle noch nicht unterschritten, keep Fan on
    return true;
  }

  return false;
}

void setup()
{
  Serial.begin(9600);
  wifimode = digitalRead(PinWifi) == LOW;
  loadconfig(&configuration);

  if(!wifimode && measureVoltage()< configuration.min_voltage){
    Serial.println("voltager to low, go to sleep");
    ESP.deepSleep(0xffffffff, WAKE_RF_DISABLED);
  }

  dht_indoor.begin();
  dht_outdoor.begin();

  pinMode(PinWifi, INPUT_PULLUP);
  pinMode(PinSchalter, INPUT_PULLUP);
  pinMode(PinRelais, OUTPUT);
  digitalWrite(PinRelais, LOW);

  if (wifimode)
  {        
    //setup ap
    Serial.println("Setting soft-AP ... ");
    WiFi.softAP(ssid, password);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
          AsyncWebServerResponse *response = request->beginResponse_P( 200, "text/html", index_html,sizeof(index_html));
          response->addHeader("Content-Encoding","deflate");    
          request->send(response); });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
              {        
        AsyncResponseStream *response = request->beginResponseStream("application/json");                      
        DynamicJsonDocument root(4096);
        root["fanOn"] = fanOn ? "On":"Off";
        root["dht_indoor_temperature"] = String(sensorDataIndoor.temperature,2);
        root["dht_indoor_humidity"] = String(sensorDataIndoor.humidity,2);
        root["dht_indoor_abshumidity"] = String(sensorDataIndoor.abshumidity,2);
        root["dht_indoor_dewpoint"] = String(sensorDataIndoor.dewpoint,2);
        root["dht_outdoor_temperature"] = String(sensorDataOutdoor.temperature,2);
        root["dht_outdoor_humidity"] = String(sensorDataOutdoor.humidity,2);
        root["dht_outdoor_abshumidity"] = String(sensorDataOutdoor.abshumidity,2);
        root["dht_outdoor_dewpoint"] = String(sensorDataOutdoor.dewpoint,2);
        root["voltage"] = String(measureVoltage(),2);
        serializeJson(root,*response);        
        request->send(response); });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
              {        
        AsyncResponseStream *response = request->beginResponseStream("application/json");                      
        DynamicJsonDocument root(4096);
        root["offset_temperature_indoor"] = configuration.offset_temperature_indoor;
        root["offset_humidity_indoor"] = configuration.offset_humidity_indoor;
        root["offset_temperature_outdoor"] = configuration.offset_temperature_outdoor;
        root["offset_humidity_outdoor"] = configuration.offset_humidity_outdoor;
        root["min_temperature_indoor"] = configuration.min_temperature_indoor;
        root["min_temperature_outdoor"] = configuration.min_temperature_outdoor;
        root["max_humidity_outdoor"] = configuration.max_humidity_outdoor;
        root["min_humidity_indoor"] = configuration.min_humidity_indoor;
        root["threshold_dewpoint_on"] = configuration.threshold_dewpoint_on;
        root["threshold_dewpoint_off"] = configuration.threshold_dewpoint_off;
        root["sleeptime_ms_fan_on"] = configuration.sleeptime_ms_fan_on;
        root["deepsleeptime_ms_fan_off"] = configuration.deepsleeptime_ms_fan_off;
        root["min_voltage"] = configuration.min_voltage;
        root["voltage_correction_factor"] = configuration.voltage_correction_factor;
        serializeJson(root,*response);        
        request->send(response); });

    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
                         {
        if (request->url() == "/config" && request->method() == HTTP_POST) {
          DynamicJsonDocument doc(4096);
          DeserializationError error = deserializeJson(doc, (const char*)data);
          if (error) {
            Serial.print(F("Error parsing JSON "));
            Serial.println(error.c_str());
            String msg = error.c_str(); 
            request->send(400, F("text/html"), "Error in parsin json body! <br>" + msg);
            return;
          } 

          configuration.offset_temperature_indoor = doc["offset_temperature_indoor"];
          configuration.offset_humidity_indoor = doc["offset_humidity_indoor"];
          configuration.offset_temperature_outdoor = doc["offset_temperature_outdoor"];
          configuration.offset_humidity_outdoor = doc["offset_humidity_outdoor"];
          configuration.min_temperature_indoor = doc["min_temperature_indoor"];
          configuration.min_temperature_outdoor = doc["min_temperature_outdoor"];
          configuration.max_humidity_outdoor = doc["max_humidity_outdoor"];
          configuration.min_humidity_indoor = doc["min_humidity_indoor"];
          configuration.threshold_dewpoint_on = doc["threshold_dewpoint_on"];
          configuration.threshold_dewpoint_off = doc["threshold_dewpoint_off"];          
          configuration.sleeptime_ms_fan_on = doc["sleeptime_ms_fan_on"];
          configuration.deepsleeptime_ms_fan_off = doc["deepsleeptime_ms_fan_off"];
          configuration.min_voltage = doc["min_voltage"];
          configuration.voltage_correction_factor = doc["voltage_correction_factor"];

          if(saveconfig(&configuration)){
              request->send(200, "text/plain", "success, config stored");
          }else{
            request->send(500, "text/plain", "error, config not stored");
          }
        } });

    server.on("/sysinfo", HTTP_GET, [](AsyncWebServerRequest *request)
              {      
        char buffer[150];
        rst_info* rinfo = ESP.getResetInfoPtr();
      
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        DynamicJsonDocument root(1024);
        root["freemem"] = ESP.getFreeHeap();    
        root["hostname"] = WiFi.hostname();
        root["ip"] = WiFi.localIP().toString();
        root["ssid"] = String(ssid);
        root["wifistatus"] = WiFi.status();
        root["resetreason"] =ESP.getResetReason();
        root["errors"] =  rinfo->exccause;
        
        //The	address	of	the	last	crash	is	printed,	which	is	used	to    
        sprintf(buffer, "epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x, exccause=0x%x, reason=0x%x",
          rinfo->epc1,	rinfo->epc2,	rinfo->epc3,	rinfo->excvaddr,	rinfo->depc, rinfo->exccause, rinfo->reason); 
        root["rstinfo"] =  buffer;
        serializeJson(root,*response);

        request->send(response); });

    AsyncElegantOTA.begin(&server); 
    server.begin();
    Serial.println("HTTP server started");

    return;
  }

  wifimode = false;
  WiFi.mode(WIFI_OFF);
}

void loop()
{
  if (Update.isRunning())
  {
    return;
  }
  if (Update.hasError())
  {
    Serial.println(F("Update Error:"));
    Serial.println(Update.getError());
    Update.clearError();
  }

  if (!readSensor(&dht_indoor, &sensorDataIndoor, configuration.offset_temperature_indoor, configuration.offset_humidity_indoor))
  {
    Serial.println(F("Error reading indoor sensor!"));
    delay(5000);
    return;
  }
  if (!readSensor(&dht_outdoor, &sensorDataOutdoor, configuration.offset_temperature_outdoor, configuration.offset_humidity_outdoor))
  {
    Serial.println(F("Error reading outdoor sensor!"));
    delay(5000);
    return;        
  }

  fanOn = digitalRead(PinSchalter) == LOW || getFanDecision(&sensorDataIndoor, &sensorDataOutdoor, fanOn);
  digitalWrite(PinRelais, fanOn ? HIGH : LOW);

  if (wifimode)
  {
    return;
  }

  Serial.print(fanOn);
  Serial.print("\t");

  Serial.print(sensorDataIndoor.temperature);
  Serial.print(F("°C"));
  Serial.print("\t");

  Serial.print(sensorDataOutdoor.temperature);
  Serial.print(F("°C"));
  Serial.print("\t");

  Serial.print(sensorDataIndoor.humidity);
  Serial.print(F("%"));
  Serial.print("\t");

  Serial.print(sensorDataOutdoor.humidity);
  Serial.print(F("%"));
  Serial.print("\t");

  Serial.print(sensorDataIndoor.dewpoint);
  Serial.print(F("°C"));
  Serial.print("\t");

  Serial.print(sensorDataOutdoor.dewpoint);
  Serial.println(F("°C"));

  if (fanOn)
  {
    Serial.println(F("goto Sleep"));
    delay(configuration.sleeptime_ms_fan_on);
  }
  else
  {
    Serial.println(F("goto DeepSleep"));
    ESP.deepSleep(configuration.deepsleeptime_ms_fan_off * 1000, WAKE_RF_DISABLED); // 30 Minuten schlafen
  }
} 