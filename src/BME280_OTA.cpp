// ota from Arduino BasicOTA
#include <Arduino.h>
#include <stdio.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
// wifimanager 9/16
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
// SPIFFS
#include <FS.h>

/*
TODO:
WifiManager
Settings for mqtt server, port, user, pw
  interval sec
add mqtt topic support for:
  version
Second bme280

 */

// bme280
// From https://github.com/finitespace/BME280
// see also https://github.com/jainrk/i2c_port_address_scanner  for diagnostics
#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>

// Persistent properties
char mqtt_server[81] = "";
char mqtt_port[6] = "";
char mqtt_username[41] = "";
char mqtt_password[41] = "";

String topic_prefix = "blorp1";
const char * version = "version 2.3 Quinn";

WiFiClient espClient;
PubSubClient mqclient(espClient);
long lastPub, pubWait=300000;  // pubWait = milliseconds between publish
WiFiManager wifiManager;

long lastMQ = 0;        // mq timeout settings
long mqTimeout = 15000;

void readAndPublish();
void startConfigPortal();

BME280I2C::Settings settings(
    BME280::OSR_X16,  // Temperature default x1
    BME280::OSR_X16,  // Pressure default x1
    BME280::OSR_X16,  // Humidity default x1
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_Off,
    BME280::SpiEnable_False,
    0x76 // I2C address. I2C specific.
);
BME280I2C::Settings settings2(
    BME280::OSR_X16,  // Temperature default x1
    BME280::OSR_X16,  // Pressure default x1
    BME280::OSR_X16,  // Humidity default x1
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms, 
    BME280::Filter_Off,
    BME280::SpiEnable_False,
    0x77 // I2C addresss. I2C specific.  77 is alternate address with SDO -> GND on the breakout board
);

BME280I2C bme( settings);    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
BME280I2C bme2( settings2);
int bmeDetected=0, bmeDetected2=0;

// MQTT functions
void mqpublish( const String &topic, const String & msg, int retained)
{
    Serial.printf("Publish  Topic :%s: , Msg :%s:\n", topic.c_str(), msg.c_str());
    mqclient.publish( topic.c_str(), msg.c_str(), retained);
}


void mqcallback(char* topic, byte* payload, unsigned int length)
{
    // handle message arrived
    // note: payload is not terminated with 0, fix it up
    char pay2[length+1];
    strncpy( pay2, (char *) payload, length);
    pay2[length]='\0';
    String msg(pay2);
    Serial.printf("Received Topic :%s: , Msg :%s:\n", topic, msg.c_str());
    if (  (topic_prefix + "/cmd") == topic )
    {
        if (msg == "stop")
        {
            // Serial.println( "Stopping");
            pubWait = -1;
        }
        if (msg == "start")
        {
            // Serial.println( "Starting");
            pubWait = 60000;
        }
        if (msg == "config")
        {
            // Serial.println( "Configing");
            startConfigPortal();
        }
        return;
    }
    if ( (topic_prefix + "/setsec") == topic )
    {
        long sec = msg.toInt();
        if ( 0 < sec  && sec < 6000)
        {
            sec *=1000;
            pubWait = sec;
        }
        else
        mqpublish( topic_prefix + "/status", "invalid setsec", 0);
    }
}

boolean mqreconnect()
{
    Serial.println("MQ connecting");
    int rc=0;
    // if username is null or empty, connect without username/password
    if (mqtt_username == NULL || strnlen( mqtt_username, sizeof(mqtt_username) ) == 0 )
        rc = mqclient.connect( (topic_prefix + "Client").c_str(), 
        (topic_prefix + "/status/lwt").c_str(), 0, 1, "offline") ;
    else  // connect with username/password
        rc = mqclient.connect( (topic_prefix + "Client").c_str(), mqtt_username, mqtt_password,
            (topic_prefix + "/status/lwt").c_str(), 0, 1, "offline"); 

    if ( rc )
    {
        lastMQ=millis();
        Serial.println("MQ connected, publishing");
        mqclient.publish( (topic_prefix + "/status/lwt").c_str(),"online", 1);
        mqclient.publish( (topic_prefix + "/status/version").c_str(), version, 1);
        mqclient.subscribe( (topic_prefix + "/cmd").c_str());
        mqclient.subscribe( (topic_prefix + "/setsec").c_str());
        readAndPublish();  // kickstart
    }
    else 
        Serial.printf( "Error connecting to broker %s port %s user %s pw %s\n", mqtt_server, mqtt_port, mqtt_username, mqtt_password );
    
    return mqclient.connected();
}

/*
Implement properties as SPIFF files
returns nonzero on error
 */
int setProperty( const char * name, const char* value)
{
    Serial.printf("setProperty %s to %s\n", name, value);
    File f = SPIFFS.open( name, "w");
    if (f) 
    {
      f.println( value);  // note println puts DOS EOL 0x0D0A -- 13, 10
      f.close();
      Serial.println("setProperty Succeeded");
      return 0;
    }
    // else
    return 1;  // error
}

int getProperty( const char * name, char * value, int maxlen)
{
    Serial.printf("getProperty %s\n", name);
    File f = SPIFFS.open( name, "r");
    if (f)
    {
      int didread = f.readBytesUntil('\r', value, maxlen);  // 0x0D  , 13
      value[didread]=0;
      Serial.printf("getProperty %s got [%s] size %d didread %d\n", name, value, strnlen( value, 30), didread);
      f.close();
      return 0;
    }
    // else
    return 1;  // error
}

// wifimanager config callback
void apConfigModeCallback (WiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    Serial.println(myWiFiManager->getConfigPortalSSID());
    digitalWrite( LED_BUILTIN, LOW);  // turn on LED
}

// wifimanager saveconfig callback
bool shouldSaveConfig = false;
void saveConfigCallback()
{
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, sizeof(mqtt_port));
WiFiManagerParameter custom_mqtt_username("username", "mqtt username", mqtt_username, sizeof(mqtt_username));
WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, sizeof(mqtt_password));

void startConfigPortal()
{
    Serial.println("startConfigPortal BEGIN");

    wifiManager.setTimeout( 120);
    int rc = wifiManager.startConfigPortal("QuinnConfig");
    Serial.printf("wifiManager.startConfigPortal rc=%d\n", rc);
    if ( ! rc )   // to force reconfig
    {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
    }

    Serial.println("connected...yeey :)");

    //read updated parameters from wifimgr
    strcpy(mqtt_server,   custom_mqtt_server.getValue());
    strcpy(mqtt_port,     custom_mqtt_port.getValue());
    strcpy(mqtt_username, custom_mqtt_username.getValue());
    strcpy(mqtt_password, custom_mqtt_password.getValue());

    if (shouldSaveConfig)
    {
        setProperty("mqtt_server", mqtt_server);
        setProperty("mqtt_port", mqtt_port);
        setProperty("mqtt_username", mqtt_username);
        setProperty("mqtt_password", mqtt_password);
    }


}

/**************************************************************** SETUP ************************/
void setup()
{
    Serial.begin(115200);
    Serial.println("Booting");
    pinMode( LED_BUILTIN, OUTPUT); 

    /////////////////////////////////////// SPIFFS filesystem
    SPIFFS.begin();
    delay(8000);

    // read persistent properties
    getProperty("mqtt_server", mqtt_server, sizeof(mqtt_server));
    getProperty("mqtt_port",   mqtt_port, sizeof(mqtt_port));
    getProperty("mqtt_username", mqtt_username, sizeof(mqtt_username));
    getProperty("mqtt_password", mqtt_password, sizeof(mqtt_password));

    wifiManager.addParameter( &custom_mqtt_server);
    wifiManager.addParameter( &custom_mqtt_port);
    wifiManager.addParameter( &custom_mqtt_username);
    wifiManager.addParameter( &custom_mqtt_password);

    wifiManager.setAPCallback(apConfigModeCallback);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setTimeout( 120);

    //wifiManager.resetSettings();  //reset settings - for testing!
    //startConfigPortal();

    /*
    File f = SPIFFS.open("/myprop.txt","w");
    if ( !f) Serial.println("Error opening file for reading");
    f.println("hello from file");
    f.close();
    */
    File f = SPIFFS.open("/myprop.txt","r");
    if ( !f) Serial.println("Error opening file for reading");
    String ans = f.readStringUntil( '\n');
    Serial.println(ans);
    f.close();

    ////////////////////////////////////  Wifi
    /* replaced with WM autoconnect
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }
    */

    /////////////////////////////////////  OTA
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
        else // U_SPIFFS
        type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    ////////////////////////////////////////////////////// BME280
        Wire.begin( 4, 5);    // (sda,scl) "wemos" d1 mini using sda=gpio4 (D2) , scl=gpio5 (D1)
        if (bme.begin())
        {
        // bme.chipID(); // Deprecated. See chipModel().
        switch(bme.chipModel())
        {
            case BME280::ChipModel_BME280:
                Serial.println("Found BME280 sensor! Success.");
                bmeDetected = 1;
                break;
            case BME280::ChipModel_BMP280:
                Serial.println("Found BMP280 sensor! No Humidity available.");
                bmeDetected = 1;
                break;
            default:
                Serial.println("Found UNKNOWN sensor! Error!");
        }
        }
        else
        {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
        }
        if (bme2.begin())
        {
        // bme.chipID(); // Deprecated. See chipModel().
        switch(bme2.chipModel())
        {
            case BME280::ChipModel_BME280:
                Serial.println("Found BME280-2 sensor! Success.");
                bmeDetected2 = 1;
                break;
            case BME280::ChipModel_BMP280:
                Serial.println("Found BMP280-2 sensor! No Humidity available.");
                bmeDetected2 = 1;
                break;
            default:
                Serial.println("Found UNKNOWN sensor! Error!");
        }
        }
        else
        {
        Serial.println("Could not find BME280-2 sensor!");
        delay(1000);
        }

    // MQTT
    mqclient.setServer(mqtt_server, atoi( mqtt_port));
    mqclient.setCallback(mqcallback);

}  // setup


/****************************************************************************** Loop ************/
void loop()
{
    /* wifi */
    if ( ! WiFi.isConnected() )
    {
        startConfigPortal( ); 
    }

    // ota
    ArduinoOTA.handle();

    // mqtt
    long now=millis();
    if (mqclient.connected()) 
    {
        lastMQ = now;
    }
    else  // not connected, try to reconnect until timeout
    {
        mqreconnect();
        delay(1000);  
        if (  ( millis() - lastMQ) > mqTimeout ) 
        {
            Serial.println("mq timeout, starting portal");
            startConfigPortal( ); 
            Serial.println("Ressetting ESP, not sure why");
            lastMQ = now;
            ESP.restart(); 
            delay(5000);  
        };
    }
    mqclient.loop();

    // bme
    // publish every so often
    now=millis();
    if ( pubWait > 0 && (now - lastPub > pubWait))
    {
        lastPub = now;
        readAndPublish();
    }

}  // loop

// return 1 for success, 0, for error
void readAndPublish()
{
    digitalWrite( LED_BUILTIN, LOW);
    BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
    BME280::PresUnit presUnit(BME280::PresUnit_inHg);  // PresUnit_inHg, PresUnit_Pa
    float temp(NAN), hum(NAN), pres(NAN);
    float temp2(NAN), hum2(NAN), pres2(NAN);
    bme.read(pres, temp, hum, tempUnit, presUnit);
    bme2.read(pres2, temp2, hum2, tempUnit, presUnit);

    /*
        TODO: If temp==NAN, publish error and exit
    */

    /*   first thermo   */
    String topic= topic_prefix + "/temp/degF";
    char msg[10];
    snprintf( msg, 10, "%.2f", temp);
    mqpublish( topic, String(msg), 1);

    topic= topic_prefix + "/humid/rh";
    snprintf( msg, 10, "%.2f", hum);
    mqpublish( topic, String(msg), 1);

    topic= topic_prefix + "/press/inHg";
    snprintf( msg, 10, "%.2f", pres);
    mqpublish( topic, String(msg), 1);

    /*    second thermo
    topic= topic_prefix + "/temp2/degF";
    snprintf( msg, 10, "%.2f", temp2);
    mqpublish( topic, String(msg) , 0);
    */

    delay( 250);
    digitalWrite( LED_BUILTIN, HIGH);
}
