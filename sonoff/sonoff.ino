/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>

#define DHTPIN 14    // GPIO digital pin
#define DHTTYPE DHT22 

DHT dht(DHTPIN, DHTTYPE);
const int buttonPin = 0;
int buttonState = 0; 
int lastButtonState = 0; 
/************************* WiFi Access Point *********************************/

//#define WLAN_SSID       "Pranaychips"
//#define WLAN_PASS       "pranay6642"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "USER_NAME "
#define AIO_KEY         "KEY"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photocell");
const char TEMP_FEED[]  = AIO_USERNAME "/feeds/temp";
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, TEMP_FEED);

// Setup a feed called 'humidity' for publishing.
const char HUMIDITY_FEED[]  = AIO_USERNAME "/feeds/humidity";
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

// Setup a feed called 'me' for subscribing to changes.
Adafruit_MQTT_Subscribe me = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/switch");
const char me_FEED[]  = AIO_USERNAME "/feeds/switch";

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
//int x=0;
int y=1;
int i=1;

void setup() {
    pinMode(0, INPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
 //pinMode(13,OUTPUT);
  EEPROM.begin(512);
Serial.begin(115200);
  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
  wifiManager.autoConnect("Pranay" , "password");
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  
  delay(10);
  dht.begin();
  Serial.println(F("Welcome Pranay"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
   Serial.println(WiFi.SSID());
  Serial.println();
  Serial.print("Passphrase");
  Serial.println();
  Serial.print(WiFi.psk());
   

//  WiFi.begin(WiFi.SSID(), WiFi.psk());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for me feed.
  mqtt.subscribe(&switch);
 // pinMode(0, INPUT);
  //pinMode(12,OUTPUT);
  //pinMode(13,OUTPUT);
 //pinMode(13,OUTPUT);
}

uint32_t x=0;

void loop() {
  
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &switch) {
      Serial.print(F("Got: "));
     //digitalWrite(13,HIGH);
     //delay(500);

      Serial.println((char *)switch.lastread);
      uint16_t num = atoi((char *)switch.lastread);
      digitalWrite(12,num);
      
      //digitalWrite(13,LOW);
           
  }
  

  }
   

  // Now we can publish stuff!

// Reading temperature and humidity 
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) ) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");

   // Now we can publish stuff!
  Serial.print(F("\nSending temperature val "));
  Serial.print(t);
  Serial.print("...");
  if (! temp.publish(t)) {
    Serial.println(F("Failed"));
  } else {
    
  Serial.print(F("\     humidity val "));
  Serial.print(h);
  Serial.print("...");
  if (! humidity.publish(h)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
}
  
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  
  /*if(! mqtt.ping(60)) {
    mqtt.disconnect();
  }*/
    buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      
      Serial.println("on");
   digitalWrite(13,HIGH);
    } else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
      digitalWrite(13,LOW);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;

  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
