/*
Author: Nolan Gilley
License: CC-BY-SA, https://creativecommons.org/licenses/by-sa/2.0/
Date: 7/30/2016
File: laundry_esp8266.ino
This sketch is for a NodeMCU wired up with 2 accelerometers and
2 reed swithches.  An MPU6050 and a reed switch are attached to
both the wash and dryer.  This sketch will send mqtt messages to
the defined topics which describe the state of the washer and dryer
and the details of the accelerometer data.

1) Update the WIFI ssid and password below
2) Update the thresholds for washer and dryer movement

The thresholds are also configurable over mqtt!

The ESP8266 will publish mqtt commands to:
sensor/dryer/(EMPTY,RUNNING,COMPLETE)
sensor/washer/(EMPTY,RUNNING,COMPLETE)
sensor/dryer/detail: dryer %d, %d, %d, %d, %d", dryer_door_status, dryer_state, dryer_ax_range, dryer_ay_range, dryer_az_range
sensor/washer/detail: dryer %d, %d, %d, %d, %d", washer_door_status, washer_state, washer_ax_range, washer_ay_range, washer_az_range

And it SUBSCRIBES to:

sensor/dryer/set/ax
Ax value for dryer (int)
sensor/dryer/set/ay
Ay value for dryer (int)
sensor/dryer/set/az
Az value for dryer (int)
sensor/washer/set/ay
Ay value for washer (int)
sensor/dryer/set/detected
# of dryer detections out of 15 cycles
sensor/washer/set/detected
# of washer detections of of 15
sensor/dryer/set/detail
0 or 1 (off/on)
sensor/washer/set/detail
0 or 1

*/

// MPU6050 Includes
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// ESP8266 Includes
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// DEFINE NODEMCU PINS
#define D0 16
#define D1 5 // I2C Bus SCL (clock)
#define D2 4 // I2C Bus SDA (data)
#define D3 0
#define D4 2 // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO 
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS)
#define D9 3 // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

// DEFINE GPIO PINS
#define DRYER_DOOR D7 // number of reed sensor for dryer
#define WASHER_DOOR D8 // number of reed sensor for dryer

// DEFINE STATES
#define CLOSED 0  //washer/dryer door closed
#define OPEN 1 //washer/dryer door open
#define EMPTY 0 //washer/dryer empty
#define RUNNING 1 //washer/dryer running
#define COMPLETE 2 //washer/dryer complete
#define MOVEMENT_DETECTED 0
#define MOVEMENT_NOT_DETECTED 1

// ESP8266 WIFI  ----------------------------------------------------------------
const char* ssid = "Internet";
const char* password = "xxx";

const char* mqtt_server = "192.168.1.52";
const char* mqtt_username = "nolan";
const char* mqtt_password = "xxx";
const char* mqtt_topic_dryer_detail = "sensor/dryer/detail";
const char* mqtt_topic_washer_detail = "sensor/washer/detail";
const char* mqtt_topic_washer = "sensor/washer";
const char* mqtt_topic_dryer = "sensor/dryer";

WiFiClient espClient;
PubSubClient client(espClient);
// END ESP8266 WIFI  ------------------------------------------------------------


// MPU-6050 Accelerometers ------------------------------------------------------
MPU6050 MPU_DRYER(0x68); //DRYER
MPU6050 MPU_WASHER(0x69); //WASHER

int16_t dryer_ax, dryer_ay, dryer_az;
int16_t dryer_gx, dryer_gy, dryer_gz;
int16_t dryer_ax_min = 0, dryer_ax_max = 0, dryer_ax_range;
int16_t dryer_ay_min = 0, dryer_ay_max = 0, dryer_ay_range;
int16_t dryer_az_min = 0, dryer_az_max = 0, dryer_az_range;

int16_t washer_ax, washer_ay, washer_az;
int16_t washer_gx, washer_gy, washer_gz;
int16_t washer_ax_min = 0, washer_ax_max = 0, washer_ax_range;
int16_t washer_ay_min = 0, washer_ay_max = 0, washer_ay_range;
int16_t washer_az_min = 0, washer_az_max = 0, washer_az_range;
// end MPU-6050-------------------------------------------------------------------



// accelerometer sensor 1 (Dryer) ===============================================
int dryer_reading = 0; //reading = 1 mean no noise, 0=noise
int dryer_reading_previous = 0;
int dryer_state = EMPTY;  //1 = running, 0 = empty, 2 = complete
int last_dryer_state = EMPTY;

unsigned long sample_time = 0;   //millis of last reading
int dryer_detector_count = 0;   //number of captures
int dryer_detected_count = 0; //number of readings showing sound

int dryer_door_status = 0;

// accelerometer sensor 2 (Washer) ==============================================
int washer_reading = 0; //reading = 1 mean no noise, 0=noise
int washer_reading_previous = 0;
int washer_state = EMPTY;  //1 = running, 0 = empty, 2 = complete
int last_washer_state = EMPTY;

int washer_detector_count = 0;   //number of captures
int washer_detected_count = 0; //number of readings showing sound

int washer_door_status = 0;

char dryerString[50];
char washerString[50]; 

// CONFIGURABLE THRESHOLDS
int dryer_ax_threshold = 17500;
int dryer_ay_threshold = 2000;
int dryer_az_threshold = 3300;

int washer_ay_threshold = 18000;

int dryer_detected_threshold = 9;
int washer_detected_threshold = 3;
int dryer_detailed_reporting = 0;
int washer_detailed_reporting = 0;

void setup()
{
  Serial.begin(115200); // setup serial

  // setup WiFi
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();
  MPU_DRYER.initialize();
  MPU_WASHER.initialize();
  pins_init();
  reconnect();
  update_via_mqtt();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if (strcmp(topic, "sensor/washer/set/ay") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    washer_ay_threshold = i;
    Serial.print("washer ay set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/dryer/set/ax") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    dryer_ax_threshold = i;
    Serial.print("dryer ax set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/dryer/set/ay") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    dryer_ay_threshold = i;
    Serial.print("dryer ay set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/dryer/set/az") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    dryer_ax_threshold = i;
    Serial.print("dryer az set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/dryer/set/detected") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    dryer_detected_threshold = i;
    Serial.print("dryer detected threshold set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/washer/set/detected") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    washer_detected_threshold = i;
    Serial.print("washer detected threshold set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/washer/set/detail") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    washer_detailed_reporting = i;
    Serial.print("washer detailed reporting set to ");
    Serial.println(i);
  }
  else if (strcmp(topic, "sensor/dryer/set/detail") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    int i= s.toInt();
    dryer_detailed_reporting = i;
    Serial.print("dryer detailed reporting set to ");
    Serial.println(i);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
//    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
//      Serial.println("connected");
      client.subscribe("sensor/washer/set/+");
      client.subscribe("sensor/dryer/set/+");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
  last_dryer_state = dryer_state;
  last_washer_state = washer_state;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //===================================================================
  //dryer/washer door reed switches
  // 1 = door open, 0 = door closed
  dryer_door_status = digitalRead(DRYER_DOOR);
  washer_door_status = digitalRead(WASHER_DOOR);

  // if dryer door is opened and dryer cycle is complete
  if (dryer_door_status == OPEN and dryer_state == COMPLETE)
  {
      dryer_state = EMPTY; // set dryer to empty
  }

  // if washer door is open and washer cycle is complete
  if (washer_door_status == OPEN and washer_state == COMPLETE)
  {
      washer_state = EMPTY; //set washer to empty
  }
  
  MPU_DRYER.getMotion6(&dryer_ax, &dryer_ay, &dryer_az, &dryer_gx, &dryer_gy, &dryer_gz);
  MPU_WASHER.getMotion6(&washer_ax, &washer_ay, &washer_az, &washer_gx, &washer_gy, &washer_gz);
  trackMinMax(dryer_ax, &dryer_ax_min, &dryer_ax_max);
  trackMinMax(dryer_ay, &dryer_ay_min, &dryer_ay_max);
  trackMinMax(dryer_az, &dryer_az_min, &dryer_az_max);
  trackMinMax(washer_ax, &washer_ax_min, &washer_ax_max);
  trackMinMax(washer_ay, &washer_ay_min, &washer_ay_max);
  trackMinMax(washer_az, &washer_az_min, &washer_az_max);
//  deal with millis rollover
  if (sample_time > millis())
  {
    sample_time = millis();
  }

  // samples every 5 sec
  if ((millis() - sample_time) > 5000)
  {
    sample_time = millis();    //reset sample_time to wait for next Xms

    // Calculate Range for each accelerometer direction.
    dryer_ax_range = dryer_ax_max - dryer_ax_min;
    dryer_ay_range = dryer_ay_max - dryer_ay_min;
    dryer_az_range = dryer_az_max - dryer_az_min;
    washer_ax_range = washer_ax_max - washer_ax_min;
    washer_ay_range = washer_ay_max - washer_ay_min;
    washer_az_range = washer_az_max - washer_az_min;

    // Reset Range Counters
    dryer_ax_min = 0, dryer_ax_max = 0;
    dryer_ay_min = 0, dryer_ay_max = 0;
    dryer_az_min = 0, dryer_az_max = 0;
    washer_ax_min = 0, washer_ax_max = 0;
    washer_ay_min = 0, washer_ay_max = 0;
    washer_az_min = 0, washer_az_max = 0;

if (dryer_detailed_reporting == 1) {
  sprintf(dryerString, "dryer %d, %d, %d, %d, %d", dryer_door_status, dryer_state, dryer_ax_range, dryer_ay_range, dryer_az_range);
  client.publish(mqtt_topic_dryer_detail, dryerString);
  Serial.println(dryerString);
}
if (washer_detailed_reporting == 1) {
  sprintf(washerString, "washer %d, %d, %d, %d, %d", washer_door_status, washer_state, washer_ax_range, washer_ay_range, washer_az_range);
  client.publish(mqtt_topic_washer_detail, washerString);
  Serial.println(washerString);
}

    if (abs(dryer_ax_range > dryer_ax_threshold) and abs(dryer_ay_range > dryer_ay_threshold) and abs(dryer_az_range > dryer_az_threshold))
    {
      dryer_reading = MOVEMENT_DETECTED;
    }
    if (abs(washer_ax_range) > washer_ay_threshold)
    {
      washer_reading = MOVEMENT_DETECTED;
    }

    dryer_detector_count = dryer_detector_count + 1;          //count how many times we listened
    if (dryer_reading == MOVEMENT_DETECTED)                    //count how many times detected movement
    {
      dryer_detected_count = dryer_detected_count + 1;
    }

    washer_detector_count = washer_detector_count + 1;          //count how many times we listened
    if (washer_reading == MOVEMENT_DETECTED)                    //count how many times detected movement
    {
      washer_detected_count = washer_detected_count + 1;
    }
    dryer_reading = MOVEMENT_NOT_DETECTED;        //reset
    washer_reading = MOVEMENT_NOT_DETECTED;        //reset

  }//end reading every 5 seconds

  if (dryer_detector_count >= 15)
  {
    if (dryer_door_status == CLOSED)
    {
      if (dryer_detected_count >= dryer_detected_threshold) dryer_state = RUNNING;
      else if (dryer_state == RUNNING) dryer_state = COMPLETE;
    }
    dryer_detector_count = 0;
    dryer_detected_count = 0;
  }

  if (washer_detector_count >= 15)
  {
    if (washer_door_status == CLOSED)
    {
      if (washer_detected_count >= washer_detected_threshold) washer_state = RUNNING;
      else if (washer_state == RUNNING and washer_door_status == CLOSED) washer_state = COMPLETE;
    }
    washer_detector_count = 0;
    washer_detected_count = 0;
  }

  if (last_dryer_state != dryer_state or last_washer_state != washer_state)
  {
    update_via_mqtt();
  }
 

}// end loop

void pins_init()
{
  pinMode(DRYER_DOOR, INPUT);
  pinMode(WASHER_DOOR, INPUT);
}

void update_via_mqtt()
{
  if (dryer_state == RUNNING) {
    client.publish(mqtt_topic_dryer, "Running", true);
  }
  else if (dryer_state == COMPLETE) {
    client.publish(mqtt_topic_dryer, "Complete", true);
  }
  else {
    client.publish(mqtt_topic_dryer, "Empty", true);
  }
  if (washer_state == RUNNING) {
    client.publish(mqtt_topic_washer, "Running", true);
  }
  else if (washer_state == COMPLETE) {
    client.publish(mqtt_topic_washer, "Complete", true);
  }
  else {
    client.publish(mqtt_topic_washer, "Empty", true);
  }
  Serial.println("mqtt published!");
}

int16_t trackMinMax(int16_t current, int16_t *min, int16_t *max)
{
  if (current > *max)
  {
    *max = current;
  }
  else if (current < *min)
  {
    *min = current;
  }
}
