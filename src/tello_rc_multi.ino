
//#include <MPU6050.h>

#include "MPU6050_6Axis_MotionApps20.h"
#include <string>
#include "Tello.h"
#include <math.h>
//#include "driver/adc.h" // ESP32 ADC library
#include <Wire.h> // I2C and SPI Library
#include <Adafruit_Sensor.h> // sensor library
#include <Adafruit_BMP280.h> // specific BME library
#include <AsyncMqttClient.h> // Asyncronous MQTT Client lib
#include "passwords.h" // Passwords file, doesnt go to git
#include <WiFi.h> // Wifi library



#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
 
#define M_PI 3.14159265359


#define ANALOGPIN1 ADC1_CHANNEL_4
#define ANALOGPIN2 ADC1_CHANNEL_5
#define JOYSTICK_SW 25
#define INTERRUPT_PIN 18

#define FLYLED 14
#define THRESLED 12
#define CONNECTEDLED 13

#define INTERVAL_P_D 1000
#define INTERVAL_P_S 10
#define INTERVAL_C_D 100
#define INTERVAL_U_M 100

#define MQTT_HOST IPAddress(10, 10, 0 ,173)
#define MQTT_PORT 1883
#define MQTT_PUB_Y "esp32/rcu/y"
#define MQTT_PUB_T "esp32/rcu/t"
#define MQTT_PUB_R "esp32/rcu/r"
#define MQTT_PUB_P "esp32/rcu/p"

float pitchAcc, rollAcc;
TimerHandle_t mqttReconnectTimer; // Timer for mqtt connection
TimerHandle_t wifiReconnectTimer; // TImer for wifi connection
AsyncMqttClient mqttClient;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int16_t accX, accY, accZ, temp, gyroX, gyroY, gyroZ;
int error = 0; // Error in I2C Setup
short headless = 0;

// Potentiometer Globals
int sensor1_value = 0; // Pan potentiometer value
int sensor2_value = 0; // Tilt potentiometer value

// TODO; Could be calculated by taking the range of the adc too, to be done, too tired
const int joystickoffset = -40;
const int joystickdivider = 10;

short x_default, x;
short y_default, y;
const long interval = 50; // Wait interval between measurements
short switchiterations = 0;
short laststate = 1;
volatile unsigned long action_time;

// Drone Globals
byte flying = 0;
byte change_state = 0;

byte mqtt_connected = 0;
Tello tello;

// BMP Globals
Adafruit_BMP280 bmp; // Adafruit BMP Instance

// Wifi Globals
//const char * networkName = "TELLO-598F72";//Replace with your Tello SSID
//const char * networkPswd = "";
boolean connected = false;

// Gyro Globals
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

const int MPU6050_addr = 0x68;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat gravity;    // [x, y, z]            gravity vector
float r, p, bat;


unsigned long now, lpd = 0, lps = 0, lcd = 0, lum = 0;

MPU6050 mpu;

void connectToWiFi() {
  if (!headless) Serial.println("Connecting to WiFi network: " + String(ssid));
  //WiFi.disconnect(true);
  //WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  if (!headless) Serial.println("Waiting for WIFI connection...");
}
void connectToMqtt() {
  if (!headless) Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event)
  {
    case SYSTEM_EVENT_STA_GOT_IP:
      //When connected set
      if (!headless) {
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());
      }
      connectToMqtt();
      //tello.init();
      connected = true;
      digitalWrite(CONNECTEDLED, connected);
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      if (!headless) Serial.println("WiFi lost connection");
      connected = false;
      xTimerStop(mqttReconnectTimer, 0); // make we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0); // Start trying to reconnect to the wifi
      digitalWrite(CONNECTEDLED, connected);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  mqtt_connected = 1;
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("MQTT disconnected");
  mqtt_connected = 0;

  if (WiFi.isConnected()) {
    // Start timer if Wifi is present
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  // Catch QoS ackknowledges
  /* Serial.print("Acked  packetId: ");
    Serial.println(packetId);*/
}


void read_mpu_data(){ 
 
  //Read the raw gyro and accelerometer data
 
  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  accX = Wire.read()<<8|Wire.read();                                  
  accY = Wire.read()<<8|Wire.read();                                  
  accZ = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyroX = Wire.read()<<8|Wire.read();                                 
  gyroY = Wire.read()<<8|Wire.read();                                 
  gyroZ = Wire.read()<<8|Wire.read();                                 
}

void setup()
{
  // Setup Serial
  if (!headless) Serial.begin(115200);

  // Setup pins
  pinMode(FLYLED, OUTPUT); // Input PullUp For switch
  pinMode(THRESLED, OUTPUT); // Input PullUp For switch
  pinMode(CONNECTEDLED, OUTPUT); // Input PullUp For switch

  // Setup i2c
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Setup MPU
  if (!headless) Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  //pinMode(INTERRUPT_PIN, INPUT);
  if (!headless) {
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  }
  delay(20);

  devStatus = mpu.dmpInitialize();
  /*
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    if (!headless) Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    if (!headless) Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    if (!headless) {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }*/
   for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyroX;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyroY; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyroZ; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }

  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
  //

  //Connect to the WiFi network
  connectToWiFi();

  // Setup sw
  action_time = millis();
  pinMode(JOYSTICK_SW, INPUT_PULLUP); // Input PullUp For switch

  // Setup ADC
  adc1_config_width(ADC_WIDTH_BIT_10); // Configure ADC for 10 Bit
  adc1_config_channel_atten(ANALOGPIN1, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_10);
  adc1_config_channel_atten(ANALOGPIN2, ADC_ATTEN_DB_11);

  // Setup BMP
  //
  //  if (bmp.begin(0x77)) {
  //    if(!headless) Serial.println("Successful setup");
  //  }
  //
  //  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //              Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //              Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //              Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //              Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  //
  // Setup Joystick
  if (!headless) Serial.println("Getting normal Joystick position");
  x_default = adc1_get_raw(ANALOGPIN1);
  y_default = adc1_get_raw(ANALOGPIN2);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));

  // Register WifiEvent
  WiFi.onEvent(WiFiEvent);

  // Register mqtt listeners
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);

  // Set Mqtt config
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(mqtt_user, mqtt_pass);

  now = millis();
}

void loop() {

  now = millis();
  //if(!connected) return;
  if ( now - lpd > INTERVAL_P_D ) {
    polling_drone();
    lpd = millis();
  }
  if ( now - lps > INTERVAL_P_S ) {
    lps = millis();
    polling_sensors();
  }

  if ( now - lum > INTERVAL_U_M ) {
    lum = millis();
    update_mqtt();
  }

  if ( now - lcd > INTERVAL_C_D ) {
    lcd = millis();
    update_drone();
  }
}

void polling_sensors() {

  //Serial.println("poll sensors");

  if (digitalRead(JOYSTICK_SW) != 1) {
    switchiterations += 1;

    if ((switchiterations * INTERVAL_P_S * 0.001) == 1) {
      // Not meant to be here but emergency
      Serial.print("Turn off");
      if (tello.turnOff()) {
        if (!headless) Serial.println("Turing off");
        flying = 0;
        digitalWrite(FLYLED, flying);
      }
    }

    if (laststate == 0) {
      // Do nothing, button pressed
    } else {
      if (flying == 0) { // Flying should not be changed by the controller, only by the drone
        change_state = 1;
        Serial.println("Go fly");
      } else {
        change_state = 1;
        Serial.println("Go die");
      }
      action_time = millis();

    }
    laststate = 0;
  } else {
    switchiterations = 0;
    laststate = 1;
  }

  x =  adc1_get_raw(ANALOGPIN1) - x_default;
  y =  adc1_get_raw(ANALOGPIN2) - y_default;
  x = (x + joystickoffset) / joystickdivider;
  y = (y + joystickoffset) / joystickdivider;

  /* // Using DMP
    if (!dmpReady) return;
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    r = ypr[2] * 180/M_PI;
    p = ypr[1] * 180/M_PI;
  */

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true);

  read_mpu_data();
  
  /*  Serial.print("AccX = "); Serial.print(AccX);
    Serial.print(" || AccY = "); Serial.print(AccY);
    Serial.print(" || GyroX = "); Serial.print(GyroX);
    Serial.print(" || GyroY = "); Serial.println(GyroY);
  */

  ComplementaryFilter();
  Serial.print(r);
  Serial.print("\t");
  Serial.println(p);
  /*

  if (abs(r) > 45.0) {
    if (r > 0) r = 45.0;
    if (r < 0) r = -45.0;
  }
  if (abs(p) > 45.0) {
    if (p > 0) p = 45.0;
    if (p < 0) p = -45.0;
  }*/
}

void polling_drone() {


  bat = tello.getBattery();
  //Serial.println("poll drone");

}

void update_mqtt() {
  if (mqtt_connected) {
    /*Serial.print(x);
      Serial.print(" is x over ");
      Serial.print(MQTT_PUB_PRES);
      Serial.println(" published");*/
    Serial.println("published");
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_Y, 1, true, String(x).c_str());
    packetIdPub1 = mqttClient.publish(MQTT_PUB_T, 1, true, String(y).c_str());
    packetIdPub1 = mqttClient.publish(MQTT_PUB_R, 1, true, String(r).c_str());
    packetIdPub1 = mqttClient.publish(MQTT_PUB_P, 1, true, String(p).c_str());
  }
}

void update_drone() {
  if (change_state == 1) {
    if (flying == 1) {
      Serial.println("Commence landing");
      flying = 0;
      change_state = 0;
      if (tello.land()) {
        flying = 0;
        change_state = 0;

      }
    } else {
      Serial.println("Commence Starting");
      flying = 1;
      if (tello.takeoff()) {
        flying = 1;
        change_state = 0;
      }
      change_state = 0;

    }
    return;
  }
  if ((abs(x) < 5) && (abs(y) < 5) && (abs(r) < 5) && (abs(p) < 5)) {
    digitalWrite(THRESLED, 0);
  } else {
    digitalWrite(THRESLED, 1);
    //        send_curr_to_serial(r, p, x, y);
    tello.sendRCcontrol(r, p, x, y);
  }
}

//void send_curr_to_serial(float r, float p, float x, float y) {
//  if(!headless) {
//    Serial.print("rpty\t");
//    Serial.print(r);
//    Serial.print("\t");
//    Serial.print(p);
//    Serial.print("\t");
//    Serial.print(x);
//    Serial.print("\t");
//    Serial.println(y);
//  }
//}


void ComplementaryFilter()
{
               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    p += ((float)gyroX / GYROSCOPE_SENSITIVITY) * INTERVAL_P_S; // Angle around the X-axis
    r -= ((float)gyroY / GYROSCOPE_SENSITIVITY) * INTERVAL_P_S;    // Angle around the Y-axis
 

    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accX) + abs(accY);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
  // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accX, (float)accY) * 180 / M_PI;
        p = p * 0.98 + pitchAcc * 0.02;
 
  // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accX, (float)accZ) * 180 / M_PI;
        r = r * 0.98 + rollAcc * 0.02;
    }
} 
