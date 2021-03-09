#include <Kalman.h>


#include <MPU6050_light.h>
#include <Wire.h>

//Adafruit_BMP280 bmp;
//float temperature = 0.0f, altitude = 0.0f, pressure = 0.0f;



MPU6050 mpu(Wire);
float pitch = 0.0f, roll = 0.0f;
float accel, temp, gyro;
uint32_t dt = 0;
uint64_t mpuTimer = 0;
uint32_t const mpuInterval = 50;

Kalman kalmanPitch = Kalman();  
Kalman kalmanRoll = Kalman(); 


void setup() {
  // Initilize hardware serial:
  Serial.begin(115200);
  delay(4000);
  
  Serial.println("#RCU started....");
  Wire.begin();
  /*
  while (!bmp.begin(BMP280_ADDRESS_ALT)) {
    Serial.println("#Could not find a valid BMP280 sensor, check wiring!");
    delay(1000);
  }
  */
  
  Serial.println("#BMP280 verbunden!");
//  temperature = bmp.readTemperature();
//  pressure =    bmp.readPressure();
//  altitude =    bmp.readAltitude();
//  Serial.printf("#Temperature: %f\tPressure: %f\tAltitude: %f\n", temperature, pressure, altitude);


  //Connecting to MPU6050
  while(mpu.begin()) {
    Serial.println("#Could not find a valid MPU6050 sensor, check wiring!");
    delay(1000);
  }

  mpu.calcGyroOffsets();
  mpu.calcAccOffsets();
  Serial.printf("#MPU: Gyro 1:%f\t2:%f\n", mpu.getAccAngleX(), mpu.getAccAngleY());
  Serial.printf("#MPU: Accel 1:%f\t2:%f\n", mpu.getGyroX(), mpu.getGyroY());
  Serial.printf("#MPU: Temp %f\n", mpu.getTemp());
  mpu.update();

  // initialise Kalman filter
  kalmanPitch.setAngle(mpu.getAngleX());
  kalmanRoll.setAngle(mpu.getAngleY());

}

void loop() {
  // put your main code here, to run repeatedly:
  cntrl();
}



void cntrl() {


  if((millis() - mpuTimer) >= mpuInterval)
  {
    mpuTimer = millis();
    dt = millis() - dt; 
    // Filter implementation with kalman filter
    pitch = kalmanPitch.getAngle(mpu.getAccAngleX(), mpu.getGyroX(), dt/1000.0f);
    roll = kalmanRoll.getAngle(mpu.getAccAngleY(), mpu.getGyroY(), dt/1000.0f);
    // Filter implementation with complementary filter

    // Serial.printf("#Gyro %f\n", mpu.getGyroX());
    // Serial.printf("#Pitch: %f\n",pitch);
    // Serial.printf("#Roll: %f\n", roll);
    Serial.printf("%ld, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", millis(), pitch, roll, mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
    mpu.update();
  }
}
