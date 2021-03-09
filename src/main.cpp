/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
#include <Wire.h>
#include <MPU6050_light.h>
#include <Kalman.h>

//MPU6050
MPU6050 mpu(Wire);
float pitch = 0.0f, roll = 0.0f;
uint32_t dt = 0;
uint64_t mpuTimer = 0;
uint32_t const mpuInterval = 100; // 10Hz entspricht dem Vorschlag von Prof. Dahlkemper
float offset_x = 0, offset_y = 0;
void calibrateXY();

//Kalman Filter
Kalman kalmanPitch = Kalman();	
Kalman kalmanRoll = Kalman();	
void calcKalman();

void setup(){
	// Initilize hardware serial:
	Serial.begin(115200); //USB-Serial
	Wire.begin();

	//Connecting to MPU6050
	while(mpu.begin()) {
	  delay(1000);
	}

	mpu.calcGyroOffsets();
	mpu.calcAccOffsets();

  calibrateXY(); // Measures Offsets

	// initialise Kalman filter
	kalmanPitch.setAngle(mpu.getAngleX());
	kalmanRoll.setAngle(mpu.getAngleY());

  mpu.update();
	//initialise time values
	mpuTimer = dt = millis();

}



void loop(){
  if(millis() - mpuTimer > mpuInterval) {
    calcKalman();
  }
	yield();
}


void calibrateXY() {
  int n = 10;
	offset_x = 0, offset_y = 0;
	
	for(int i = 0; i < n; ++i) {
		offset_x += mpu.getAngleX();
		offset_y += mpu.getAngleY();
		delay(100);
	}
	offset_x /=n;
	offset_y /=n;
}

void calcKalman() {
	// Calculate pitch and roll
	if((millis() - mpuTimer) >= mpuInterval)
	{
		mpuTimer = millis();
		dt = millis() - dt;	
		// Filter implementation with kalman filter
		pitch = kalmanPitch.getAngle(mpu.getAccAngleX(), mpu.getGyroX(), dt/1000.0f);
		roll = kalmanRoll.getAngle(mpu.getAccAngleY(), mpu.getGyroY(), dt/1000.0f);
		Serial.printf("%ld, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", millis(), pitch, roll, mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
		mpu.update();
	}
}