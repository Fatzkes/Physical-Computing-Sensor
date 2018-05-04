#include <Basecamp.hpp>

// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain


//Includes
//For MQTT Usage
#include <Wire.h>    //For I2C Usage
//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//Include the Header of Jeff Rowberg
//#include<MPU6050.h>
// Defines for easy Use
#define MPU_ADDRESS_GYRO_CONFIG 0x1B
#define MPU_ADDRESS_ACCEL_CONFIG 0x1C
#define MPU_ADDRESS_GYX 0x43
#define MPU_ADDRESS_GYY 0x45
#define MPU_ADDRESS_GYZ 0x47
#define MPU_ADDRESS_ACX 0x3B
#define MPU_ADDRESS_ACY 0x3D
#define MPU_ADDRESS_ACZ 0x3F
#define MPU_ADDRESS_TEMP 0x41

#define MPU_ADDRESS_AD0_GND 0x68
#define MPU_ADDRESS_AD0_VCC 0x69
#define MPU_ADDRESS_PWR_MGMT_1 0x6B
//Defines for FS_SEL
#define FS_SEL_0 0x0
#define FS_SEL_1 0x1
#define FS_SEL_2 0x2
#define FS_SEL_3 0x3



int16_t RawAcX,RawAcY,RawAcZ,RawTmp,RawGyX,RawGyY,RawGyZ;
float PhysAcX,PhysAcY,PhysAcZ,PhysTmp,PhysGyX,PhysGyY,PhysGyZ;
int16_t deggyrox;
int16_t deggyroy;
int16_t stepNull, stepOne, stepTwo, stepThree, stepFour , stepSizeY, stepSizeX;
long time_1,time_2;
Basecamp iot;
String subTopic;
String pubTopic;
int MEINE_LED = 21;
void setup()
{
  pinMode(MEINE_LED, OUTPUT);
  iot.begin();
  iot.configuration.set("MQTTPort", "1883"); // Broker:
  iot.configuration.set("MQTTHost", "192.168.XXX.YYY");
  iot.configuration.set("MQTTUser", "");
  iot.configuration.set("MQTTPass", "");
  iot.configuration.set("WifiEssid", "medinf");
  iot.configuration.set("WifiPassword", "XXXXXXXXX");
  iot.configuration.set("WifiConfigured", "True");
  iot.configuration.set("DeviceName", "esp32us"); // ändern
  iot.configuration.save(); // nur wenn cfg. geändert wurde
  iot.mqtt.onConnect(mqttConnected);
  iot.mqtt.onSubscribe(mqttSubscribed);
  iot.mqtt.onMessage(mqttOnMessage);
  //Setup Registers of the GY-521
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS_AD0_GND);
  Wire.write(MPU_ADDRESS_PWR_MGMT_1);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Setup Registers of the Accelerometer
  Wire.beginTransmission(MPU_ADDRESS_AD0_GND);
  Wire.write(MPU_ADDRESS_ACCEL_CONFIG);  // PWR_MGMT_1 register
  Wire.write(FS_SEL_0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Setup Registers of the Gyroscope
  Wire.beginTransmission(MPU_ADDRESS_AD0_GND);
  Wire.write(MPU_ADDRESS_GYRO_CONFIG);  // PWR_MGMT_1 register
  Wire.write(FS_SEL_0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Begin Communcation with Serial Monitor
  Serial.begin(115200);
  stepSizeY = 2000;
  stepSizeX = 2000;  
}
void loop()
{
  time_1=ReadRawGyroData();
  printDegData(time_1);
  RawToPhysData();
  standartReadout(time_1);
  // hier z.B. regelmäßig Sensordaten publishen
  //iot.mqtt.publish(pubTopic.c_str(), 1, true, "up" );

  delay(10000);
}

void standartReadout(long time)
{
  Serial.print("AcX = "); Serial.print(RawAcX);
  Serial.print(" | AcY = "); Serial.print(RawAcY);
  Serial.print(" | AcZ = "); Serial.println(RawAcZ);
  Serial.print(" | Tmp = "); Serial.print(RawTmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" |DegreeX = "); Serial.print(deggyrox);
  Serial.print(" | RawGyX = "); Serial.print(RawGyX);
  Serial.print(" | PhysGyX = "); Serial.print(PhysGyX);
  //Serial.print(" |Time = "); Serial.print(time);
  //Serial.print(" | DegreeY = "); Serial.print(deggyroy);
  Serial.print(" | RawGyY = "); Serial.print(RawGyY);
  Serial.print(" | PhysGyY = "); Serial.print(PhysGyY);
  Serial.print(" | RawGyZ = "); Serial.println(RawGyZ);
  Serial.print(" | PhysGyZ = "); Serial.print(PhysGyZ);
}
long ReadRawGyroData()
{
  Wire.beginTransmission(MPU_ADDRESS_AD0_GND);
  Wire.write(MPU_ADDRESS_GYX);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS_AD0_GND,6,true);  // request a total of 14 registers
  long time = micros();
  RawGyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  RawGyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  RawGyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  return(time);
}
void printDegData(long int time)
{
  deggyrox=((RawGyX)/2)*time;
  deggyroy=((RawGyY)/2)*time;
}
long ReadRawAccData()
{
  Wire.beginTransmission(MPU_ADDRESS_AD0_GND);
  Wire.write(MPU_ADDRESS_ACX);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS_AD0_GND,6,true);  // request a total of 14 registers
  long time = micros();
  RawAcX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  RawAcY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  RawAcZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  return(time);
}
void ReadRawTempData()
{
  Wire.beginTransmission(MPU_ADDRESS_AD0_GND);
  Wire.write(MPU_ADDRESS_TEMP);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS_AD0_GND,1,true);  // request a total of 14 registers
  RawTmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
}
void RawToPhysData() //Umrechnung von Rohdaten in Physikalische Daten
{
  PhysAcX=RawAcX/16384.;
  PhysAcY=RawAcY/16384.;
  PhysAcZ=RawAcZ/16384.;

  PhysGyX=RawGyX/131.;
  PhysGyY=RawGyY/131.;
  PhysGyZ=RawGyZ/131.;
}
void mqttConnected(bool sessionPresent) 
{
  Serial.println("MQTT verbunden!");
  subTopic = iot.hostname + "/roteLED";
  pubTopic = iot.hostname + "/status";
  iot.mqtt.subscribe(subTopic.c_str(), 2);
  iot.mqtt.publish(pubTopic.c_str(), 1, true, "online");
}

void mqttSubscribed(uint16_t packetId, uint8_t qos) 
{
  Serial.println("Abonnement erfolgreich");
}
void mqttOnMessage(char* topic, char* payload,AsyncMqttClientMessageProperties properties, size_t len,size_t index, size_t total) 
{
  Serial.println("Neue MQTT-Nachricht:");
  Serial.print("Topic:");
  Serial.println(topic);
  Serial.print("Payload:");
  Serial.println(payload);
  if (strcmp(payload,"an")==0)
  digitalWrite(MEINE_LED, HIGH);
  else
  digitalWrite(MEINE_LED, LOW);
  iot.mqtt.publish(pubTopic.c_str(), 1, true, payload);
}
void mqttPublished(uint16_t packetId) 
{
  Serial.println("MQTT-Nachricht veröffentlicht");
}
