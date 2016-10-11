#include <Adafruit_SleepyDog.h>

//==========================DS18B20==========================
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
//===========================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
unsigned long time = 0;
int resetPin=12;
void setup() {
  Serial.begin(115200);
  digitalWrite(resetPin,HIGH);
  pinMode(resetPin,OUTPUT);
//==========================DS18B20==========================
  sensors.begin();
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  sensors.setResolution(insideThermometer, 9);
//===========================================================
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    digitalWrite(INTERRUPT_PIN,HIGH);
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }
    pinMode(LED_PIN, OUTPUT);

    int countdownMS = Watchdog.enable();
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
  Serial.println();
}
//==========================DS18B20==========================
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print(F("T"));
  Serial.print(tempC);
  Serial.println(F("!"));
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print(F("0"));
    Serial.print(deviceAddress[i], HEX);
  }
}

void printDs18b20(){
    sensors.requestTemperatures(); // Send the command to get temperatures
    printTemperature(insideThermometer); // Use a simple function to print out the data
}
//===========================================================

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

int count = 0 ;
int count_mpu = 0 ;

void loop() {
    Watchdog.reset();

    //==========================DS18B20==========================
    if(count == 500){
      count = 0;
      printDs18b20();
//      time = millis();
    }
    count ++;
    //Serial.flush();
//    if(time > 40000){
//      digitalWrite(resetPin, LOW);
//    }
    //==========================================================
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        break;
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        if (mpuIntStatus == 0x13 && fifoCount == 1024) {
//Serial.print("Sta : ");
//Serial.print(mpuIntStatus, HEX);
//Serial.print(", ");
//Serial.println(fifoCount);
//          Serial.println("RESET");
          //delay(1000);
          // digitalWrite(resetPin, LOW);
        }
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
          Serial.println(fifoCount);
          Serial.println(packetSize);
        }
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            if(count_mpu > 1000){
            Serial.print(F("Z"));
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(F("X"));
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(F("Y"));
            Serial.print(ypr[2] * 180/M_PI);
            Serial.println(F("@"));
            }
            else {
              count_mpu++;
            }
        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
