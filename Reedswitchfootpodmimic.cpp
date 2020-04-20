#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

bool        is_inst_stride_len_present = 1;                                 /**< True if Instantaneous Stride Length is present in the measurement. */
bool        is_total_distance_present = 1;                                  /**< True if Total Distance is present in the measurement. */
bool        is_running = 1;                                                 /**< True if running, False if walking. */
uint16_t    inst_speed = 40;                                                 /**< Instantaneous Speed. */
uint8_t     inst_cadence = 1;                                               /**< Instantaneous Cadence. */
uint16_t    inst_stride_length = 1;                                         /**< Instantaneous Stride Length. */
uint32_t    total_distance = 10;

int encoder = 2;
volatile unsigned int counter;
int rpm;

float kmph;
float mps;
//byte rscmArray[8] = {0,0,1,60,1,1,0,0};
//byte rscmArray[4] = {0b000001,1,1,30};
//byte rscmArray[12] = {0b000001,0x0136,1,30};
//byte rscmArray[12] = {0b000001,10,10,10}; This goes to 36km
byte rscmArray[12] = {0b000001,10,10,10};
byte fakePos[1] = {1};

bool _BLEClientConnected = false;

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);
//BLECharacteristic sensorFeatureCharacteristic(BLEUUID((uint16_t)0x2A54), BLECharacteristic::PROPERTY_READ);
BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("Footpodmimic_D03");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
  RSCDescriptor.setValue("Rate from 0 to 200");
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  pServer->getAdvertising()->addServiceUUID(RSCService);

  pRSC->start();
  // Start advertising
  pServer->getAdvertising()->start();
}

void poop() {
counter++;
//Serial.print("counter called:");
Serial.print(counter);
}

void setup() {
  Serial.begin(115200);
  //Serial.println("Start");
Serial.print("started");
  pinMode(15, INPUT);
  attachInterrupt(digitalPinToInterrupt(15),poop,RISING);
  InitBLE();




//byte rscmArray[] = {is_inst_stride_len_present,is_total_distance_present,is_running,inst_speed,inst_cadence,inst_stride_length,total_distance};
  kmph = 8;
  mps = kmph/3.6;

  //rscmArray[1] = 240;
}



void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm = (counter/20)*60;
            counter = 0;
            previousMillis += 1000;
  }
  //1 rev =21cm
  //mps = (rmp*.21)/60

  Serial.print("rpm: ");
  Serial.print(rpm);
    mps = (rpm*.21)/60;
  Serial.print("Speed(mps): ");
  Serial.print(mps);
  Serial.print("Speed(kmph): ");
  kmph=mps*3.6;
  Serial.print(kmph);

  //int speedms = 300;
  //rscmArray[2] = 255;
  //rscmArray[1] = 255;
  //rscmArray[0] = 2;
  //rscmArray[2] = (byte)msmps;

  //rscmArray[1] =rscmArray[1]+12;
//rscmArray[1] =km;

  //Serial.println(rscmArray[1]);

//  bool        is_inst_stride_len_present = 1;                                 /**< True if Instantaneous Stride Length is present in the measurement. */
  //bool        is_total_distance_present = 1;                                  /**< True if Total Distance is present in the measurement. */
  //bool        is_running = 1;                                                 /**< True if running, False if walking. */
  //uint16_t    inst_speed = 40;                                                 /**< Instantaneous Speed. */
  //uint8_t     inst_cadence = 1;                                               /**< Instantaneous Cadence. */
  //uint16_t    inst_stride_length = 1;                                         /**< Instantaneous Stride Length. */
  //uint32_t    total_distance = 10;*/
//256=1 mps

  inst_speed =(256/3.6)*kmph;

  byte charArray[10] = {
      3,
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
      (unsigned byte)inst_cadence,
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8),
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24)};



/**
  const unsigned char charArray[12] = {
      3,1,1,
      (unsigned char)inst_speed, (unsigned char)(inst_speed >> 8),
      (unsigned char)inst_cadence,
      (unsigned char)inst_stride_length, (unsigned char)(inst_stride_length >> 8),
      (unsigned char)total_distance, (unsigned char)(total_distance >> 8), (unsigned char)(total_distance >> 16), (unsigned char)(total_distance >> 24)};
**/

  //  cscChar.setValue(charArray, 11);
  //(unsigned char)wheelRev, (unsigned char)(wheelRev >> 8), (unsigned char)(wheelRev >> 16), (unsigned char)(wheelRev >> 24),

  RSCMeasurementCharacteristics.setValue(charArray,10);
  //RSCMeasurementCharacteristics.setValue(rscmArray,sizeof(rscmArray));
  RSCMeasurementCharacteristics.notify();

  sensorPositionCharacteristic.setValue(fakePos, 1);
  //mps++;
  //km=0.277;
  //km++;

  delay(1000);
}
