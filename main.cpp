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

byte fakePos[1] = {1};

bool _BLEClientConnected = false;

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);

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
  BLEDevice::init("FootpodMimic");
  // CBLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Do some BLE Setup
  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
  RSCDescriptor.setValue("Send all your RCSM rubbish here");
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);


  pServer->getAdvertising()->addServiceUUID(RSCService);

  pRSC->start();

  pServer->getAdvertising()->start();
}

void poop() {
  //this gets called when the speed sensor rises
counter++;
Serial.print(counter);
}

void setup() {
  Serial.begin(115200);
Serial.print("started");
//Init speed sensor
  pinMode(15, INPUT);
  attachInterrupt(digitalPinToInterrupt(15),poop,RISING);
//Init BLE
  InitBLE();

//some unrequired rubbish
  kmph = 8;
  mps = kmph/3.6;


}



void loop() {
  // determine my wheel revolutions
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm = (counter/20)*60;
            counter = 0;
            previousMillis += 1000;
  }

//Calculate my speed
  Serial.print("rpm: ");
  Serial.print(rpm);
    mps = (rpm*.21)/60;
  Serial.print("Speed(mps): ");
  Serial.print(mps);
  Serial.print("Speed(kmph): ");
  kmph=mps*3.6;
  Serial.print(kmph);

//get speed ble ready
  inst_speed =(256/3.6)*kmph;

//Create the bytearray to send to Zwift via BLE
  byte charArray[10] = {
      3,
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
      (unsigned byte)inst_cadence,
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8),
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24)};



  RSCMeasurementCharacteristics.setValue(charArray,10);

  RSCMeasurementCharacteristics.notify();

  sensorPositionCharacteristic.setValue(fakePos, 1);


  delay(1000);
}
