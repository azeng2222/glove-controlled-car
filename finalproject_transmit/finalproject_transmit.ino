#include <bluefruit.h>

const uint16_t MANUFACTURER_ID = 0x1234;
uint8_t beaconPayload[3] = {0xBE, 0xAC, 0};
uint8_t flexSensor = 0;

void startBeacon(uint8_t flexSensor) {
  BLEAdvertising &adv = Bluefruit.Advertising;

  adv.clearData();

  // set last value of beaconPayload based on which flex sensor was detected
  beaconPayload[2] = flexSensor;

  // Build manufacturer data field manually:
  // [0] = low byte of manufacturer ID
  // [1] = high byte of manufacturer ID
  // [2..] = your custom payload
  uint8_t manData[2 + sizeof(beaconPayload)];
  manData[0] = MANUFACTURER_ID & 0xFF;
  manData[1] = (MANUFACTURER_ID >> 8) & 0xFF;
  memcpy(&manData[2], beaconPayload, sizeof(beaconPayload));

  adv.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
              manData,
              sizeof(manData));

  adv.setInterval(32, 244);  // ~20 ms
  adv.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);

  adv.start();
}

void stopBeacon() {
  Bluefruit.Advertising.stop();
}

void setup() {
  // Set the analog reference to 1.8V (default = 3.6V)
  analogReference(AR_INTERNAL_1_8);

  Serial.begin(115200);
  Bluefruit.begin();
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

void loop() {

  int flex1 = analogRead(A1);
  int flex2 = analogRead(A2);
  int flex3 = analogRead(A3);
  int flex4 = analogRead(A4);
  int flex5 = analogRead(A5);

  if (flex1 == 1023) {

    digitalWrite(LED_BLUE, HIGH);
    flexSensor = 0b1 | flexSensor;

    Serial.println("Flex sensor 1 bent - Starting beacon...");
    startBeacon(flexSensor);

  }
  
  if (flex2 == 1023) {

    digitalWrite(LED_RED, HIGH);
    flexSensor = 0b10 | flexSensor;

    Serial.println("Flex sensor 2 bent - Starting beacon...");
    startBeacon(flexSensor);

  } 
  
  if (flex1 < 1023 && flex2 < 1023) {
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, LOW);

    if (Bluefruit.Advertising.isRunning()){
      flexSensor = 0;
      Serial.println("Stopping beacon...");
      stopBeacon();
    }

  }

  delay(50);
}