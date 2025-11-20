#include <bluefruit.h>

const uint16_t MANUFACTURER_ID = 0x1234;
uint8_t beaconPayload[3] = {0xBE, 0xAC, 0};

unsigned long lastDetectionTime = 0;
const unsigned long DETECTION_TIMEOUT = 1000; // ms
int detectionCount = 0;
uint8_t flexSensor;

void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t buffer[32];
  uint8_t len = 0;
  
  // Check if manufacturer data is present
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  
  if (len > 0) {
    // Print what we received for debugging
    // Serial.print("Mfg data found, len=");
    // Serial.print(len);
    // Serial.print(" ID=0x");
    
    // Extract manufacturer ID (little endian)
    uint16_t mfgID = buffer[0] | (buffer[1] << 8);
    // Serial.print(mfgID, HEX);
    
    // Check if manufacturer ID matches and we have enough data
    if (mfgID == MANUFACTURER_ID && len >= 2 + sizeof(beaconPayload)) {
      // Check if payload matches, minus flex sensor data
      bool payloadMatches = true;
      for (int i = 0; i < (sizeof(beaconPayload) - 1); i++) {
        if (buffer[2 + i] != beaconPayload[i]) {
          payloadMatches = false;
          break;
        }
      }
      
      if (payloadMatches) {
        lastDetectionTime = millis();
        detectionCount++;
        Serial.println(">>> OUR BEACON DETECTED! <<<");
        
        // save flex sensor number data
        flexSensor = buffer[1 + sizeof(beaconPayload)];
        Serial.print("flex sensor data set: ");
        Serial.println(flexSensor);

        // Print payload
      Serial.print(" Payload: ");
      for (int i = 2; i < len && i < 10; i++) {
        // Serial.print("0x");
        // if (buffer[i] < 16) Serial.print("0");
        Serial.print(buffer[i]);
        Serial.print(" ");
      }
      Serial.println();

      } else {
        Serial.println("ID matches but payload doesn't");
      }
    }
  }
  
  // Continue scanning
  Bluefruit.Scanner.resume();
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give time to open serial monitor
  
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(12, OUTPUT);
  
  // Turn off LEDs initially
  digitalWrite(LED_BLUE, LOW);  // OFF
  digitalWrite(LED_RED, LOW);   // OFF

  digitalWrite(12, LOW);
  
  Serial.println("=== BLE Beacon Receiver ===");
  Serial.print("Looking for Mfg ID: 0x");
  Serial.println(MANUFACTURER_ID, HEX);
  Serial.print("Looking for Payload: 0x");
  Serial.print(beaconPayload[0], HEX);
  Serial.print(" 0x");
  Serial.println(beaconPayload[1], HEX);
  
  // Initialize Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  
  // Set up scanning
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);
  
  Serial.println("Scanning started...");
  Serial.println("-------------------");
  
  lastDetectionTime = 0;
}

void loop() {
  unsigned long currentTime = millis();
  static unsigned long lastPrintTime = 0;
  
  // Check if beacon was detected recently
  if (lastDetectionTime > 0 && (currentTime - lastDetectionTime < DETECTION_TIMEOUT)) {
    // Beacon detected - turn on blue LED
    // Serial.print("last detection time: \n");
    // Serial.println(lastDetectionTime);

    // trigger different outputs depending on which flex sensor was detected

    if (flexSensor & 0b1) {
      Serial.println("flex sensor 1 bent!");
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(12, HIGH);

    } 
    
    if (flexSensor > 1 & 0b1) {
      Serial.println("flex sensor 2 bent!");
      digitalWrite(LED_RED, HIGH);
    }

    if (flexSensor > 2 & 0b1) {
      Serial.println("flex sensor 2 bent!");
    }
    
    // Print status every 2 seconds
    // if (currentTime - lastPrintTime > 2000) {
    //   Serial.print("ACTIVE - Last seen: ");
    //   Serial.print(currentTime - lastDetectionTime);
    //   Serial.print(" ms ago, Total detections: ");
    //   Serial.println(detectionCount);
    //   lastPrintTime = currentTime;
    // }

  } else {
    // No beacon detected - turn LEDs and digital outputs off
    digitalWrite(LED_BLUE, LOW); // OFF
    digitalWrite(LED_RED, LOW);   // OFF
    digitalWrite(12, LOW);
    
    // Print status every 2 seconds
    if (currentTime - lastPrintTime > 2000) {
      Serial.println("INACTIVE - No beacon detected");
      Serial.println("flex data reset");
      flexSensor = 0;
      lastPrintTime = currentTime;
    }
  }
  
  delay(50);
}