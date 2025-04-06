#include <esp_now.h>
#include <ESP32Time.h>
#include <WiFi.h>

#define X_AXIS_PIN 32
#define Y_AXIS_PIN 33
#define SWITCH_PIN 36
bool switchPressed = false;
const int buttonDebounce = 20; //millis
// const int buttonPressDebounce = 20; //millis
// const int buttonUnpressDebounce = 20; //millis
ESP32Time rtc;

const int deadbandTolerance = 150;
const int rawValueCenter = 1500; //ideal = 2047
const int rawValueMax = 3200; //ideal = 4095
const int ValueCenter = 127;
const int ValueMax = 254;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t receiverMacAddress[] = {0x5C,0x01,0x3B,0x4B,0xAF,0xB4};  //5C:01:3B:4B:AF:B4

typedef struct PacketData {
  int xAxisValue;
  int yAxisValue;
  bool switchState;
} PacketData;

PacketData data;

esp_now_peer_info_t peerInfo;

//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
int mapAndAdjustJoystickDeadBandValues(int value) {
  if (value > rawValueCenter + deadbandTolerance) {
    value = map(value, (rawValueCenter + deadbandTolerance), rawValueMax, ValueCenter, ValueMax);
  } else if (value < rawValueCenter - deadbandTolerance) {
    value = map(value, (rawValueCenter - deadbandTolerance), 0, ValueCenter, 0);  
  } else {
    value = 127;
  }

  if (value > ValueMax) { value = ValueMax; }
  return value;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup()  {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  // esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  data.switchState = true;
  pinMode(X_AXIS_PIN, INPUT);
  pinMode(X_AXIS_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLDOWN);   
  delay(500);
}
 
void loop() {
  delay(0.001);
  data.xAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(X_AXIS_PIN));
  data.yAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(Y_AXIS_PIN));  
  Serial.print(data.xAxisValue); Serial.print(" - "); Serial.print(data.yAxisValue); Serial.print(" - "); Serial.println(data.switchState);

  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // } else {
  //   Serial.println("Error sending the data");
  // }    

    if (digitalRead(SWITCH_PIN) == switchPressed) {              
      if (rtc.getMillis() >= (buttonDebounce)) {
        switchPressed = !switchPressed;
        if (!switchPressed) { data.switchState = !data.switchState; }
      }
    } else {
      rtc.setTime();
    }

}
