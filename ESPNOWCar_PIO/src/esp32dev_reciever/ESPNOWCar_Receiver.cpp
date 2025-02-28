#include <esp_now.h>
#include <WiFi.h>

//motors
int allMotorsEnable = 33;
int RM_FWD_pwm = 27;
int RM_RVS_pwm = 14;
int LM_FWD_pwm = 12;
int LM_RVS_pwm = 13;

#define MAX_MOTOR_SPEED 255
bool throttleAndSteeringMode = false;

const int PWMFreq = 10000; /* 1 KHz */
const int PWMResolution = 8;
int RM_FWD_pwmChannel = 0;
int RM_RVS_pwmChannel = 1;
int LM_FWD_pwmChannel = 2;
int LM_RVS_pwmChannel = 3;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

typedef struct PacketData {
  int xAxisValue;
  int yAxisValue;
  bool switchState;
} PacketData;

PacketData receiverData;



void rotateMotor(int rightMotorSpeed, int leftMotorSpeed, bool enable = true) {
  if (enable) {
    digitalWrite(allMotorsEnable, HIGH);
    if (rightMotorSpeed > 0) {
      ledcWrite(RM_FWD_pwm, abs(rightMotorSpeed));   
    }
    if (rightMotorSpeed < 0) {
      ledcWrite(RM_RVS_pwm, abs(rightMotorSpeed));   
    }

    if (leftMotorSpeed > 0) {
      ledcWrite(LM_FWD_pwm, abs(leftMotorSpeed));   
    }
    if (leftMotorSpeed < 0) {
      ledcWrite(LM_RVS_pwm, abs(leftMotorSpeed));   
    } 
  } else {
    digitalWrite(allMotorsEnable, LOW);
    ledcWrite(RM_FWD_pwm, 0);
    ledcWrite(RM_FWD_pwm, 0);
  }
}

void simpleMovements() {
  if (receiverData.yAxisValue <= 75) {                      //Move car Forward
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  } 
  else if (receiverData.yAxisValue >= 175) {                //Move car Backward
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValue >= 175) {                //Move car Right
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValue <= 75) {                 //Move car Left
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }else {                                                   //Stop the car
    rotateMotor(0, 0);  
  }   
}

void throttleAndSteeringMovements() {
  int throttle = map( receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map( receiverData.xAxisValue, 0, 254, -255, 255);  
  int motorDirection = 1;
  
  if (throttle < 0)       //Move car backward
  {
    motorDirection = -1;    
  }

  int rightMotorSpeed, leftMotorSpeed;
  rightMotorSpeed =  abs(throttle) - steering;
  leftMotorSpeed =  abs(throttle) + steering;
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == 0) {return;} //exit if no data was sent
  //grab data
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  //print recieved data
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.switchState;
  Serial.println(inputData); 

  throttleAndSteeringMode = receiverData.switchState;

  if (throttleAndSteeringMode) {
    throttleAndSteeringMovements();
  } else {
    simpleMovements();
  }
  
  lastRecvTime = millis();  
}


void setUpPinModes() {
  pinMode(allMotorsEnable, OUTPUT);
  pinMode(RM_FWD_pwm, OUTPUT);
  pinMode(RM_RVS_pwm, OUTPUT);
  pinMode(LM_FWD_pwm, OUTPUT);
  pinMode(LM_RVS_pwm, OUTPUT);

  //Set up PWM
  ledcSetup(RM_FWD_pwmChannel, PWMFreq, PWMResolution);
  ledcSetup(RM_RVS_pwmChannel, PWMFreq, PWMResolution);  
  ledcSetup(LM_FWD_pwmChannel, PWMFreq, PWMResolution);
  ledcSetup(LM_FWD_pwmChannel, PWMFreq, PWMResolution);  

  ledcAttachPin(RM_FWD_pwm, RM_FWD_pwmChannel);
  ledcAttachPin(RM_RVS_pwm, RM_RVS_pwmChannel); 
  ledcAttachPin(LM_FWD_pwm, LM_FWD_pwmChannel);
  ledcAttachPin(LM_RVS_pwm, LM_RVS_pwmChannel); 
  
  rotateMotor(0, 0);
}



void setup() {
  setUpPinModes();
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  //Check Signal lost.
  unsigned long now = millis();

  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    rotateMotor(0, 0);
  }
}
