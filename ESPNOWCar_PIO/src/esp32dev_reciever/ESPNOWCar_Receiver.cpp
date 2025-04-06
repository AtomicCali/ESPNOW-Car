#include <esp_now.h>
#include <WiFi.h>

//motors (FWD = R ; RVS = L)
int RM_enable = 23;
int LM_enable = 22;
int RM_FWD_pwm = 32;
int RM_RVS_pwm = 33;
int LM_FWD_pwm = 27;
int LM_RVS_pwm = 14;

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 5 
bool throttleAndSteeringMode = false;

const int PWMFreq = 10000; /* 1 KHz */
const int PWMResolution = 8;
int RM_FWD_pwmChannel = 0;
int RM_RVS_pwmChannel = 1;
int LM_FWD_pwmChannel = 2;
int LM_RVS_pwmChannel = 3;
const bool ON = true;
const bool OFF = false;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

typedef struct PacketData {
  int xAxisValue;
  int yAxisValue;
  bool switchState;
} PacketData;

PacketData receiverData;


void allMotors(bool state, bool leftSide = true, bool rightSide = true, bool coast = false) {
  if (state == ON) { coast = false; }
  
  if (rightSide) {
    ledcWrite(RM_FWD_pwmChannel, 0);
    ledcWrite(RM_RVS_pwmChannel, 0);
    digitalWrite(RM_FWD_pwm, state);
    digitalWrite(RM_RVS_pwm, state);
    digitalWrite(RM_enable, HIGH);
    if (coast) { digitalWrite(RM_enable, LOW); }
  }
  if (leftSide) {
    ledcWrite(LM_FWD_pwmChannel, 0);
    ledcWrite(LM_RVS_pwmChannel, 0);
    digitalWrite(LM_FWD_pwm, state);
    digitalWrite(LM_RVS_pwm, state);
    digitalWrite(LM_enable, HIGH);
    if (coast) { digitalWrite(LM_enable, LOW); }
  }
}


void motorSet(bool state, int pin, int channel, int pwm = 0) {
  if (state) { //on
    digitalWrite(pin, HIGH);
    ledcWrite(channel, pwm); 
  } else {
    ledcWrite(channel, 0);
    digitalWrite(pin, LOW); 
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed, bool coast = false, bool enable = true) {
  if (enable) {
    allMotors(ON, true, true);
    //right
    if (rightMotorSpeed > 0) {
      motorSet(ON, RM_FWD_pwm, RM_FWD_pwmChannel, abs(rightMotorSpeed));  
      motorSet(OFF, RM_RVS_pwm, RM_RVS_pwmChannel);
    }
    if (rightMotorSpeed < 0) {
      motorSet(OFF, RM_FWD_pwm, RM_FWD_pwmChannel);  
      motorSet(ON, RM_RVS_pwm, RM_RVS_pwmChannel, abs(rightMotorSpeed));
    } 
    if (rightMotorSpeed == 0) {
      allMotors(OFF, false, true, coast);
    }
    //  left
    if (leftMotorSpeed > 0) {
      motorSet(ON, LM_FWD_pwm, LM_FWD_pwmChannel, abs(leftMotorSpeed));  
      motorSet(OFF, LM_RVS_pwm, LM_RVS_pwmChannel);
    }
    if (leftMotorSpeed < 0) {
      motorSet(OFF, LM_FWD_pwm, LM_FWD_pwmChannel);  
      motorSet(ON, LM_RVS_pwm, LM_RVS_pwmChannel, abs(leftMotorSpeed));
    } 
    if (leftMotorSpeed == 0) {
      allMotors(OFF, true, false, coast);
    }
  } 

  if (!enable) {
    allMotors(OFF);
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
  float rightMotorSpeed = 255;
  float leftMotorSpeed = 255;

  //orient with joystick wires going left
  float steering = map(receiverData.xAxisValue, 0, 254, -1000, 1000);  
  float throttle = map(receiverData.yAxisValue, 254, 0, -1000, 1000);
  steering /= 1000;
  throttle /= 1000;

  if (steering > 0) {
    rightMotorSpeed *= (1.0-abs(steering));
  }
  if (steering < 0) {
    leftMotorSpeed *= (1.0-abs(steering));
  }
  rightMotorSpeed *= throttle;
  leftMotorSpeed *= throttle;

  rotateMotor(floor(rightMotorSpeed), floor(leftMotorSpeed), true);
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
  pinMode(RM_enable, OUTPUT);
  pinMode(LM_enable, OUTPUT);
  pinMode(RM_FWD_pwm, OUTPUT);
  pinMode(RM_RVS_pwm, OUTPUT);
  pinMode(LM_FWD_pwm, OUTPUT);
  pinMode(LM_RVS_pwm, OUTPUT);

  //Set up PWM
  ledcSetup(RM_FWD_pwmChannel, PWMFreq, PWMResolution);
  ledcSetup(RM_RVS_pwmChannel, PWMFreq, PWMResolution);  
  ledcSetup(LM_FWD_pwmChannel, PWMFreq, PWMResolution);
  ledcSetup(LM_RVS_pwmChannel, PWMFreq, PWMResolution);  

  ledcAttachPin(RM_FWD_pwm, RM_FWD_pwmChannel);
  ledcAttachPin(RM_RVS_pwm, RM_RVS_pwmChannel); 
  ledcAttachPin(LM_FWD_pwm, LM_FWD_pwmChannel);
  ledcAttachPin(LM_RVS_pwm, LM_RVS_pwmChannel); 
  
  allMotors(OFF, true, true, true);
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
  delay(0.01);
  //Check Signal lost.
  unsigned long now = millis();

  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    rotateMotor(0, 0);
  }
}
