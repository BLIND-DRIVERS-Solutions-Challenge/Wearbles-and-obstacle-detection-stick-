#include <Wire.h>
#include <NewPing.h>
#include <MPU6050_tockn.h>
#include <SoftwareSerial.h>

MPU6050 mpu6050(Wire);
NewPing sonar(2,3,300); // Trigger pin (2), Echo pin (3), Max distance (300 cm)
// setting boolean values for detection and prevention of repeated obstacle detection
bool obstacleDetected = false; 
bool tiltDetected = false;

SoftwareSerial bluetoothSerial(0, 1);  // RX, TX

const int rightVibrationMotorPin = 9;  // Right vibrator
const int leftVibrationMotorPin = 10;  // Left vibrator

void setup() {
  Serial.begin(9600); 
  bluetoothSerial.begin(9600);  // baud rate for Bluetooth communication
  Wire.begin();
  mpu6050.begin();

  digitalWrite(rightVibrationMotorPin, LOW);
  digitalWrite(leftVibrationMotorPin, LOW);   

  pinMode(rightVibrationMotorPin, OUTPUT);
  pinMode(leftVibrationMotorPin, OUTPUT);

}

void loop() {
  mpu6050.update();
  float distance = getDistance();
  float tiltAngle = getTiltAngle();

  float obstacleThreshold = 30.0;  // range of obstacle detection
  float tiltThreshold = 30.0; // min angle of tilt detection

  // for slope and hill detection 
  float inclination = mpu6050.getAngleX();
  float slopeThreshold = 20.0 // to be adjusted 

  // Obtain accelerometer and gyroscope data
  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();

  float gx = mpu6050.getGyroX();
  float gy = mpu6050.getGyroY();
  float gz = mpu6050.getGyroZ();

  // Calculate acceleration magnitude
  float accelerationMagnitude = sqrt(ax * ax + ay * ay + az * az);

  // Define fall detection criteria and thresholds
  float accelerationThreshold = 15.0; // to be adjusted
  float gyroscopeThreshold = 5.0;

  // check if we have signal from app ,else excute further
  if(!bluetoothSerial.available()){
    if (distance < obstacleThreshold) {
      // Obstacle detected
      activateBothVibrators(); // for vibrator 
      if (!obstacleDetected) {
        // If an obstacle was not detected in the previous loop, print "STOP"
        Serial.println("STOP1"); // testing code to be removed
        Serial.println(distance);
        sendBluetoothData("Obstacle_detected"); // app data
        obstacleDetected = true; // Set to true
      }
    }else if (tiltAngle > tiltThreshold) {
      if(!tiltDetected){
        Serial.println("STOP2");// testing code to be removed
        // Tilt detected, possible change in direction,
        activateBothVibrators(); // for vibrator
        sendBluetoothData("Change_in_direction_detected"); //app data
        tiltDetected = true; //set to true
      }
    }else if(obstacleDetected){
      // No obstacle detected
      deactivateBothVibrators(); // for vibrator 
      Serial.println("Walk"); // testing code to be removed
      //reset 
      obstacleDetected = false; 
      tiltDetected = false;
    }else if (accelerationMagnitude > accelerationThreshold && (abs(gx) > gyroscopeThreshold || abs(gy) > gyroscopeThreshold || abs(gz) > gyroscopeThreshold)) {
    // Fall detected, trigger alert or notification
    Serial.println("Fall detected!");
    sendBluetoothData("user_Fell")
    
    }else if (abs(inclination) > slopeInclinationThreshold) {
    // Possible slope or hill
    Serial.println("Slope or hill detected!");
    sendBluetoothData("Slope detected");
    }else{
      activateBothVibrators(); //works for buzzer 
      // deactivateBothVibrators(); //works for vibrator 
    }
  }else if(bluetoothSerial.available()>0){
    char command = bluetoothSerial.read(); // read app data
    adjustVibrationIntensity(command); // process it for vibrations
  }
}

float getDistance() {
  delay(50); 
  return sonar.ping_cm();
}
float getTiltAngle() {
  mpu6050.update();
  return mpu6050.getAngleY();
}

void activateRightVibrator() {
  digitalWrite(rightVibrationMotorPin, HIGH);
  digitalWrite(leftVibrationMotorPin, LOW);
}

void activateLeftVibrator() {
  digitalWrite(rightVibrationMotorPin, LOW);
  digitalWrite(leftVibrationMotorPin, HIGH);
}

void activateBothVibrators() {
  digitalWrite(rightVibrationMotorPin, HIGH);
  digitalWrite(leftVibrationMotorPin, HIGH);
}

void deactivateBothVibrators() {
  digitalWrite(rightVibrationMotorPin, LOW);
  digitalWrite(leftVibrationMotorPin, LOW);
}

void sendBluetoothData(String data){
  bluetoothSerial.println(data);
}

void adjustVibrationIntensity(char command) {
  const int pwmRange = 255;  // PWM range
  const int vibrationIncrement = 20;  // Increment for adjusting vibration intensity
  const int vibrationGapDuration = 2000;  // Vibration gap duration
  const int stopDuration = 7000;  // Stop duration

  static int rightVibrationIntensity = 0;
  static int leftVibrationIntensity = 0;

  switch (command) {
    // adjusting vibrations intensity as required by user
    case 'I':  // Increase vibration intensity
      rightVibrationIntensity = constrain(rightVibrationIntensity + vibrationIncrement, 0, pwmRange);
      leftVibrationIntensity = constrain(leftVibrationIntensity + vibrationIncrement, 0, pwmRange);
      break;

    case 'D':  // Decrease vibration intensity
      rightVibrationIntensity = constrain(rightVibrationIntensity - vibrationIncrement, 0, pwmRange);
      leftVibrationIntensity = constrain(leftVibrationIntensity - vibrationIncrement, 0, pwmRange);
      break;

    case 'H':  // Stop vibration
      deactivateBothVibrators();
      break;

    // Navigations signal
    case 'S': // stop 
      activateBothVibrators();
      delay(stopDuration);  // Stop duration
      break;

    case 'R':  // Turn right, Start right vibrator with a gap
      digitalWrite(leftVibrationMotorPin, LOW);  
      activateRightVibrator();
      delay(vibrationGapDuration);
      break;

    case 'L':  // turn left, Start left vibrator with a gap
      digitalWrite(rightVibrationMotorPin, LOW);
      activateLeftVibrator();
      delay(vibrationGapDuration);
      break;

    default:
      break;
  }

  // Apply PWM to control vibration intensity
  analogWrite(rightVibrationMotorPin, rightVibrationIntensity);
  analogWrite(leftVibrationMotorPin, leftVibrationIntensity);
}

