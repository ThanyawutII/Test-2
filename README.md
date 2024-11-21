```c++
#include <Mapf.h>
#include <PID_v2.h>
#include <Wire.h>
#include <Servo.h>

Servo myservo;
Servo myservo2;

const int E1Pin = 10;
const int M1Pin = 12;

/**inner definition**/
typedef struct {
  byte enPin;
  byte directionPin;
} MotorContrl;

const int M1 = 0;
const int MotorNum = 1;

const MotorContrl MotorPin[] = { { E1Pin, M1Pin } };

const int Forward = LOW;
const int Backward = HIGH;


//Button
int BUTTON = A8;

//PID
PID_v2 compassPID(0.7, 0.0001, 0.05, PID::Direct);

//Ultra
int const ULTRA_PIN = A9;
int const ULTRA_PIN_II = A10;

//INEX Gyro
float pvYaw;
uint8_t rxCnt = 0, rxBuf[8];

//  Light Sensors
int const RED_SEN = A6;
int const BLUE_SEN = A7;

// Servo
int const STEER_SRV = 27;  //16
int const ULTRA_SRV = 25;  //23

//Others
char TURN = 'U';
long halt_detect_line_timer;
int Line_Number = 0;
int plus_degree = 0;
int count;
```
This code sets up a robot with servos, motors, sensors (light, ultrasonic, and gyro), and a PID controller for compass-based movement. It configures motor pins, light sensor pins, and controls turn direction and line detection timing to guide the robot's navigation.


- #### **Section 2 [Open Challenge round]**

```c++
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  compassPID.Start(0, 0, 0);
  compassPID.SetOutputLimits(-180, 180);
  compassPID.SetSampleTime(10);
  pinMode(STEER_SRV, OUTPUT);
  pinMode(ULTRA_SRV, OUTPUT);
  pinMode(ULTRA_PIN, INPUT);
  pinMode(RED_SEN, INPUT);
  pinMode(BLUE_SEN, INPUT);
  pinMode(BUTTON, INPUT);
  initMotor();
  while (!Serial)
    ;
  myservo.attach(ULTRA_SRV, 500, 2400);
  myservo2.attach(STEER_SRV, 500, 2500);
  steering_servo(0);
  ultra_servo(0, 'L');
  // check_leds();
  while (analogRead(BUTTON) > 500)
    ;
  zeroYaw();
}
```
The setup function initializes serial communication, PID control, motor, servo pins, and sensors. It calibrates the compass with `zeroYaw()`, waits for a button press, and prepares the system for operation by attaching servos and starting the PID control.

- #### **Section 3 [Open Challenge round]**

```c++
void loop() {
  float baseDesiredDistance = 25;
  while (analogRead(BUTTON) > 500) {
    getIMU();
    motor(50);
    Color_detection();
    float desiredDistance = baseDesiredDistance;  // Start with the base value
    if (TURN == 'L') {
      desiredDistance = 16;
    } else if (TURN == 'R') {
      desiredDistance = 17.5;
    }
    float distanceError = getDistance() - desiredDistance;
    float deadband = 2.0;
    if (abs(distanceError) < deadband) {
      distanceError = 0.0;
    }
    float directionFactor = (TURN == 'R') ? -1.0 : 1.0;
    float adjustedYaw = pvYaw - (distanceError * directionFactor);
    float pidOutput = compassPID.Run(adjustedYaw);
    steering_servo(pidOutput);
    ultra_servo(pvYaw, TURN);
    // Serial.println(getDistance());

    if (count >= 12) {
      long timer01 = millis();
      while (millis() - timer01 < 800) {
        motor(100);
        getIMU();
        Color_detection();
        float desiredDistance = baseDesiredDistance;  // Start with the base value
        if (TURN == 'L') {
          desiredDistance = 12;
        } else if (TURN == 'R') {
          desiredDistance = 14.5;
        }
        float distanceError = getDistance() - desiredDistance;
        float deadband = 2.0;
        if (abs(distanceError) < deadband) {
          distanceError = 0.0;
        }
        float directionFactor = (TURN == 'L') ? -1.0 : 1.0;
        float adjustedYaw = pvYaw + (distanceError * directionFactor);
        float pidOutput = compassPID.Run(adjustedYaw);
        steering_servo(pidOutput);
        ultra_servo(pvYaw, TURN);
      }
      motor(0);
      while (true) {
      }
    }
  }
  motor(0);
  while (analogRead(BUTTON) <= 500) {
  }
  while (analogRead(BUTTON) > 500)
    ;
  while (analogRead(BUTTON) <= 500)
    ;
}
```
The loop function controls the robot's movement by reading sensor values, adjusting motor speed, and steering using PID feedback. It checks the button status to start/stop, adjusts the robot's direction based on distance errors, and uses servos to steer. When a specific count is reached, it stops.

<br><hr>

## Obstacle Challenge round

In this round, our robot must complete three laps on a track marked with randomly placed green and red traffic signs.
<br>
● Red Obstacle: Keep to the right side of the lane.
<br>
● Green Obstacle: Keep to the left side of the lane.
<br>
The last traffic sign in the second round indicates the next move: a green sign means continue in the same direction for the third round, while a red sign requires turning around to complete the round in the opposite direction. The robot must not move any traffic signs. After finishing the three laps, the robot must find a parking lot and perform parallel parking.

### **The strategy**

In the WRO 2024 Obstacle Challenge round, the robot uses a combination of ultrasonic sensors, a color sensor, a gyro, and an OpenMV Camera to navigate the course, detect and avoid obstacles, and maintain a safe distance from walls. The OpenMV Camera identifies obstacles and their colors, turning right for red obstacles and left for green ones, while the gyro ensures smooth and precise turns.

After completing the second round, the robot uses the OpenMV Camera to search for the purple parking area by detecting its color and comparing its size to the red and green pillars to determine its location. If the last pillar encountered before the end of the third round is red, the robot performs a U-turn; if it is green, the robot continues straight. Once the third round is complete, the robot proceeds to park in the identified purple parking area, accurately positioning itself based on the location recorded by the OpenMV Camera.

<br>

<p align="center">
If the robot sees red obstacle.
</p>

</p>
<p align="center">
  <img src="https://github.com/ThanyawutII/Test-2/blob/main/obr.jpg" width="700"/>
</p>

<br>

<p align="center">
If the robot sees green obstacle
</p>

</p>
<p align="center">
  <img src="https://github.com/ThanyawutII/Test-2/blob/main/obg.jpg" width="700"/>
</p>

The robot still use the PID to walk, but we added the avoidance degree to avoid the obstacles.

<br>

<p align="center">
After completing the second round
</p>

</p>
<p align="center">
  <img src="https://github.com/ThanyawutII/Test-2/blob/main/pur.jpg" width="700"/>
</p>

The robot uses the OpenMV Camera to search for the purple parking area, detecting its color directly and comparing its size to the red and green pillars to determine its position.

<br>

<p align="center">
If the color of the last pillar is green(continuing straight)
</p>

</p>
<p align="center">
  <img src="https://github.com/ThanyawutII/Test-2/blob/main/con.jpg" width="700"/>
</p>

<br>

<p align="center">
If the color of the last pillar is red(U- turn)
</p>

</p>
<p align="center">
  <img src="https://github.com/ThanyawutII/Test-2/blob/main/uturn.jpg" width="700"/>
</p>

<br>

<p align="center">
Park in parking area
</p>

</p>
<p align="center">
  <img src="https://github.com/ThanyawutII/Test-2/blob/main/park.jpg" width="700"/>
</p>

The robot will drive to park in the purple parking area that was detected, using the stored position to accurately align itself.

<br><hr>

### **Flowchart**

<image src="https://github.com/ThanyawutII/Test-2/blob/main/Obstecle%20round.jpg" height = "650">

<hr>

### **Source Code**

- #### **Section 1 [Obstacle Challenge round]**

```c++
#include <Mapf.h>
#include <PID_v2.h>
#include <Servo.h>
#include "CameraHandler.h"
```
We declare essential libraries for robot control: `Mapf.h` for Mapping the constrained distance from one range to another, `PID_v2.h` for smooth movement control, `Servo.h` for servo motor positioning, and `CameraHandler.h` for processing camera data. These libraries enable the robot to navigate, adjust movement, and interpret visual information effectively.

- #### **Section 2 [Obstacle Challenge round]**

```c++
CameraHandler camera;
BlobData blob;
BlobData purple_blob1;
BlobData purple_blob2;
```
We initializes a `CameraHandler` object called camera to manage the camera’s functions. It also creates three `BlobData` instances: `blob` for storing red and green pillar information, and `purple_blob1` and `purple_blob2` specifically for tracking two separate purple blobs. These variables enable the robot to detect, distinguish, and interact with multiple objects in its environment, particularly purple-colored ones.

- #### **Section 3 [Obstacle Challenge round]**

```c++
Servo myservo;
Servo myservo2;
```
We declare two Servo objects, `myservo` and `myservo2`, allowing control of two individual servo motors.

- #### **Section 4 [Obstacle Challenge round]**

```c++
const int E1Pin = 10;
const int M1Pin = 12;

typedef struct {
  byte enPin;
  byte directionPin;
} MotorContrl;

const int M1 = 0;
const int MotorNum = 1;

const MotorContrl MotorPin[] = { E1Pin, M1Pin };

const int Forward = LOW;
const int Backward = HIGH;
```
We sets up motor control using `E1Pin` and `M1Pin` for power and direction. The `MotorContrl` structure and `MotorPin` array organize these pins, while Forward and Backward constants control motor rotation, making direction easy to manage.

- #### **Section 5 [Obstacle Challenge round]**

```c++
int const RED_SEN = 6;
int const BLUE_SEN = 7;
int const BUTTON = 8;
int const ULTRA_PIN = 9;
int const ULTRA_PIN_II = 10;
int const STEER_SRV = 27;
int const ULTRA_SRV = 25;
```
We connect Red sensor to port 6, Blue sensor to port 7, Button to port 8, Ultrasonic that measure distance between robot and the wall to port 9, Ultrasonic in front of the robot to port 8+10, Servo for steering port 27, and the last one, Servo for turning ultrasonic port 25.

- #### **Section 6 [Obstacle Challenge round]**

```c++
float pvYaw;
uint8_t rxCnt = 0, rxBuf[8];
```
We defines `pvYaw` as a float to store the robot's yaw (orientation) angle. `rxCnt` is an 8-bit integer to count received data, and `rxBuf` is an 8-byte array to hold incoming data.

- #### **Section 7 [Obstacle Challenge round]**

```c++
long halt_detect_line_timer = 0;
long halt_detect_parking = 0;
long MV_timer = 0;

float found_parkAngle = 0;
float absYaw;
float uturnYaw;
float avoidance_degree;

int last_found_signature;
int plus_degree = 0;
int count_line = 0;
int parking_step = 0;
int parkingsection = -1;
int side = 1;
int hi = 0;
int angle = 115;

bool startpark = false;
bool parking = false;
bool next = false;
bool foundpark = false;
bool uturn = false;

char currentBlock = 'N';
char previousBlock = 'N';
char lastblock;
char lastfound = 'U';
char TURN = 'U';
char ULTRA_DIR = 'R';
```
This code above, we defines several variables used for various control and tracking functions. `long` is used for define time variable. `float` for variable that has decimal. `int` for variable that is integer. `bool` for variable that its output is true and false. `char` is for variable that is used to store data as a single character.

- #### **Section 8 [Obstacle Challenge round]**

```c++
void setup() {
  initialize_everything();
  while (analogRead(BUTTON) > 500)
    ;
  zeroYaw();
}
```
In `void setup` we initialize every part of our robot (function mentioned in another page) and then wait until the button is pressed. After that, reset the compass.

- #### **Section 9 [Obstacle Challenge round]**

```c++
void loop() {
  // Calculate camera errors
  camera.handleIncomingData();
  BlobData tempBlob = camera.getBlobData();
  if (tempBlob.signature == 1) {
    // RED
    last_found_signature = 1;
    blob = tempBlob;

  } else if (tempBlob.signature == 2) {
    // GREEN
    last_found_signature = 2;
    blob = tempBlob;

  } else if (tempBlob.signature == 3) {
    purple_blob1 = tempBlob;
  } else if (tempBlob.signature == 4) {
    purple_blob2 = tempBlob;
  }
```
This code processes data from the camera to identify and sort detected blobs by color. It first updates the camera data, then retrieves the current blob as `tempBlob`. The code checks the color of `tempBlob` based on its "signature": if it’s red (1), it sets `last_found_signature` to 1 and stores `tempBlob` as blob. If it’s green (2), it does the same but sets `last_found_signature` to 2. Purple blobs are handled separately, with `purple_blob1` storing blobs marked as 3 and `purple_blob2` storing blobs marked as 4.

- #### **Section 10 [Obstacle Challenge round]**

```c++
 float avoidance_degree = 0;
  if (tempBlob.signature == 3 && tempBlob.width / 3.9 > blob.width) {
    avoidance_degree = calculate_avoidance(tempBlob.signature, tempBlob.width, tempBlob.x, tempBlob.y) * -2;
  } else {
    avoidance_degree = calculate_avoidance(blob.signature, blob.width, blob.x, blob.y);
  }
```



- #### **Section 11 [Obstacle Challenge round]**
```c++
  int desiredDistance = parking_step == 0 ? (camera.isBlockFound() ? 20 : 40) : 15;
  float distanceError = getDistance() - desiredDistance;
  float frontDistance = getDistanceII();

  float deadband = 2.0;
  if (abs(distanceError) < deadband) {
    distanceError = 0.0;
  }
  float directionFactor = (ULTRA_DIR == 'R') ? -1.0 : 1.0;
  float adjustedYaw = pvYaw - clamp(distanceError * directionFactor, -20, 20);
  float pidOutput = compassPID.Run(adjustedYaw);
```


- #### **Section 12 [Obstacle Challenge round]**

```c++
  // TEST PARKING

  if (count_line >= 8 && count_line < 12 && tempBlob.signature == 3 && parkingsection == -1) {
    parkingsection = count_line % 4;
  } else if (count_line > 12) {
    if (parkingsection == 0) {
      parkingsection = 4;
    }
  }
```


- #### **Section 13 [Obstacle Challenge round]**

```c++
if (parking_step == 1) {
    ULTRA_DIR = TURN == 'R' ? 'L' : 'R';
  }
```


- #### **Section 14 [Obstacle Challenge round]**

```c++
int parking_degree = ((purple_blob1.x + purple_blob2.x) / 2 - 160) * -0.5;
  int final_degree = camera.isBlockFound() ? mapf(min(max(getDistance(), 15), desiredDistance), 15, desiredDistance, pidOutput, avoidance_degree) : pidOutput;
```


- #### **Section 15 [Obstacle Challenge round]**

```c++
 getIMU();
  color_detection();
```

- #### **Section 16 [Obstacle Challenge round]**

```c++
 switch (parking_step) {
    case 1:

      if (!startpark) {
        halt_detect_parking = millis();
        startpark = true;
      }
      if (tempBlob.signature != 3 && purple_blob1.width < 70) {
        steering_servo(pidOutput);
        ultra_servo(pvYaw, ULTRA_DIR);
        motor(40);
      } else {
        parking_step = 2;
        startpark = false;
      }
      break;
    case 2:
      halt_detect_line_timer = millis();
      if (!startpark) {
        halt_detect_parking = millis();
        startpark = true;
      }
      if (millis() - halt_detect_parking < 1400) {
        steering_servo(final_degree);
        ultra_servo(0, ULTRA_DIR);
        motor(40);
      } else {
        parking_step = 3;
        startpark = false;
        if (TURN == 'L') {
          plus_degree += 90;
        } else {
          plus_degree -= 90;
        }
      }

      break;

    case 3:

      if (!startpark) {
        halt_detect_parking = millis();
        startpark = true;
      }
      if (millis() - halt_detect_parking < 2000) {
        steering_servo(pvYaw);
        ultra_servo(0, ULTRA_DIR);
        motor(-40);
      } else {
        parking_step = 4;
        startpark = false;
      }

      break;

    case 4:
      if (frontDistance > 10) {
        steering_servo(mapf(clamp(frontDistance, 10, 20), 20, 10, parking_degree, 0));
        motor(mapf(clamp(frontDistance, 10, 20), 20, 10, 35, 30));
      } else {
        motor(30);
        delay(500);
        motor(0);
        while (true)
          ;
      }
      break;

    case 5:
      if (count_line >= 13 && count_line < 12 + parkingsection) {
        motor_and_steer(pidOutput);
        ultra_servo(pvYaw, TURN);
      } else {
        parking_step = 1;
      }
      break;

    default:
      motor_and_steer(final_degree);
      ultra_servo(pvYaw, ULTRA_DIR);
      if (count_line > 12 && parkingsection != -1) {
        parking_step = 5;
      }
      break;
  }
```

- #### **Section 17 [Obstacle Challenge round]**

```c++
uTurn();
```


<hr><br>

### Function


### `Initialize Everything`

```c++
void initialize_everything() {
  Serial.begin(19200);
  Serial1.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);

  compassPID.Start(0, 0, 0);
  compassPID.SetOutputLimits(-180, 180);
  compassPID.SetSampleTime(10);

  pinMode(STEER_SRV, OUTPUT);
  pinMode(ULTRA_SRV, OUTPUT);
  pinMode(ULTRA_PIN, INPUT);
  pinMode(RED_SEN, INPUT);
  pinMode(BLUE_SEN, INPUT);
  pinMode(BUTTON, INPUT);

  initMotor();
  while (!Serial)
    ;
  myservo.attach(ULTRA_SRV, 500, 2400);
  myservo2.attach(STEER_SRV, 500, 2500);
  steering_servo(0);
  ultra_servo(0, 'U');
}
```


### `Degrees to radians`

```c++
float degreesToRadians(double degrees) {
  return degrees * PI / 180.0;
}

float radiansToDegree(double raidans) {
  return raidans / PI * 180.0;
}
```


### `Avoidance Calculation(based on size and position)`

```c++
float _cal_avoidance(char mode, int targetWidth, int objectWidth, int blockCenterX, int blockCenterY) {
  float focalLength = 2.8;
  float cameraFOV = 70;

  float distance = (targetWidth * focalLength * 100) / objectWidth;

  float deltaX = blockCenterX - (320 / 2);
  float deltaY = blockCenterY - (240 / 2);

  float detected_degree = -deltaX * cameraFOV / 320.0;

  float blockPositionX = distance * sin(degreesToRadians(detected_degree));
  float blockPositionY = distance * cos(degreesToRadians(detected_degree)) - 10;

  ULTRA_DIR = mode;

  if (mode == 'L') {
    return max(radiansToDegree(atan2(blockPositionX + (targetWidth / 2 + 10), blockPositionY)), 5) * 1.1;
  } else if (mode == 'R') {
    return min(radiansToDegree(atan2(blockPositionX - (targetWidth / 2 + 10), blockPositionY)), -5) * 0.9;
  } else {
    return 0;
  }
}
```


### `Avoidance Calculation(based on signature)`

```c++
float calculate_avoidance(int signature, int objectWidth, int blockCenterX, int blockCenterY) {
  int avoidance_degree = 0;

  if (signature == 2) {
    avoidance_degree = _cal_avoidance('L', 5, objectWidth, blockCenterX, blockCenterY);
  } else if (signature == 1) {
    avoidance_degree = _cal_avoidance('R', 5, objectWidth, blockCenterX, blockCenterY);
  } else if (signature == 3 || signature == 4) {
    avoidance_degree = _cal_avoidance(TURN, 20, objectWidth, blockCenterX, blockCenterY);
  }
  return avoidance_degree;
}
```


### `Wrap value`

```c++
int wrapValue(int value, int minValue, int maxValue) {
  int range = maxValue - minValue + 1;
  if (value < minValue) {
    value += range * ((minValue - value) / range + 1);
  }
  return minValue + (value - minValue) % range;
}
```

### `Initiate motor`

```c++
void initMotor() {
  int i;
  for (i = 0; i < MotorNum; i++) {
    digitalWrite(MotorPin[i].enPin, LOW);

    pinMode(MotorPin[i].enPin, OUTPUT);
    pinMode(MotorPin[i].directionPin, OUTPUT);
  }
}
```


### `Set motor direction`

```c++
void setMotorDirection(int motorNumber, int direction) {
  digitalWrite(MotorPin[motorNumber].directionPin, direction);
}
```


### `Set motor speed`

```c++
inline void setMotorSpeed(int motorNumber, int speed) {
  analogWrite(MotorPin[motorNumber].enPin, 255.0 * (speed / 100.0));
}
```


### `Motor`

```c++
void motor(int speed) {
  if (speed > 0) {
    setMotorDirection(M1, Forward);
    setMotorSpeed(M1, speed);
  } else {
    setMotorDirection(M1, Backward);
    setMotorSpeed(M1, speed);
  }
}
```

### `Color detection`

```c++
void color_detection() {
  int blue_value = analogRead(BLUE_SEN);
  if (TURN == 'U') {
    int red_value = analogRead(RED_SEN);
    if (blue_value < 600 || red_value < 600) {
      int lowest_red_sen = red_value;
      long timer_line = millis();
      while (millis() - timer_line < 100) {
        int red_value = analogRead(RED_SEN);
        if (red_value < lowest_red_sen) {
          lowest_red_sen = red_value;
        }
      }
      if (lowest_red_sen > 600) {
        TURN = 'L';
        plus_degree += 90;
      } else {
        TURN = 'R';
        plus_degree -= 90;
      }
      halt_detect_line_timer = millis();
      count_line++;
    }
  } else {
    if (millis() - halt_detect_line_timer > 1800) {
      if (blue_value < 600) {
        if (TURN == 'R') {
          plus_degree -= 90;
        } else {
          plus_degree += 90;
        }
        halt_detect_line_timer = millis();
        count_line++;
      }
    }
  }
}
```


```c++
void steering_servo(int degree) {
  myservo2.write((90 + max(min(degree, 50), -50)) / 2);
}
```


### `Ultrasonic Servo`

```c++
void ultra_servo(int degree, char mode_steer) {
  int middle_degree = 0;
  if (mode_steer == 'F') {
    middle_degree = 150;
  } else if (mode_steer == 'R') {
    middle_degree = 225;
  } else if (mode_steer == 'L' || mode_steer == 'U') {
    middle_degree = 80;
  } else {
  }
  myservo.write(mapf(max(min(middle_degree + degree, 225), 45), 0, 270, 0, 180));
}
```

### `Get distance`

```c++
float getDistance() {
  float raw_distance = mapf(analogRead(ULTRA_PIN), 0, 1023, 0, 500);
  if (TURN == 'L') {
    raw_distance += 0;
  } else if (TURN == 'R') {
    raw_distance -= 0;
  }
  return min(raw_distance, 50);
}

float getDistanceII() {
  float raw_distance = mapf(analogRead(ULTRA_PIN_II), 0, 1023, 0, 500);
  if (TURN == 'L') {
    raw_distance += 0;
  } else if (TURN == 'R') {
    raw_distance -= 0;
  }
  return min(raw_distance, 50);
}
```


### `Get IMU`

```c++
bool getIMU() {
  while (Serial1.available()) {
    rxBuf[rxCnt] = Serial1.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA) return;
    rxCnt++;
    if (rxCnt == 8) {
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) {
        pvYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        pvYaw = wrapValue(pvYaw + plus_degree, -179, 180);
        return true;
      }
    }
  }
  return false;
}
```


### `Zero Yaw`

```c++
void zeroYaw() {
  Serial1.begin(115200);
  delay(100);
  Serial1.write(0XA5);
  delay(10);
  Serial1.write(0X54);
  delay(100);
  Serial1.write(0XA5);
  delay(10);
  Serial1.write(0X55);
  delay(100);
  Serial1.write(0XA5);
  delay(10);
  Serial1.write(0X52);
  delay(100);
}
```

### `Motor and Steering`

```c++
void motor_and_steer(int degree) {
  degree = clamp(degree, -52, 52);
  steering_servo(degree);
  motor((map(abs(degree), 0, 30, 49, 49)));
}
```

### `Clamp`

```c++
float clamp(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}
```


### `Check LEDs`

```c++
void check_leds() {
  while (true) {
    Serial.print("Geen: ");
    Serial.print(analogRead(BLUE_SEN));
    Serial.print("   Red: ");
    Serial.println(analogRead(RED_SEN));
  }
}
```

### `U Turn`

```c++
void uTurn() {
  camera.handleIncomingData();
  BlobData tempBlob = camera.getBlobData();
  if ((count_line == 3 || (count_line == 4 && millis() - halt_detect_line_timer < 900))) {
    if (tempBlob.signature == 1) {
      currentBlock = 'R';
    } else if (tempBlob.signature == 2) {
      currentBlock = 'G';
    } else {
      currentBlock = 'N';
    }

    if (currentBlock != 'N' && currentBlock != lastblock) {
      lastfound = previousBlock;
      lastblock = currentBlock;
      previousBlock = lastblock;
    }
  }

  if (lastfound == 'G' && count_line == 8 && uturn == false) {
    side = -1;
    angle = 90;
  } else if (lastfound == 'R' && count_line == 8 && uturn == false) {
    side = 1;
  }
  absYaw = abs(pvYaw);
  while (lastblock == 'R' && count_line == 8 && (millis() - halt_detect_line_timer > 1000)) {
    getIMU();
    uturn = true;
    uturnYaw = pvYaw;  // Remove abs()
    float diff = angleDiff(uturnYaw, absYaw);
    while (diff >= -angle && diff <= angle) {
      getIMU();
      uturnYaw = pvYaw;  // Remove abs()
      diff = angleDiff(uturnYaw, absYaw);
      steering_servo(45 * side);
      motor(50);
      Serial.println(diff);
    }

    count_line++;
    if (TURN == 'R') {
      TURN = 'L';
      plus_degree += 90;
    } else if (TURN == 'L') {
      TURN = 'R';
      plus_degree -= 90;
    }
    break;
  }
}
```

### `Angle Difference`

```c++
float angleDiff(float a, float b) {
  float diff = a - b;
  while (diff > 180) diff -= 360;
  while (diff <= -180) diff += 360;
  return diff;
}
```

<hr><br>

### OpenMV

- #### **Section 1 [OpenMV]**

```c++
import time
import sensor
import display
from pyb import UART
```

- #### **Section 2 [OpenMV]**

```c++
# Initialize sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240 resolution

sensor.skip_frames(time=2000)  # Wait for settings to take effect.

# Lock auto-exposure and auto-white-balance to prevent drift across reboots.
sensor.set_auto_gain(False)  # Disable auto gain.
sensor.set_auto_whitebal(False)  # Disable auto white balance.

# Lock exposure to prevent fluctuations in lighting conditions.
sensor.set_auto_exposure(False, exposure_us=7000)  # Adjust exposure manually.


sensor.set_contrast(3)
sensor.set_brightness(0)
sensor.set_saturation(0)

sensor.skip_frames(time=1000)

#sensor.__write_reg(0x0E, 0b00000000)  # Disable night mode
#sensor.__write_reg(0x3E, 0b00000000)  # Disable BLC
```


- #### **Section 3 [OpenMV]**

```c++
# Color thresholds
GREEN_THRESHOLDS = [(22, 48, -52, -21, 17, 49)]
RED_THRESHOLDS = [(0, 52, 13, 37, -3, 25)]
PURPLE_THRESHOLDS = [(33, 70, -25, 14, 42, 76)]
```


- #### **Section 4 [OpenMV]**

```c++
# Region of Interest (ROI)
ROI = (0, 160, 320, 240)

# Initialize UART
uart = UART(3, 19200, timeout_char = 2000)
clock = time.clock()
```


- #### **Section 5 [OpenMV]**

```c++
def send_blob_data(blob, blob_type, color):
    img.draw_rectangle(blob.rect(), color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color)
    data = f"{blob.cx()},{blob.cy()},{blob.w()},{blob.h()},{blob_type}\n"
    uart.write(data)
    print(data)
```


- #### **Section 6 [OpenMV]**

```c++
def send_no_blob_data():
    data = "0,0,0,0,0\n"
    uart.write(data)
    print(data)
```


- #### **Section 7 [OpenMV]**

```c++
while True:
    clock.tick()
    img = sensor.snapshot()

    # Detect red and green blobs
    green_blobs = img.find_blobs(GREEN_THRESHOLDS, roi=ROI, area_threshold=30, pixels_threshold=30, merge=True)
    red_blobs = img.find_blobs(RED_THRESHOLDS, roi=ROI, area_threshold=30, pixels_threshold=30, merge=True)

    # Find the largest blob between red and green blobs
    largest_green = max(green_blobs, key=lambda b: b.area(), default=None)
    largest_red = max(red_blobs, key=lambda b: b.area(), default=None)

    # Determine the largest blob between red and green
    largest_blob = None
    if largest_green and largest_red:
        largest_blob = largest_green if largest_green.area() > largest_red.area() else largest_red
    elif largest_green:
        largest_blob = largest_green
    elif largest_red:
        largest_blob = largest_red

    # Send data for the largest red or green blob (if found), else send '0'
    if largest_blob:
        blob_type = 2 if largest_blob in green_blobs else 1  # Green = 2, Red = 1
        color = (0, 255, 0) if blob_type == 2 else (200, 0, 0)
        send_blob_data(largest_blob, blob_type, color)
    else:
        send_no_blob_data()  # Send 0 when no red or green blobs are found

    # Detect and send data for all purple blobs
    purple_blobs = img.find_blobs(PURPLE_THRESHOLDS, roi=ROI, area_threshold=30, pixels_threshold=30, merge=True)
    purple_blobs = sorted(purple_blobs, key=lambda b: b.cx())

    if purple_blobs:
        for i, purple_blob in enumerate(purple_blobs):
            color = (255, 0, 255) if i == 0 else (252, 244, 3)  # Alternate colors for blobs
            send_blob_data(purple_blob, 3 + i, color)  # Different blob types for each purple blob
    cropped_img = img.crop(roi=ROI)  # Crop to exclude the top 120 pixels
    # Optional: Read and print incoming UART data
    if uart.any():
        data = uart.readline()
        print("Received:", data)

```
