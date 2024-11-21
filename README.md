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
