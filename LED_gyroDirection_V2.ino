// Once you import this library into an app on the web based IDE, modify the code to look something like the following.
// This code is a heavily modified version of the MPU6050_raw.ino example found elsewhere in this repo.
// This code has been tested against the MPU-9150 breakout board from Sparkfun.

// This #include statement was automatically added by the Particle IDE.
#include "MPU6050.h"
#include <math.h> 

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN_LED 6
#define NUM_LEDS 16
#define BRIGHTNESS 25

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// MPU variables:
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accelMag;

float rot[3];

int LED_Direction; 
int accel_avg = 0; 


void setup() {

    Wire.begin();
    Serial.begin(9600);

    // The following line will wait until you connect to the Spark.io using serial and hit enter. This gives
    // you enough time to start capturing the data when you are ready instead of just spewing data to the UART.
    //
    // So, open a serial connection using something like:
    // screen /dev/tty.usbmodem1411 9600
//    while(!Serial.available()) SPARK_WLAN_Loop();
    
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // Cerify the connection:
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


    // set up LED pins
    pixels.setBrightness(BRIGHTNESS);
    pixels.begin();
    clearPixels();
    startUpPattern();
}

void loop() {
    // read raw accel/gyro measurements from device
    delay(10);
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax = ax/16384;
    ay = ay/16384;
    az = az/16384;

    gx = gx/131;
    gy = gy/131;
    gz = gz/131;
    
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    accelMag =  pow(pow(ax,2) + pow(ay,2) + pow(az,2),0.5);
    LED_Direction = atan2(gy,gx) * 180 / M_PI; 

    // get average acceleration
    accel_avg = (accel_avg * 50 + (accelMag/6000 * 100)*10) / 60; // running average of 50 last values to slow down sweep;
       

    rot[0] = gz;
    rot[1] = gy;
    rot[2] = gx;

    // set LED ring - Roll pitch from gyro for direction, accelerometer magnitude for magnitude of sweep
    accelDirection(getLED_Direction(rot),accel_avg);
    
}

int getLED_Direction(float rot[]){ 
  float yaw = rot[0];
  float pitch = rot[1];
  float roll = rot[2]; //recalibrate to have all values have 40 degrees from -4 to 4
  
  int default_pitch = 0;
  int default_roll = 0;

  double LED_angle = atan2((pitch),(roll)) * 180/M_PI;
  
  
//  Serial.print("LED_angle: ");
//  Serial.println(LED_angle);
//
//  Serial.print("Pitch: ");
//  Serial.println(pitch);
//  Serial.print("Roll: ");
//  Serial.println(roll);
  
  
  // if statements to determine LED direction
  if (abs(roll) < 10 && abs(pitch) < 10){ // balanced 
    return 16;
  } 
  else if (LED_angle > 168.75 || LED_angle <= -168.75){ return 4; }
  else if (LED_angle > -168.75 && LED_angle <= -146.25){ return 5; }
  else if (LED_angle > -146.25 && LED_angle <= -123.75){ return 6; }
  else if (LED_angle > -123.75 && LED_angle <= -101.25){ return 7; }
  else if (LED_angle > -101.25 && LED_angle <= -78.25){ return 8; }
  else if (LED_angle > -78.75 && LED_angle <= -56.25){ return 9; }
  else if (LED_angle > -56.25 && LED_angle <= -33.75){ return 10; }
  else if (LED_angle > -33.75 && LED_angle <= -11.25){ return 11; }
  else if (LED_angle > -11.25 && LED_angle <= 11.25){ return 12; }
  else if (LED_angle > 11.25 && LED_angle <= 33.75){ return 13; }
  else if (LED_angle > 33.75 && LED_angle <= 56.25){ return 14; }
  else if (LED_angle > 56.25 && LED_angle <= 78.75){ return 15; }
  else if (LED_angle > 78.75 && LED_angle <= 101.25){ return 0; }
  else if (LED_angle > 101.25 && LED_angle <= 123.75){ return 1; }
  else if (LED_angle > 123.25 && LED_angle <= 146.25){ return 2;}
  else if (LED_angle > 146.25 && LED_angle <= 168.75){ return 3; }

  else{
    return 16;
  }  
}

void accelDirection(uint8_t LED_dir, uint8_t LED_mag) {
  float maxMag = 100;
  int redVal;
  int blueVal;
  int greenVal;
  int num_pixels; 
  // Error checking 
  if (LED_dir > 16 || LED_dir < 0){
    LED_dir = 0;
  }

  if (LED_mag > 100 || LED_mag < 0) {
    LED_mag = 100;
  }
  clearPixels(); // clear previous pixels;

  if (LED_dir == 16) { // balanced/level gyro - make all pixels change relative to speed
    redVal = 255 - LED_mag / maxMag * 255;
    greenVal = 0;
    blueVal = LED_mag/maxMag * 255;
    Serial.println(redVal);
    for (uint8_t i=0; i<NUM_LEDS; i++) {
      pixels.setPixelColor(i,redVal,greenVal,blueVal);
    }
    pixels.show();
  }
  else { // not balanced gyro - sweep one dot
    // set 1, 3, 5, 7, 9, 11, 13, 15 pixels depending on magnitude of speed
    num_pixels = LED_mag / 13 + 1;
    
    pixels.setPixelColor(LED_dir,255,0,0); // set first directional pixel
    for (uint8_t i=1; i<num_pixels;i++) {
      redVal = 255/num_pixels *(num_pixels - i);
      greenVal = 0; 
      blueVal = 255/num_pixels * i;
  
      // output pixels left and right of primary pixel. Amount of pixels depends on magnitude
      if (LED_dir-i < 0){
        pixels.setPixelColor(16 + (LED_dir-i),redVal,greenVal,blueVal); 
      }
      else{
        pixels.setPixelColor(((LED_dir-i)%16),redVal,greenVal,blueVal);
      }
      pixels.setPixelColor(((LED_dir+i)%16),redVal,greenVal,blueVal);
    }
    pixels.show();
  }
}

void clearPixels(){
    for(uint16_t i=0; i<NUM_LEDS; i++) {
        pixels.setPixelColor(i, pixels.Color(0,0,0));
    } 
    pixels.show();
}

void startUpPattern() {
    
    int redVal;
    int greenVal;
    int blueVal; 
    delay(100);
    for(uint16_t k=0; k<2; k++){
      for(uint16_t i=0; i<NUM_LEDS; i++) {
        for(uint16_t j=0; j<NUM_LEDS; j++) {
          redVal = 255 / 16 * (16-(j-i+1)%16);
          greenVal = 0;
          blueVal = 255 - 255 / 16 * (16-(j-i+1)%16);
          pixels.setPixelColor(j, pixels.Color(redVal,greenVal,blueVal));
        } 
        pixels.show();
        delay(50);
      }
    }
}
