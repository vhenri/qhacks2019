#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <math.h> 
#include <Adafruit_SSD1306.h>

 //Use I2C with OLED RESET pin on 4
 #define OLED_RESET A4
 Adafruit_SSD1306 oled(OLED_RESET);

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN_LED 6
#define NUM_LEDS 16
#define BRIGHTNESS 25
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// class default I2C address is 0x68. Specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float Magnitude;
float accMag;
int LED_Direction; 
int accel_avg = 0; 

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                       OLED VARS                          ===
// ================================================================

 unsigned long previousMillis;
 unsigned long interval = 30000;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize serial communication
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #endif

    // initialize device
    mpu.initialize();

    // verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus(); // enable Arduino interrupt detection
        dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
        
    } else {  // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // set up LED pins
    pixels.setBrightness(BRIGHTNESS);
    pixels.begin();
    clearPixels();
    startUpPattern();
}

 // ---- IDK RANDOM OLED STUFF ----//
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done         
  //oled.display(); // show splashscreen
  //Time.zone(-4);
// ----------------------------//

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
//    Serial.println("Running");
    delay(10);

    // ------- OLED STUFF IDK? ------ //
    oled.clearDisplay();
    delay(200);
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0,0);
    oled.print(oled.print("Hello World"));
    oled.setTextColor(BLACK, WHITE); // 'inverted' text
    
    oled.display();
    
    delay(800);
    // ---------------------------- //
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO(); // reset so we can continue cleanly
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        float acc[3]; //acceleration vector [x, y, z]
        float rot[3]; //rotation vector [yaw, pitch, roll]
        Magnitude = getacc(acc);
        getrot(rot);

//        Serial.print("ypr\t");
//        Serial.print(rot[0]);
//        Serial.print("\t");
//        Serial.print(rot[1]);
//        Serial.print("\t");
//        Serial.println(rot[2]);
//
//        Serial.print("aworld\t");
//        Serial.print(acc[0]);
//        Serial.print("\t");
//        Serial.print(acc[1]);
//        Serial.print("\t");
//        Serial.println(acc[2]);

        // get average acceleration
        accel_avg = (accel_avg * 50 + (Magnitude/6000 * 100)*10) / 60; // running average of 50 last values to slow down sweep;
       
        
        // set LED ring - Roll pitch from gyro for direction, accelerometer magnitude for magnitude of sweep
        accelDirection(getLED_Direction(rot),accel_avg);
        Serial.println(getFreeMemory());
    }
}

float getacc(float accel[])
{
  // wait for correct available data length, should be a VERY short wait
  // read a packet from FIFO
  // track FIFO count here in case there is > 1 packet available
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  
  // get initial world-frame acceleration, adjusted to remove gravity and rotated based on known orientation from quaternion
  // Adjusted for the world frame of reference (yaw is relative to initial orientation).
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  
  accel[0] = aaWorld.x;
  accel[1] = aaWorld.y;
  accel[2] = aaWorld.z;

  accMag = pow(pow(accel[0],2) + pow(accel[1],2) + pow(accel[2],2), 0.5);
  return accMag;
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
void getrot(float rotation[])
{
  // see getacc
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  
  // get Euler angles in degrees
  // Note that yaw/pitch/roll angles suffer from gimbal lock
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  rotation[0] = (ypr[0] * 180/M_PI);
  rotation[1] = (ypr[1] * 180/M_PI);
  rotation[2] = (ypr[2] * 180/M_PI);
}

void gyroDirection(uint8_t LED_dir, uint8_t LED_mag) {
  // LED_dir from 0-15 to indicate direction,.
  // LED_mag indicates magnitude of tilt - reflect in color
  float maxMag = 100;
  int redVal;
  int blueVal;
  int greenVal;
  
  // Error checking 
  if (LED_dir > 16 || LED_dir < 0){
    LED_dir = 0;
  }

  if (LED_mag > 100 || LED_mag < 0) {
    LED_mag = 100;
  }
 
  if (LED_dir == 16) {  // Balanced / level Gyro
    clearPixels();
    for (uint8_t i=0; i < NUM_LEDS; i++){
      pixels.setPixelColor(i,255,0,0);
    }
    pixels.show();
  }
  else { // imbalanced Gyro
    // calculate values
    redVal = LED_mag/maxMag * 255;
    greenVal = 0;
    blueVal = 255 - (LED_mag/maxMag * 255);
  
    // clear and output pixel colors
    clearPixels();
    pixels.setPixelColor(LED_dir,redVal,greenVal,blueVal);
    pixels.show();
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

// Find out how much free memory we have
unsigned int getFreeMemory()
{
 uint8_t* temp = (uint8_t*)malloc(16);    // assumes there are no free holes so this is allocated at the end
 unsigned int rslt = (uint8_t*)SP - temp;
 free(temp);
 return rslt;
}
