// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_SSD1306.h>

/* ============== MAIN =====================*/

 //Use I2C with OLED RESET pin on D4
 #define OLED_RESET A4
 Adafruit_SSD1306 oled(OLED_RESET);


 unsigned long previousMillis;
 unsigned long interval = 30000;

 void setup() {
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done     
     
  //oled.display(); // show splashscreen

//  Time.zone(-4); 
}

void loop() {
   oled.clearDisplay();
  delay(200);
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.print("Hello");
  oled.setTextColor(BLACK, WHITE); // 'inverted' text
  
  oled.display();
  
  delay(800);
}
