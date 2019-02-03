import processing.serial.*;

Serial port;
String serial;
PFont f; 
String x,y;
float xPos, yPos;
void setup() {
  //background(222);
  size(1920,1080);
  f = createFont("Arial",16,true);
  port = new Serial(this, "COM11", 9600);
  port.clear();
  serial = port.readStringUntil('\n');
  serial = null;
}

void draw() {
  background(222);
  textFont(f,20);                
  fill(255, 0, 0); 
  text(xPos, 920, 23);
  text(yPos, 920, 43);
  while(port.available() > 0) {
      serial = port.readStringUntil('\n');
  }  
  if (serial != null) {
       String[]a = split(serial, ',');
       x = a[0];
       y = a[1]; 
       //println(x);
       println(a[0]);
       println(a[1]);
       xPos = float(x);
       yPos = float(y);
       cursor();
  }
}

void cursor() {
  fill(0,0,0);
  ellipse(xPos,yPos,100,100);
  
}
