#define xPin A1 
#define yPin A0
#define buttonPin 12

int xPosition;
int yPosition;
int button;

void setup() {
Serial.begin(9600);
pinMode(xPin, INPUT);
pinMode(yPin, INPUT);
pinMode(buttonPin, INPUT);
}

void loop() {
buttonState();
readX();
readY();
Serial.print(xPosition);
Serial.print(",");
Serial.print(yPosition);
Serial.print(",");
Serial.println(button);
// example value 500,500,0; -- x,y,button -- data[0],data[1],data[2]
//delay(100); // to see serial value properly
}

void buttonState() {
button = digitalRead(buttonPin);
if (button == HIGH) {
digitalWrite(13, HIGH); // indicating button is pressed.
}else {
digitalWrite(13, LOW);
}
}
void readX() {
xPosition = analogRead(xPin);
xPosition = map(xPosition, 1023, 0 , 0, 1920); // prevent invert axis.
}

void readY() {
yPosition = analogRead(yPin);
yPosition = map(yPosition, 0, 1023, 0, 1080);
}
