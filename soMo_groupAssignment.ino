const int ledPin1 = 7;    // Pin 7 has PWM function
const int ledPin2 = 8;    // Pin 8 has PWM function
const int flexPin1 = A0;  // Pin A0 to read analog input
const int flexPin2 = A1;  // Pin A0 to read analog input

// Variables:
int value1;
int value2;  // Save analog value

void setup() {
  pinMode(ledPin1, OUTPUT); 
  pinMode(ledPin2, OUTPUT);
  Serial.begin(9600);       // Begin serial communication
}

void loop() {
  value1 = analogRead(flexPin1);
  value2 = analogRead(flexPin2);// Read and save analog value from potentiometer
  Serial.println(value1);
  Serial.println(value2);          // Print value
  value1 = map(value1, 700, 900, 0, 255);
  value2 = map(value2, 700, 900, 0, 255);
  analogWrite(ledPin1, value1); 
  analogWrite(ledPin2, value2); 
  delay(100);                    // Small delay
}
