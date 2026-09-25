// ==========================================
// ARDUINO NANO - MAIN SENSOR HUB
// ==========================================
// Reads two ultrasonic sensors + TCS3200 color sensor
// Displays color on RGB LED, and transmits all data to ESP32!
//
// WIRING:
// Ultrasonic Front: TRIG=D2, ECHO=D3
// Ultrasonic Right: TRIG=D4, ECHO=D5
// TCS3200 Color:    S0=D6, S1=D7, S2=D8, S3=D9, OUT=D10, OE=GND
// RGB LED:          RED=D11, GREEN=D12, BLUE=D13

// --- ULTRASONIC PINS ---
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5

// --- TCS3200 COLOR PINS ---
#define COLOR_S0  6
#define COLOR_S1  7
#define COLOR_S2  8
#define COLOR_S3  9
#define COLOR_OUT 10

// --- RGB LED PINS ---
#define LED_RED   11
#define LED_GREEN 12
#define LED_BLUE  13

int redValue = 0;
int greenValue = 0;
int blueValue = 0;

void setup() {
  Serial.begin(9600); // Talk to ESP32
  
  // Ultrasonics
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Color Sensor
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // RGB LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Frequency scaling 20%
  digitalWrite(COLOR_S0, LOW);
  digitalWrite(COLOR_S1, HIGH);
}

void setLED(int r, int g, int b) {
  // Assuming Common Cathode (Normal)
  digitalWrite(LED_RED,   r);
  digitalWrite(LED_GREEN, g);
  digitalWrite(LED_BLUE,  b);
}

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

int readColor(int s2State, int s3State) {
  digitalWrite(COLOR_S2, s2State);
  digitalWrite(COLOR_S3, s3State);
  delay(15); 
  
  int pulseWidth = pulseIn(COLOR_OUT, LOW, 80000); // 80ms timeout
  if (pulseWidth == 0) return 9999; // Return high number for timeout
  return pulseWidth;
}

void loop() {
  // 1. Read Ultrasonics
  long distFront = readDistance(TRIG_FRONT, ECHO_FRONT);
  delay(10); 
  long distRight = readDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // 2. Read Color
  redValue   = readColor(LOW, LOW);      
  greenValue = readColor(HIGH, HIGH);    
  blueValue  = readColor(LOW, HIGH);     
  
  String color = "UNKNOWN";
  
  // Apply the calibrated math!
  if (redValue == 9999 || greenValue == 9999 || blueValue == 9999) {
      color = "NO READING";
      setLED(0, 0, 0);
  }
  else if (redValue > 4000 && greenValue > 4000 && blueValue > 4000) {
      color = "BLACK";
      setLED(0, 0, 0);
  }
  else if (redValue < 1200 && greenValue < 1200 && blueValue < 1200) {
      color = "WHITE";
      setLED(1, 1, 1);
  }
  else if (redValue < 1400 && greenValue < 1800 && blueValue > 1800) {
      color = "YELLOW";
      setLED(1, 1, 0);
  }
  else if (redValue < greenValue && redValue < blueValue && redValue < 1500) {
      color = "RED";
      setLED(1, 0, 0);
  }
  else if (blueValue < redValue && blueValue < greenValue && blueValue < 1400) {
      color = "BLUE";
      setLED(0, 0, 1);
  }
  else if (greenValue < redValue && greenValue < blueValue && greenValue < 2000) {
      color = "GREEN";
      setLED(0, 1, 0);
  }
  else {
      color = "UNKNOWN";
      setLED(0, 0, 0);
  }
  
  // 3. Send everything to ESP32 over Serial!
  // Example: "F:15,R:20,C:RED"
  Serial.print("F:");
  Serial.print(distFront);
  Serial.print(",R:");
  Serial.print(distRight);
  Serial.print(",C:");
  Serial.println(color); 
  
  delay(30); 
}
