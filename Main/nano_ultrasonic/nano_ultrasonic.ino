// ==========================================
// ARDUINO NANO - SENSOR CONTROLLER
// ==========================================
// Reads two ultrasonic sensors + TCS3200 color sensor
// and transmits all data to the ESP32.
//
// WIRING:
// Ultrasonic Front: TRIG=D2, ECHO=D3
// Ultrasonic Right: TRIG=D4, ECHO=D5
// TCS3200 Color:    S0=D6, S1=D7, S2=D8, S3=D9, OUT=D10, OE=GND

// --- ULTRASONIC PINS ---
#define TRIG_FRONT 2
#define ECHO_FRONT 3

#define TRIG_RIGHT 4
#define ECHO_RIGHT 5

// --- TCS3200 COLOR SENSOR PINS ---
#define COLOR_S0  6
#define COLOR_S1  7
#define COLOR_S2  8
#define COLOR_S3  9
#define COLOR_OUT 10

int redValue = 0;
int greenValue = 0;
int blueValue = 0;

void setup() {
  Serial.begin(9600); 
  
  // Ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Color sensor pins
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // Set frequency scaling to 20% (best for Arduino Nano)
  digitalWrite(COLOR_S0, LOW);
  digitalWrite(COLOR_S1, HIGH);

  Serial.println("NANO SENSOR HUB ONLINE");
}

// --- ULTRASONIC ---
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); 
  if (duration == 0) return 999; 
  
  long distance = duration * 0.034 / 2;
  return distance;
}

// --- COLOR SENSOR ---
int readColor(int s2State, int s3State) {
  digitalWrite(COLOR_S2, s2State);
  digitalWrite(COLOR_S3, s3State);
  delay(20); // Let filter settle
  
  int pulseWidth = pulseIn(COLOR_OUT, LOW, 100000);
  if (pulseWidth == 0) return 999;
  return pulseWidth;
}

String detectColor(int r, int g, int b) {
  if (r == 999 || g == 999 || b == 999) return "NONE";
  if (r > 200 && g > 200 && b > 200) return "BLACK";
  if (r < 80 && g < 80 && b < 80) return "WHITE";
  if (r < g && r < b && (g - r) > 20 && (b - r) > 20) return "RED";
  if (g < r && g < b && (r - g) > 20 && (b - g) > 20) return "GREEN";
  if (b < r && b < g && (r - b) > 20 && (g - b) > 20) return "BLUE";
  return "UNKNOWN";
}

void loop() {
  // 1. Read Ultrasonic Sensors
  long distFront = readDistance(TRIG_FRONT, ECHO_FRONT);
  delay(15); 
  long distRight = readDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // 2. Read Color Sensor
  redValue   = readColor(LOW, LOW);      // Red filter
  greenValue = readColor(HIGH, HIGH);    // Green filter
  blueValue  = readColor(LOW, HIGH);     // Blue filter
  String color = detectColor(redValue, greenValue, blueValue);
  
  // 3. Send ALL data to ESP32 in one line
  // Format: F:15,R:20,C:RED,RGB:45:120:110
  Serial.print("F:");
  Serial.print(distFront);
  Serial.print(",R:");
  Serial.print(distRight);
  Serial.print(",C:");
  Serial.print(color);
  Serial.print(",RGB:");
  Serial.print(redValue);
  Serial.print(":");
  Serial.print(greenValue);
  Serial.print(":");
  Serial.println(blueValue);
  
  delay(50); 
}
