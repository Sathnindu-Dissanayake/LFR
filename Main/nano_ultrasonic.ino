// ==========================================
// ARDUINO NANO - ULTRASONIC CONTROLLER
// ==========================================
// This code reads two ultrasonic sensors and 
// safely transmits the distances to the ESP32.

#define TRIG_FRONT 2
#define ECHO_FRONT 3

#define TRIG_RIGHT 4
#define ECHO_RIGHT 5

void setup() {
  // Start Serial communication at 9600 baud to talk to ESP32
  Serial.begin(9600); 
  
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

// Function to calculate distance in centimeters
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond ping
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo (30ms timeout = max range of ~5 meters)
  long duration = pulseIn(echoPin, HIGH, 30000); 
  
  // If nothing is detected (timeout), return 999 so ESP32 knows path is clear
  if (duration == 0) return 999; 
  
  // Convert time to distance (cm)
  long distance = duration * 0.034 / 2;
  return distance;
}

void loop() {
  // 1. Read Front Sensor
  long distFront = readDistance(TRIG_FRONT, ECHO_FRONT);
  
  // Short delay so the sound waves from the front sensor 
  // don't confuse the right sensor!
  delay(15); 
  
  // 2. Read Right Sensor
  long distRight = readDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // 3. Send Data to ESP32
  // We format it simply as: F:15,R:20
  Serial.print("F:");
  Serial.print(distFront);
  Serial.print(",R:");
  Serial.println(distRight); // println adds the \n so ESP32 knows message is done
  
  // Send updates 20 times a second
  delay(50); 
}
