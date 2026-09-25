// Define motor pins
#define ENA 4  
#define ENB 23 
#define IN1 18 
#define IN2 19 
#define IN3 21 
#define IN4 22 

// Encoder pins 
#define ENCODER_A_LEFT 34
#define ENCODER_B_LEFT 35
#define ENCODER_A_RIGHT 36
#define ENCODER_B_RIGHT 39

// IR sensor pins (Main 8 Array)
#define NUM_SENSORS 8
const int sensor_pins[NUM_SENSORS] = {32, 33, 25, 26, 27, 14, 15, 13}; 

// --- NEW FRONT 3 SENSORS ---
// Because they are digital, we can safely use these pins!
#define FRONT_LEFT 17
#define FRONT_CENTER 5
#define FRONT_RIGHT 2

// PID Constants
float Kp = 12.0; 
float Ki = 0.0;
float Kd = 80.0;
float Kp_encoder = 2.0;

float lastError = 0;
float integral = 0;
int threshold = 2500; 
float error = 0;

// --- SENSOR HUB DATA (From Nano) ---
long front_distance = 999;
long right_distance = 999;
String detected_color = "UNKNOWN";
String nano_data = "";

volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;
volatile bool left_motor_forward = true;
volatile bool right_motor_forward = true;

void IRAM_ATTR leftEncoderISR() {
    if (left_motor_forward) left_encoder_count++;
    else left_encoder_count--;
}

void IRAM_ATTR rightEncoderISR() {
    if (right_motor_forward) right_encoder_count++;
    else right_encoder_count--;
}

const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
#else
    ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
    ledcAttachPin(ENA, pwmChannelLeft);
    ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);
    ledcAttachPin(ENB, pwmChannelRight);
#endif

    pinMode(ENCODER_A_LEFT, INPUT);
    pinMode(ENCODER_B_LEFT, INPUT);
    pinMode(ENCODER_A_RIGHT, INPUT);
    pinMode(ENCODER_B_RIGHT, INPUT);

    // Front digital sensors require pinMode
    pinMode(FRONT_LEFT, INPUT);
    pinMode(FRONT_CENTER, INPUT);
    pinMode(FRONT_RIGHT, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), rightEncoderISR, RISING);

    Serial.begin(9600);
    delay(500); // Give the serial port time to open
    Serial.println("=================================");
    Serial.println("ESP32 ROBOT BRAIN ONLINE!");
    Serial.println("Waiting to hear from Nano...");
    Serial.println("=================================");

    // Start Serial2 to listen to the Arduino Nano! 
    // RX2 is GPIO 16. We set TX to GPIO 12 (unused) to guarantee it doesn't hijack Pin 17!
    Serial2.begin(9600, SERIAL_8N1, 16, 12); 
}

void writeMotorPWM(int leftPWM, int rightPWM) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);
#else
    ledcWrite(pwmChannelLeft, leftPWM);
    ledcWrite(pwmChannelRight, rightPWM);
#endif
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    left_motor_forward = (leftSpeed >= 0);
    right_motor_forward = (rightSpeed >= 0);
    
    writeMotorPWM(abs(leftSpeed), abs(rightSpeed));
    
    digitalWrite(IN1, leftSpeed >= 0);
    digitalWrite(IN2, leftSpeed < 0);
    digitalWrite(IN3, rightSpeed >= 0);
    digitalWrite(IN4, rightSpeed < 0);
}

void resetEncoders() {
    left_encoder_count = 0;
    right_encoder_count = 0;
}

unsigned long lastPIDTime = 0;
const int loopInterval = 10; 

// Memory variable to remember which way the line went before we lost it or hit an intersection
static int last_front_seen = 0; // -1 for left, 1 for right, 0 for center

void loop() {
    // 0. ALWAYS Read Ultrasonic Data from Nano (Non-blocking, runs every loop cycle!)
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n') {
            int f_index = nano_data.indexOf("F:");
            int r_index = nano_data.indexOf(",R:");
            int c_index = nano_data.indexOf(",C:");
            
            if (f_index != -1 && r_index != -1) {
                front_distance = nano_data.substring(f_index + 2, r_index).toInt();
                
                if (c_index != -1) {
                    right_distance = nano_data.substring(r_index + 3, c_index).toInt();
                    detected_color = nano_data.substring(c_index + 3); // Read to end of string
                    detected_color.trim(); // Remove any invisible \r characters
                } else {
                    right_distance = nano_data.substring(r_index + 3).toInt();
                }
                
                // Debug print so we can see the ESP32 getting the color data!
                // (You can comment this out later if it spams too much)
                Serial.print("ESP32 heard -> Front: "); Serial.print(front_distance);
                Serial.print(" | Right: "); Serial.print(right_distance);
                Serial.print(" | Color: "); Serial.println(detected_color);
            }
            nano_data = "";
        } else {
            nano_data += c;
            if (nano_data.length() > 30) nano_data = "";
        }
    }

    // PID timing gate
    if (millis() - lastPIDTime < loopInterval) {
        return; 
    }
    lastPIDTime = millis();
    
    // --- OBSTACLE AVOIDANCE ---
    // FRONT: Object directly in path -> Full stop!
    if (front_distance > 0 && front_distance < 15) {
        setMotorSpeed(0, 0);
        return; // Don't move until path is clear
    }
    
    // RIGHT: Wall or object too close on the right side -> Steer left to avoid!
    // This acts as both wall-following awareness AND side obstacle detection.
    bool right_obstacle = (right_distance > 0 && right_distance < 12);
    
    // 1. Read Front Sensors (Digital)
    bool f_left = (digitalRead(FRONT_LEFT) == HIGH);
    bool f_center = (digitalRead(FRONT_CENTER) == HIGH);
    bool f_right = (digitalRead(FRONT_RIGHT) == HIGH);

    // Update memory of the path's direction
    if (f_left && !f_right) last_front_seen = -1;
    else if (f_right && !f_left) last_front_seen = 1;
    else if (f_center) last_front_seen = 0;
    
    // 2. Read Main Array
    int sensor_values[NUM_SENSORS];
    float position = 0.0;
    int weight_sum = 0;
    int total_active_sensors = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_values[i] = analogRead(sensor_pins[i]);
        if (sensor_values[i] > threshold) {
            position += (float)i * sensor_values[i];
            weight_sum += sensor_values[i];
            total_active_sensors++;
        }
    }

    // --- DYNAMIC PID & PROACTIVE STEERING ---
    int baseSpeed = 160; 
    float proactiveCorrection = 0;
    float current_Kp = Kp;
    float current_Kd = Kd;

    // If there is a wall/obstacle on the right, gently push the robot left
    if (right_obstacle) {
        baseSpeed = 100; // Slow down near walls
        proactiveCorrection = -30; // Steer away from the wall (left)
    }

    if (f_center && !f_left && !f_right) {
        // PERFECTLY STRAIGHT PATH
        baseSpeed = 220; // Turbo Speed!
        current_Kp = Kp * 0.7; // Relax steering to prevent wobbles at high speed
        current_Kd = Kd * 1.2; // Increase damping
    } else if (f_left && !f_right) {
        // CURVE LEFT APPROACHING
        baseSpeed = 100; // Brake hard
        current_Kp = Kp * 1.6; // Aggressive steering to grip the curve
        proactiveCorrection = -50; // Feed-forward pre-steer
    } else if (f_right && !f_left) {
        // CURVE RIGHT APPROACHING
        baseSpeed = 100; // Brake hard
        current_Kp = Kp * 1.6; // Aggressive steering
        proactiveCorrection = 50; // Feed-forward pre-steer
    } else if (!f_left && !f_center && !f_right) {
        // FRONT LOST LINE (L-Junctions, Sharp Curves, or Severe Misalignment)
        // The front is completely blind! We must rely 100% on the main array to find the next path.
        baseSpeed = 70; // Brake very hard
        current_Kp = Kp * 2.8; // EXTREME multiplier! This will force one wheel in reverse to aggressively yank the robot back into alignment.
        current_Kd = Kd * 2.0; // Heavy damping to snap perfectly onto the line without over-swinging.
    }

    // --- INTERSECTIONS & 90-DEGREE CORNERS ---
    if (total_active_sensors >= 5) {
        // Center the wheels on the intersection
        setMotorSpeed(110, 110); 
        unsigned long t1 = millis();
        while (millis() - t1 < 2000) { // Safety timeout: 2 seconds max
            if (digitalRead(FRONT_LEFT) == LOW && 
                digitalRead(FRONT_CENTER) == LOW && 
                digitalRead(FRONT_RIGHT) == LOW) {
                break; 
            }
            delay(2);
        }
        
        // Decide turn based on front sensor memory
        unsigned long t2 = millis();
        if (last_front_seen == -1) {
            // Turn Left
            setMotorSpeed(-140, 140); 
            delay(150); // Initial kick
            while (analogRead(sensor_pins[3]) < threshold && analogRead(sensor_pins[4]) < threshold && millis() - t2 < 2000) {
                setMotorSpeed(-120, 120);
                delay(2);
            }
        } else if (last_front_seen == 1) {
            // Turn Right
            setMotorSpeed(140, -140); 
            delay(150);
            while (analogRead(sensor_pins[3]) < threshold && analogRead(sensor_pins[4]) < threshold && millis() - t2 < 2000) {
                setMotorSpeed(120, -120);
                delay(2);
            }
        } else {
            // T-Junction default Left Hand Rule
            setMotorSpeed(-140, 140); 
            delay(150);
            while (analogRead(sensor_pins[3]) < threshold && analogRead(sensor_pins[4]) < threshold && millis() - t2 < 2000) {
                setMotorSpeed(-120, 120);
                delay(2);
            }
        }
        integral = 0; 
        return; 
    }

    // --- LINE LOST LOGIC (Gaps vs Sawtooth Zig-Zags) ---
    if (total_active_sensors == 0) {
        
        // DASHED LINE (GAP) DETECTION
        // If error was tiny and we were straight, bridge the gap using ENCODERS (Time Independent!)
        if (abs(lastError) <= 1.5 && last_front_seen == 0) {
            int start_left = left_encoder_count;
            int start_right = right_encoder_count;
            unsigned long t_gap = millis();
            
            // Bridge gap for up to ~2500 encoder ticks OR 3 seconds max
            while (abs(left_encoder_count - start_left) < 2500 && millis() - t_gap < 3000) {
                int current_left = left_encoder_count - start_left;
                int current_right = right_encoder_count - start_right;
                int encoderDiff = current_left - current_right;
                
                int correction = encoderDiff * Kp_encoder;
                int bridgeSpeed = 130;
                setMotorSpeed(bridgeSpeed - correction, bridgeSpeed + correction);
                
                // If ANY sensor (Front or Main) hooks the line, exit gap mode immediately!
                bool lineFound = false;
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (analogRead(sensor_pins[i]) > threshold) { lineFound = true; break; }
                }
                if (digitalRead(FRONT_CENTER) == HIGH || digitalRead(FRONT_LEFT) == HIGH || digitalRead(FRONT_RIGHT) == HIGH) {
                    lineFound = true;
                }
                
                if (lineFound) {
                    integral = 0; return; 
                }
                delay(2); // Yield
            }
            
            // If we drove 2500 ticks and found nothing, it's a Dead End. 180 Spin!
            setMotorSpeed(140, -140); 
            delay(300); 
            unsigned long t3 = millis();
            while (analogRead(sensor_pins[3]) < threshold && analogRead(sensor_pins[4]) < threshold && millis() - t3 < 2000) {
                setMotorSpeed(120, -120); delay(2);
            }
            integral = 0; return;
        } 
        else {
            // SAWTOOTH / ACUTE CORNER DETECTION
            // We lost the line while turning. Pivot tightly to grab it!
            unsigned long t4 = millis();
            if (lastError < 0 || last_front_seen == -1) {
                // Hook Left Sawtooth
                while (analogRead(sensor_pins[3]) < threshold && analogRead(sensor_pins[4]) < threshold && millis() - t4 < 2000) {
                    setMotorSpeed(-160, 160); // Aggressive pivot speed
                    delay(2);
                }
            } else {
                // Hook Right Sawtooth
                while (analogRead(sensor_pins[3]) < threshold && analogRead(sensor_pins[4]) < threshold && millis() - t4 < 2000) {
                    setMotorSpeed(160, -160); 
                    delay(2);
                }
            }
            integral = 0; return;
        }
    }
    
    // Normal PID Line Following
    position /= weight_sum;
    
    // Reset encoders constantly while tracking the line so they are ready for the next gap
    resetEncoders(); 

    error = position - 3.5;
    integral += error;
    float derivative = error - lastError;
    
    // Apply Dynamic PID constants!
    float correction = (current_Kp * error) + (Ki * integral) + (current_Kd * derivative) + proactiveCorrection;
    lastError = error;

    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    setMotorSpeed(leftSpeed, rightSpeed);
}
