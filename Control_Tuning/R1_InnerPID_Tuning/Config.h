#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// FEATURE FLAGS
// ==========================================
#define ENABLE_IMU 1       // Set to 1 to enable BNO08x sensor fusion, 0 for robot-centric
#define ENABLE_PID 1       // Set to 1 for M/T closed-loop control, 0 for open-loop
    // Set to 1 to enable Serial prints, 0 for competition

// ==========================================
// PIN DEFINITIONS
// ==========================================
const int light = 2;
const int arm = 23;

// Motor 1 (RMCS)
const int motor1_pin1 = 13; 
const int motor1_pin2 = 14; 
const int enc1_A = 39; 
const int enc1_B = 36;

// Motor 2 (BTS7960)
const int motor2_pin1 = 18; 
const int motor2_pin2 = 19; 
const int enc2_A = 33; 
const int enc2_B = 32;

// Motor 3 (BTS7960)
const int motor3_pin1 = 26; 
const int motor3_pin2 = 25; 
const int enc3_A = 35; 
const int enc3_B = 34;

// ==========================================
// ROBOT KINEMATICS & LIMITS
// ==========================================
const int JOYSTICK_LIMIT = 127;
const float HEADING_DEADZONE = 2.0f;
const int ROTATION_DIR = -1;
const int IMU_TIMEOUT_MS = 500;

const int TICKS_PER_REV = 1300;
const float MAX_RPM_REQUIRED = 500.0f;
const float JOY_TO_RPM = MAX_RPM_REQUIRED / 127.0f;
const float WHEEL_TO_CENTRE = 1.0f;
const float PRECISION_MULTIPLIER = 0.3f;

// ==========================================
// HEADING PID (Outer Loop)
// ==========================================
float HEADING_KP = 4.0f;
float HEADING_KI = 0.0f;
float HEADING_KD = 0.0f;

// ==========================================
// CONTROL LOOP TUNING (Inner Loops)
// ==========================================
int FF1 = 25;  // RMCS Static Friction
int FF2 = 25;  // BTS7960 Static Friction
int FF3 = 18;  // BTS7960 Static Friction

const float current_step = 20.0f;
const float FILTER_ALPHA = 0.3f;

const unsigned long MT_WINDOW_US = 20000;
const unsigned long TIMEOUT_US = 100000;
const unsigned long PS4_TIMEOUT = 300;

#endif