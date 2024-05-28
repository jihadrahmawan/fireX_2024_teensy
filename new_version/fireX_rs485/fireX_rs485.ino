#include <Arduino.h>
#include <Wire.h>
#include "TeensyThreads.h"
#include <math.h>
#include "BNO055_support.h"  //Contains the bridge code between the API and Arduino
#include <Wire.h>
#include "SerialParser.h"



struct bno055_t myBNO;
struct bno055_euler myEulerData;  //Structure to hold the Euler data

unsigned long lastTime = 0;

const int PENENDANG = 15;



#include <WS2812Serial.h>

const int numled = 64;
const int pin = 17;

byte drawingMemory[numled * 3];          //  3 bytes per LED
DMAMEM byte displayMemory[numled * 12];  // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);

#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define YELLOW 0xFFFF00
#define PINK 0xFF1088
#define ORANGE 0xE05800
#define WHITE 0xFFFFFF


// Serial communication parameters
#define BAUDRATE 115200
#define SERIAL_CONFIG SERIAL_8N1

// Modbus slave address
#define ADDRESS 1

// Register addresses
#define CONTROL_MODE_REGISTER 0x200D
#define CONTROL_WORD_REGISTER 0x200E
#define ACCELERATION_TIME_LEFT_REGISTER 0x2080
#define ACCELERATION_TIME_RIGHT_REGISTER 0x2081
#define DECELERATION_TIME_LEFT_REGISTER 0x2082
#define DECELERATION_TIME_RIGHT_REGISTER 0x2083
#define TARGET_VELOCITY_LEFT_REGISTER 0x2088
#define TARGET_VELOCITY_RIGHT_REGISTER 0x2089

// Control words
#define CONTROL_MODE_VELOCITY 3
#define CONTROL_WORD_ENABLE 0x08
#define CONTROL_WORD_STOP 0x07
const int PWM_PENENDANG_1 = 2;
const int PWM_PENENDANG_2 = 3;
// Modbus frame
uint8_t modbusFrame[8];

volatile int speedx, speedy, speedz;
int speed_motor_1 = 0;
int speed_motor_2 = 0;
int speed_motor_3 = 0;
int motor_1 = 0;
int motor_2 = 0;
int motor_3 = 0;
const int set_max_speed = 150;

#define ENCODER_S1_A 9
#define ENCODER_S1_B 10
#define ENCODER_S2_A 11
#define ENCODER_S2_B 12

// Variables to store encoder counts
volatile long encoderS1Count = 0;
volatile long encoderS2Count = 0;

volatile float bno_x = 0;
volatile float bno_y = 0;
volatile float bno_z = 0;

volatile int led_color_set = 0;
// Variables to store position
float x_pos = 0.0;
float y_pos = 0.0;

float global_var_last_error_slot3 = 0;
float integral3;

void encoderS1A_ISR() {
  if (digitalRead(ENCODER_S1_A) == digitalRead(ENCODER_S1_B)) {
    encoderS1Count++;
  } else {
    encoderS1Count--;
  }
}

void encoderS2A_ISR() {
  if (digitalRead(ENCODER_S2_A) == digitalRead(ENCODER_S2_B)) {
    encoderS2Count++;
  } else {
    encoderS2Count--;
  }
}

volatile bool odometry_selesai = false;
volatile bool vision_selesai = false;
volatile bool odometry_aktif = false;
volatile bool set_heading_aktif = false;
volatile bool penggiring_aktif = false;
volatile bool yaw_kontrol_odometry_on = false;

volatile int targetX = 0;
volatile int targetY = 0;

volatile int targetHeading = 0;

volatile int kamera_x = 0;
volatile int kamera_y = 0;

String komando;

String mapSerial[9] = {
  "komando",
  "strategi",
  "x",
  "y",
  "state",
  "dummy1",
  "dummy2",
  "kamera",
  "ally"
};
SerialParser serialParser;

int step = 0;

void setup() {

  // delay(4000);
  pinMode(PENENDANG, OUTPUT);
  Serial.begin(9600);
  pinMode(PWM_PENENDANG_1, OUTPUT);
  pinMode(PWM_PENENDANG_2, OUTPUT);
  Wire.begin();
  //Initialization of the BNO055
  BNO_Init(&myBNO);  //Assigning the structure to hold information about the device
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1000);
  threads.addThread(imu_read);
  delay(2000);
  Serial2.begin(BAUDRATE, SERIAL_CONFIG);
  initializeMotor();
  turnOnMotor_1();
  delay(1000);
  threads.addThread(odometry_system);
  leds.begin();
  delay(1000);
  threads.addThread(led_action);
  delay(1000);
  // Setup encoder pins
  pinMode(ENCODER_S1_A, INPUT);
  pinMode(ENCODER_S1_B, INPUT);
  pinMode(ENCODER_S2_A, INPUT);
  pinMode(ENCODER_S2_B, INPUT);
  pinMode(PENENDANG, OUTPUT);

  //Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_S1_A), encoderS1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_S2_A), encoderS2A_ISR, CHANGE);
  pinMode(13, OUTPUT);
  delay(1000);
  threads.addThread(get_robot_pos_from_encoder);
  //threads.addThread(run_odometry);
  //threads.addThread(run_heading);
  //threads.addThread(send_toNuc);
  threads.addThread(receive_fromNuc);
}



void loop() {

  // if (komando=='1'){



  // }

 // menunggu_oper_vision();

  if (step == 0) {

    // contoh jika step 0 menggunakan odometry
    // 1 isi dulu koordinat robot
    targetX = -35;
    targetY = 110;
    // perlukan kontrol yaw untuk bergerak?
    // jika iya maka:
    yaw_kontrol_odometry_on = true;
    // jika perlu penggiring maka :
    penggiring_aktif = false;
    moveToPosition(targetX, targetY);
    //disini robot sudah sampai tujuan
    set_speed(0, 0, 0);  // Stop the robot
    delay(5000);
    //menendang (30);
    step = 1;
  }


  // //===============================step 1

  // if (step == 1) {
  //   // contoh step 1 set heading
  //   targetHeading = -75;
  //   penggiring_aktif = true;   // penggiring aktif
  //   set_heading_aktif = true;  //nyalakan robot untuk set heading
  //   run_heading(targetHeading);
  //   delay(1000);  // delay sejenak sebelum menendang lagi
  //   menendang(20);
  //   //reset_variable();
  //   step = 2;
  // }

  // if (step == 4) {
  //   int kecepatan_kejar = 100;
  //   penggiring_aktif = true;
  //   mengejar_bola_vision(kecepatan_kejar);
  //   //jika proxymity detec maka slsai
  //   //menendang(20);
  //   delay(1000);
  //   step = 5;
  // }

  // if (step == 10) {
  //   penggiring_aktif = true;
  //   menunggu_oper_vision();
  //   //jika proxymity detec maka slsai
  // }

  // if (step == 2) {
  //   set_speed(0, 0, 0);  // Stop the robot
  //   delay(1000);
  //   led_color_set = RED;
  // }





  digitalWrite(PENENDANG, 1);
  // Loop back to set motor speed again, or implement other logic as needed
}




void reset_variable() {
  odometry_selesai = false;
  vision_selesai = false;
  odometry_aktif = false;
  set_heading_aktif = false;
  penggiring_aktif = false;
  yaw_kontrol_odometry_on = false;
}
void menendang(int jeda) {

  digitalWrite(PENENDANG, 0);
  delay(jeda);
  digitalWrite(PENENDANG, 1);
  //delay(1000);
}

void led_action() {
  while (1) {
    int microsec = 1500000 / leds.numPixels();
    colorWipe(led_color_set, microsec);
    //colorWipe(PINK, microsec);
    threads.yield();
  }
}

void imu_read() {

  while (1) {
    bno055_read_euler_hrp(&myEulerData);     //Update Euler data into the structure
    bno_z = (float(myEulerData.h) / 16.00);  //Convert to degrees
    bno_x = (float(myEulerData.r) / 16.00);  //Convert to degrees
    bno_y = (float(myEulerData.p) / 16.00);  //Convert to degrees
    threads.delay(50);
  }
}

void get_robot_pos_from_encoder() {
  while (1) {
    noInterrupts();
    long newS1Count = encoderS1Count;
    long newS2Count = encoderS2Count;
    interrupts();

    // Constants for the angles (in degrees)
    const float angleS1 = 45.0;
    const float angleS2 = 135.0;

    // Conversion factor from ticks to distance (assuming a certain wheel radius and ticks per revolution)
    const float ticksPerRevolution = 1280;  // Example value, adjust according to your encoder specification
    const float wheelDiameter = 6;
    const float wheelCircumference = PI * wheelDiameter;  // roda diameter
    const float distancePerTick = wheelCircumference / ticksPerRevolution;

    float distanceS1 = newS1Count * distancePerTick;
    float distanceS2 = newS2Count * distancePerTick;

    // Calculate the (x, y) position
    x_pos = (distanceS1 * cos(radians(angleS1)) + distanceS2 * cos(radians(angleS2))) / 2;
    y_pos = (distanceS1 * sin(radians(angleS1)) + distanceS2 * sin(radians(angleS2))) / 2;
    threads.yield();
  }
}

void set_speed(float x, float y, float z) {
  speedx = x;
  speedy = y;
  speedz = z;
}




void odometry_system() {
  while (1) {
    float vx = float(speedx);     // Desired velocity in x direction
    float vy = float(speedy);     // Desired velocity in y direction
    float omega = float(speedz);  // Desired rotational velocity

    // Calculate individual wheel speeds
    const float r = 20;  //20cm
    int v1 = vx - (1.0 / r) * omega;
    int v2 = (-0.5 * vx) + ((sqrt(3) / 2) * vy) - (1.0 / r) * omega;
    int v3 = (-0.5 * vx) - ((sqrt(3) / 2) * vy) - (1.0 / r) * omega;

    setMotorSpeed(v2, v1, v3);
    threads.delay(50);
  }
}

void colorWipe(int color, int wait) {
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    threads.delay_us(wait);
  }
}
// Function to initialize motors
void initializeMotor() {
  // Set control mode to velocity mode
  writeSingleRegister(CONTROL_MODE_REGISTER, CONTROL_MODE_VELOCITY);

  // Set acceleration and deceleration times to 500 ms
  // writeSingleRegister(ACCELERATION_TIME_LEFT_REGISTER, 500);
  // writeSingleRegister(ACCELERATION_TIME_RIGHT_REGISTER, 500);
  // writeSingleRegister(DECELERATION_TIME_LEFT_REGISTER, 500);
  // writeSingleRegister(DECELERATION_TIME_RIGHT_REGISTER, 500);

  // Enable the motor
  writeSingleRegister(CONTROL_WORD_REGISTER, CONTROL_WORD_ENABLE);
}

// Function to set motor speed
void setMotorSpeed(int speed1, int speed2, int speed3) {
  // Set target velocities
  writeSingleRegister(TARGET_VELOCITY_LEFT_REGISTER, speed3);
  writeSingleRegister(TARGET_VELOCITY_RIGHT_REGISTER, speed2);
  setMotorSpeed_1(speed1);
}

// Function to stop motors
void stopMotor() {
  writeSingleRegister(CONTROL_WORD_REGISTER, CONTROL_WORD_STOP);
}

// Function to write a single register
void writeSingleRegister(uint16_t reg, uint16_t value) {
  modbusFrame[0] = ADDRESS;       // Slave address
  modbusFrame[1] = 0x06;          // Function code (write single register)
  modbusFrame[2] = reg >> 8;      // Register address high byte
  modbusFrame[3] = reg & 0xFF;    // Register address low byte
  modbusFrame[4] = value >> 8;    // Value high byte
  modbusFrame[5] = value & 0xFF;  // Value low byte

  uint16_t crc = calculateCRC(modbusFrame, 6);  // Calculate CRC
  modbusFrame[6] = crc & 0xFF;                  // CRC low byte
  modbusFrame[7] = crc >> 8;                    // CRC high byte

  // Send modbus frame
  Serial2.write(modbusFrame, 8);
  Serial2.flush();  // Wait for transmission to complete

  // Wait for response (adjust timing as needed based on your Modbus device)
  delay(10);
}

void sendModbusCommand(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value) {
  uint8_t frame[8];
  frame[0] = address;
  frame[1] = functionCode;
  frame[2] = highByte(registerAddress);
  frame[3] = lowByte(registerAddress);
  frame[4] = highByte(value);
  frame[5] = lowByte(value);
  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);

  // Send frame
  Serial2.write(frame, 8);
  Serial2.flush();  // Wait for the transmission to complete

  // For debugging
  // Serial.print("Sent frame: ");
  // for (int i = 0; i < 8; i++) {
  //   Serial.print(frame[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
}

// Function to calculate CRC
uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}


void turnOnMotor_1() {
  // Enable the motor driver
  sendModbusCommand(4, 0x06, 0x2031, 0x0008);
  delay(100);

  // Start the motor
  sendModbusCommand(4, 0x06, 0x2031, 0x0010);
}

void setMotorSpeed_1(int speed) {
  // Set motor speed
  sendModbusCommand(4, 0x06, 0x203A, speed);
}



void send_toNuc() {
  while (1) {

    Serial.println(String(x_pos) + ";" + String(y_pos) + ";" + String(bno_z) + ";" + String(step));
    threads.delay(50);
  }
}

void receive_fromNuc() {
  while (1) {

    if (Serial.available() > 0) {
      String dataSerial = Serial.readStringUntil('#');
      serialParser.parse(dataSerial, ';');
      komando = serialParser.getValue("komando");
      int teman_x = serialParser.getValue("x").toFloat();
      int teman_y = serialParser.getValue("y").toFloat();
      int teman_state = serialParser.getValue("state").toInt();
      int data_dummy1 = serialParser.getValue("dummy2").toInt();
      int data_dummy2 = serialParser.getValue("dummy1").toInt();
      kamera_x = serialParser.getValue("kamera").toInt();
      kamera_y = serialParser.getValue("ally").toInt();
    }
    threads.delay(10);
  }
}

void stop (){
   setMotorSpeed(0, 0, 0);
}

void moveToPosition(int targetX, int targetY) {

  while (true) {

    Serial.println(String(x_pos) + ";" + String(y_pos) + ";" + String(bno_z) + ";" + String(step));
     Serial.flush();
    digitalWrite(PENENDANG, 1);
    if (penggiring_aktif) {
      analogWrite(PWM_PENENDANG_1, 200);
      analogWrite(PWM_PENENDANG_2, 200);
    } else {
      analogWrite(PWM_PENENDANG_1, 0);
      analogWrite(PWM_PENENDANG_2, 0);
    }

    float currentX = x_pos;
    float currentY = y_pos;

    float errorX = targetX - currentX;
    float errorY = targetY - currentY;
    const float tolerance = 30;  // Adjust based on your robot's precision

    if (abs(errorX) < tolerance && abs(errorY) < tolerance) {
      set_speed(0, 0, 0);  // Stop the robot
      encoderS1Count = 0;
      encoderS2Count = 0;
      odometry_aktif = false;
      break;  // Target position reached
    }

    // Calculate speed based on error (simple proportional control)
    float KP = 2.54;
    float speedX = errorX * KP;  // Adjust the gain as needed
    float speedY = errorY * KP;  // Adjust the gain as needed
    float max_speed = 100;
    if (speedX >= max_speed) speedX = max_speed;
    if (speedX <= -max_speed) speedX = -max_speed;

    if (speedY >= max_speed) speedY = max_speed;
    if (speedY <= -max_speed) speedY = -max_speed;

    float error_imu = bno_z;
    if (error_imu >= 180) {
      error_imu = error_imu - 360;
    }
    float error = error_imu;
    float maxError = 20;
    float maxerr_intergral = 10;
    if (error > maxError) {
      error = maxError;
    } else if (error < -maxError) {
      error = -maxError;
    }

    if (abs(error) < maxerr_intergral) {
      integral3 += error;
    } else {
      integral3 = 0;
    }

    // PID control IMU
    float kp = 25.0;
    float ki = 0;
    float kd = 20.0;
    float speedZ = kp * error + ki * integral3 + kd * (error - global_var_last_error_slot3);
    global_var_last_error_slot3 = error;
    //float speedZ = error_imu * 1.5;

    if (speedZ >= 100) speedZ = 100;
    if (speedZ <= -100) speedZ = -100;
    if (yaw_kontrol_odometry_on) set_speed(speedY, speedX, speedZ);
    else set_speed(speedY, speedX, 0);
    digitalWrite(13, !digitalRead(13));
    led_color_set = GREEN;

    delay(50);  // Adjust the delay based on the control loop requirements
  }
}



void run_heading(int deg) {

  while (1) {

    Serial.println(String(x_pos) + ";" + String(y_pos) + ";" + String(bno_z) + ";" + String(step));
     Serial.flush();
    digitalWrite(PENENDANG, 1);
    if (penggiring_aktif) {
      analogWrite(PWM_PENENDANG_1, 200);
      analogWrite(PWM_PENENDANG_2, 200);
    } else {
      analogWrite(PWM_PENENDANG_1, 0);
      analogWrite(PWM_PENENDANG_2, 0);
    }


    float raw_imu_z = bno_z;
    if (raw_imu_z > 180) raw_imu_z = raw_imu_z - 360;
    float error_imu = float(deg) - raw_imu_z;
    float error = error_imu;
    float maxError = 100;
    float maxerr_intergral = 10;

    if (error > maxError) {
      error = maxError;
    } else if (error < -maxError) {
      error = -maxError;
    }

    if (abs(error) < maxerr_intergral) {
      integral3 += error;
    } else {
      integral3 = 0;
    }

    // PID control IMU
    float kp = 25.0;
    float ki = 0;
    float kd = 20.0;
    float speedZ = -(kp * error + ki * integral3 + kd * (error - global_var_last_error_slot3));
    global_var_last_error_slot3 = error;
    //float speedZ = error_imu * 1.5;

    if (speedZ >= 750) speedZ = 750;
    if (speedZ <= -750) speedZ = -750;

    set_speed(0, 0, speedZ);
    led_color_set = WHITE;
    if (abs(speedZ) < 250) {
      encoderS1Count = 0;
      encoderS2Count = 0;
      set_speed(0, 0, 0);  // Stop the robot
      break;               // Target position reached
    }
    led_color_set = WHITE;
    delay(50);  // Adjust the delay based on the control loop requirements
  }
}

void mengejar_bola_vision(int kecepatan_kejar) {

  while (1) {

    int ir = digitalRead(10);  //proxymity sensor

    digitalWrite(PENENDANG, 1);
    if (penggiring_aktif) {
      analogWrite(PWM_PENENDANG_1, 200);
      analogWrite(PWM_PENENDANG_2, 200);
    } else {
      analogWrite(PWM_PENENDANG_1, 0);
      analogWrite(PWM_PENENDANG_2, 0);
    }

    int error = kamera_x;
    float control_gain = 7.5;

    float speedZ = error * control_gain;
    if (speedZ >= 750) speedZ = 750;
    if (speedZ <= -750) speedZ = -750;

    set_speed(kecepatan_kejar, 0, speedZ);
    if (ir == 0) {
      encoderS1Count = 0;
      encoderS2Count = 0;
      set_speed(0, 0, 0);  // Stop the robot
      break;
      //break;
    }
  }
}

void menunggu_oper_vision() {

  while (1) {
    int ir = digitalRead(10);  //proxymity sensor

    digitalWrite(PENENDANG, 1);
    if (penggiring_aktif) {
      analogWrite(PWM_PENENDANG_1, 200);
      analogWrite(PWM_PENENDANG_2, 200);
    } else {
      analogWrite(PWM_PENENDANG_1, 0);
      analogWrite(PWM_PENENDANG_2, 0);
    }

    int error_kamera = kamera_x;
    float control_gain = 7.5;

    float speedX = error_kamera * control_gain;
    if (speedX >= 500) speedX = 500;
    if (speedX <= -500) speedX = -500;


    float error_imu = bno_z;
    if (error_imu >= 180) {
      error_imu = error_imu - 360;
    }
    float error = error_imu;
    float maxError = 20;
    float maxerr_intergral = 10;
    if (error > maxError) {
      error = maxError;
    } else if (error < -maxError) {
      error = -maxError;
    }

    if (abs(error) < maxerr_intergral) {
      integral3 += error;
    } else {
      integral3 = 0;
    }

    // PID control IMU
    float kp = 25.0;
    float ki = 0;
    float kd = 20.0;
    float speedZ = kp * error + ki * integral3 + kd * (error - global_var_last_error_slot3);
    global_var_last_error_slot3 = error;
    //float speedZ = error_imu * 1.5;

    if (speedZ >= 100) speedZ = 100;
    if (speedZ <= -100) speedZ = -100;
    if (yaw_kontrol_odometry_on) set_speed(0, speedX, speedZ);
    else set_speed(0, speedX, 0);
    if (ir == 0) {
      encoderS1Count = 0;
      encoderS2Count = 0;
      set_speed(0, 0, 0);  // Stop the robot
      break;
      //break;
    }
  }
}
