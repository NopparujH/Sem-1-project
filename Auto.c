#include <Servo.h>

// ------------------- Pin setup -------------------
#define TRIG_F 2
#define ECHO_F 3
#define TRIG_L 7
#define ECHO_L 6
#define TRIG_R 4
#define ECHO_R 5

#define ENA A1
#define ENB A3
#define IN1 8
#define IN2 9
#define IN3 12
#define IN4 13

#define SERV_PIN A2

Servo gripper;

// ------------------- Global variables -------------------
float distance_F, distance_L, distance_R;
bool hasObject = false;  
// ------------------- Basic motor control -------------------
void moveForward(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  
  // unsigned long start = millis();
  // while (millis() - start < t) {
  //   if (dF() < 10) break; // หยุดถ้ามีสิ่งกีดขวางใกล้เกิน
  // }
  delay(t);
  stopMotor();
}

void WallRight(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  unsigned long start = millis();
  while (millis() - start < t) {
    float distR = dR();
    if (distR > 15 ) {
      break;
    }
    delay(100); // ← ให้เวลา 0.1 วินาทีต่อรอบ
  }
  stopMotor();
}

void WallLeft(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  unsigned long start = millis();
  while (millis() - start < t) {
    float distL = dL();
    if (distL > 15 ) {
      break;
    }
    delay(100); // ← ให้เวลา 0.1 วินาทีต่อรอบ
  }
  stopMotor();
}

void moveBackward(int t) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  delay(t);
  stopMotor();
}

void rotateLeft(int t) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(t);
  stopMotor();
}

void rotateRight(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(t);
  stopMotor();
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
///////////////////for hard code////////////
void BackRight(int t) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 0);
  delay(t);
  stopMotor();
}

void BackLeft(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, 150);
  delay(t);
  stopMotor();
}

void LongB(int t) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  delay(t);
  stopMotor();
}

void LongF(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  delay(t);
  stopMotor();
}

void FLEFT(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 180);
  delay(t);
  stopMotor();
}

void FRIGHT(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 0);
  delay(t);
  stopMotor();
}
// ------------------- Ultrasonic functions -------------------
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  
  unsigned long t = pulseIn(echoPin, HIGH, 25000UL);
  if (t == 0) return 400.0;
  return t * 0.034 / 2.0;
}


float dF() { distance_F = readDistance(TRIG_F, ECHO_F); return distance_F; }
float dL() { distance_L = readDistance(TRIG_L, ECHO_L); return distance_L; }
float dR() { distance_R = readDistance(TRIG_R, ECHO_R); return distance_R; }

// ------------------- Servo functions -------------------
void servoGrab() {  
  gripper.write(160);      // คีบสุด
  delay(700);
  moveForward(300) ;
  stopMotor() ;
  delay(300);
  gripper.write(120);      // คีบสุด
  delay(700);
  hasObject = true;      // บันทึกสถานะว่ากำลังถือของ
}

void checkGrab(){
  gripper.write(120);      // คีบสุด
  delay(200);
}

void servoRelease() {
  if (hasObject) {
    gripper.write(160);   // ค่อยๆ ปล่อย
    delay(700);
    moveBackward(500) ;
    gripper.write(60);   // กลับตำแหน่งพัก
    delay(400);
    hasObject = false;   // ไม่มีของแล้ว
  }
}

// ------------------- Action Plan -------------------
enum Action { FORWARD, LEFT, RIGHT, GRAB, RELEASE, BACKWARD, CHECK,BACKL,BACKR,LONGB,LONGF,FL,FR,WallR,WallL};


Action planA[] = {
  FORWARD, RIGHT, FORWARD, FORWARD, RIGHT, GRAB, BACKL, BACKL, LONGB , LEFT, LEFT, FORWARD,
  WallL,FL, WallR, RIGHT,FR,
  FORWARD, LEFT, FORWARD, RIGHT, WallR, RIGHT, FL, RELEASE
};

Action planB[] = {
  FORWARD, RIGHT, FORWARD, WallR, RIGHT, GRAB, BACKL, BACKL, LONGB , LEFT, LEFT, WallL, FL, 
  WallR, RIGHT,FR,
  FORWARD, LEFT, FORWARD, RIGHT, WallR, RIGHT, FL, RELEASE
};  

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);

  // Motor
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  // Servo
  gripper.attach(SERV_PIN);
  gripper.write(60); // ตำแหน่งเปิดเริ่มต้น

  delay(3000);
}

// ------------------- Main Loop -------------------
void loop() {
  for (int i = 0; i < sizeof(planA) / sizeof(planA[0]); i++) {
    switch (planA[i]) {
      case FORWARD:
        moveForward(350);
        break;
      case LEFT:
        rotateLeft(150);
        break;
      case RIGHT:
        rotateRight(140);
        break;
      case GRAB:
        servoGrab();
        break;
      case RELEASE:
        servoRelease();
        break;
      case BACKWARD:
        moveBackward(200);
        break;
      case CHECK:
        checkGrab() ;
        break ;
      case BACKL:
        BackLeft(200);
        break;
      case BACKR:
        BackRight(350);
        break;
      case LONGB:
        LongB(600);
        break;
      case LONGF:
        LongF(800);
        break;
      case FL:
        FLEFT(1600);
        break;
      case FR :
        FRIGHT(1600);
        break;
      case WallR :
        WallRight(10000);
        break;
      case WallL :
        WallLeft(10000);
        break;
    }
    delay(400);
  }
  while (1);
}