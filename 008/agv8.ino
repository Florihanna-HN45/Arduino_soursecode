#include <Arduino.h>

#define IN1 18  // Trái
#define IN2 19
#define IN3 21  // Phải
#define IN4 22
#define ENA 16  // PWM Trái
#define ENB 23  // PWM Phải

const int sensorPins[5] = {25, 34, 35, 32, 33}; 
#define TRIG_PIN 26
#define ECHO_PIN 27
int baseSpeed = 200;    // Tốc độ chạy thẳng
int maxSpeed  = 255;    // Tốc độ tối đa giới hạn
int turnSpeed = 180;    // Tốc độ khi rẽ 90 độ (cần mạnh để thắng ma sát)

// --- PID Constants ---
float Kp = 35;   // Độ nhạy
float Kd = 140;  // Chống rung (Dự đoán)

// ================================================================
// 2. BẢN ĐỒ CHIẾN THUẬT (MAPPING)
// ================================================================
// 0 = ĐI THẲNG (Ghost Mode - Dùng cho ngã tư hoặc chữ X)
// 1 = RẼ TRÁI
// 2 = RẼ PHẢI

// Checkpoint list (Dựa trên Map Đội Xanh):
// 1. X đầu tiên (Đi thẳng) -> 2. Ngã 3 cụt (Phải) -> 3. Ngã 3 dọc (Thẳng)
// 4. Vào Box (Trái) -> 5. Ra Box (Phải) -> 6. X thứ hai (Thẳng)
// 7. Ngã 3 dọc (Thẳng) -> 8. Góc vuông cuối (Trái)
int mapActions[8] = {0, 2, 0, 1, 2, 0, 0, 1}; 
int totalCheckpoints = 8;

int lastError = 0;
int sensorValues[5];
int checkPointIndex = 0;       // Đếm số giao lộ đã qua
unsigned long cooldownTimer = 0; // Thời gian chờ giữa 2 giao lộ

void setMotor(int speedLeft, int speedRight) {
  // --- Motor Trái ---
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    speedLeft = -speedLeft;
  }
  analogWrite(ENA, constrain(speedLeft, 0, 255));

  // --- Motor Phải ---
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    speedRight = -speedRight;
  }
  analogWrite(ENB, constrain(speedRight, 0, 255));
}

void stopCar() {
  setMotor(0, 0);
}

int readError() {
  long weightedSum = 0;
  int sum = 0;
  bool onLine = false;
  long weights[5] = {-4000, -2000, 0, 2000, 4000}; 

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]); // 1 là ĐEN
    if (sensorValues[i] == 1) { 
      weightedSum += weights[i];
      sum++;
      onLine = true;
    }
  }

  if (!onLine) {
    if (abs(lastError) <= 1500) { 
        return 0; 
    }
    if (lastError > 0) return 5000; 
    else return -5000;
  }
  
  return weightedSum / sum;
}
// X - ngã 4 đường phố =))
void actionGoStraight() {
  setMotor(baseSpeed, baseSpeed);
  delay(180); // Chạy mù 180ms để thoát vùng giao lộ
}

void actionTurnLeft() {
  setMotor(baseSpeed, baseSpeed); delay(100); // Lao lên chút
  setMotor(-turnSpeed, turnSpeed); delay(250); // Quay mù
  
  unsigned long t = millis();
  // Chờ mắt GIỮA (sensor 2) chạm đen
  while (digitalRead(sensorPins[2]) == 0 && (millis() - t < 1500)) {
     // Loop chờ
  }
}

void actionTurnRight() {
  setMotor(baseSpeed, baseSpeed); delay(100);
  setMotor(turnSpeed, -turnSpeed); delay(250);
  
  unsigned long t = millis();
  while (digitalRead(sensorPins[2]) == 0 && (millis() - t < 1500)) {
     // Loop chờ
  }
}
void avoidObstacle() {
  stopCar(); delay(200);
  setMotor(-150, -150); delay(200); // Lùi nhẹ

  // 1. Rẽ PHẢI ra
  setMotor(turnSpeed, -turnSpeed); delay(400); 
  
  // 2. Đi thẳng qua hông vật cản
  setMotor(baseSpeed, baseSpeed); delay(500);

  // 3. Rẽ TRÁI (song song)
  setMotor(-turnSpeed, turnSpeed); delay(400);

  // 4. Đi thẳng tiếp
  setMotor(baseSpeed, baseSpeed); delay(500);

  // // 5. Rẽ TRÁI (về line)
  // setMotor(-turnSpeed, turnSpeed); delay(400);

  // 6. Đi từ từ bắt line
  setMotor(150, 150);
  while(digitalRead(sensorPins[2]) == 0) {
    // Chờ bắt line
  }
  setMotor(150, -150); delay(100); // Chỉnh thẳng xe
}

bool checkAndHandleIntersection() {
  // Cooldown 0.8 giây giữa các giao lộ để không đếm trùng
  if (millis() - cooldownTimer < 800) return false; 

  // Đếm số mắt đen
  int activeSensors = 0;
  for(int i=0; i<5; i++) if(sensorValues[i] == 1) activeSensors++;

  // Điều kiện phát hiện: 4 mắt đen trở lên HOẶC 3 mắt lệch không đều
  bool isIntersection = false;
  if (activeSensors >= 4) isIntersection = true; // Ngã tư / Chữ X
  else if (sensorValues[0] && sensorValues[1] && sensorValues[2]) isIntersection = true; // Góc Trái
  else if (sensorValues[2] && sensorValues[3] && sensorValues[4]) isIntersection = true; // Góc Phải

  if (isIntersection) {
    if (checkPointIndex >= totalCheckpoints) return false;

    int action = mapActions[checkPointIndex];
    
    switch (action) {
      case 0: actionGoStraight(); break;
      case 1: actionTurnLeft();   break;
      case 2: actionTurnRight();  break;
    }

    checkPointIndex++;       // Tăng biến đếm
    cooldownTimer = millis(); // Reset bộ đếm thời gian
    lastError = 0;           
    return true; // Báo hiệu đã xử lý xong
  }

  return false;
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);
  pinMode(OBSTACLE_PIN, INPUT);
  
  stopCar();
  delay(1000); // 1s chờ đợi
}

void loop() {
  // --- 1. ƯU TIÊN VẬT CẢN (Cao nhất) ---
  if (digitalRead(OBSTACLE_PIN) == LOW) { // Nếu cảm biến mức 0 là có vật
    avoidObstacle();
    return; // Né xong thì quay lại đầu loop
  }

  // --- 2. ĐỌC CẢM BIẾN ---
  int error = readError(); 

  // --- 3. KIỂM TRA CHECKPOINT ---
  // Nếu gặp giao lộ -> Tự lái -> Return ngay để không chạy PID
  if (checkAndHandleIntersection() == true) {
    return;
  }

  // --- 4. PID CONTROL (Chạy bám line thường) ---
  int P = error;
  int D = error - lastError;
  int motorAdjust = (Kp * P) + (Kd * D);
  lastError = error;

  int leftSpeed  = baseSpeed + motorAdjust;
  int rightSpeed = baseSpeed - motorAdjust;

  leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
}