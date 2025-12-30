#include <Arduino.h>

// ================================================================
// 1. CẤU HÌNH PHẦN CỨNG & THÔNG SỐ (TUNING)
// ================================================================

// --- Chân Motor L298N (ESP32) ---
#define IN1 27   // Trái
#define IN2 26
#define IN3 25   // Phải
#define IN4 33
#define ENA 14   // PWM Trái
#define ENB 32   // PWM Phải

// --- Cảm biến Line (5 Mắt) & Vật cản ---
const int sensorPins[5] = {13, 12, 21, 19, 18}; // Từ Trái -> Phải
#define OBSTACLE_PIN 23                         // Cảm biến vật cản

// --- Tốc độ (0 - 255) ---
int baseSpeed = 200;    // Tốc độ chạy thẳng tiêu chuẩn
int maxSpeed  = 255;    // Tốc độ tối đa khi Boost
int turnSpeed = 160;    // Tốc độ khi rẽ cứng (90 độ)

// --- PID Constants (Cần tinh chỉnh thực tế) ---
// Kp: Lỗi bao nhiêu sửa bấy nhiêu. Kd: Dự đoán tương lai để chống rung.
float Kp = 35;  
float Kd = 140; // Kd cao để xử lý khúc cua Snake và chống văng

// ================================================================
// 2. BẢN ĐỒ CHIẾN THUẬT (MAPPING) - ĐỘI XANH
// ================================================================
// Quy ước hành động: 
// 0 = ĐI THẲNG (Ghost Mode - Băng qua giao lộ)
// 1 = RẼ TRÁI (Turn Left)
// 2 = RẼ PHẢI (Turn Right)

// Danh sách 8 điểm "tử thần" cần can thiệp:
// 1. Giao lộ X đầu tiên          -> 0 (Đi thẳng)
// 2. Ngã 3 cụt (Điểm số 4)       -> 2 (Rẽ Phải)
// 3. Ngã 3 dọc (Điểm số 5)       -> 0 (Đi thẳng - Bỏ qua rẽ trái)
// 4. Vào Box/Bập bênh (Điểm 8)   -> 1 (Rẽ Trái)
// 5. Ra Box/Bập bênh (Điểm 10)   -> 2 (Rẽ Phải)
// 6. Giao lộ X thứ hai (Điểm 11) -> 0 (Đi thẳng)
// 7. Ngã 3 dọc (Điểm 13)         -> 0 (Đi thẳng - Bỏ qua rẽ phải)
// 8. Góc vuông cuối (Điểm 14)    -> 1 (Rẽ Trái - Vào Snake)

int mapActions[8] = {0, 2, 0, 1, 2, 0, 0, 1}; 
int totalCheckpoints = 8;

// ================================================================
// 3. BIẾN TOÀN CỤC
// ================================================================
int lastError = 0;
int sensorValues[5];
int checkPointIndex = 0;       // Đếm số giao lộ đã qua
unsigned long cooldownTimer = 0; // Chống đếm trùng 1 giao lộ

// ================================================================
// 4. CÁC HÀM ĐIỀU KHIỂN CƠ BẢN
// ================================================================

void setMotor(int speedLeft, int speedRight) {
  // Điều khiển motor Trái
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    speedLeft = -speedLeft;
  }
  analogWrite(ENA, constrain(speedLeft, 0, 255));

  // Điều khiển motor Phải
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

// Hàm đọc lỗi (-4000 đến 4000)
int readError() {
  long weightedSum = 0;
  int sum = 0;
  bool onLine = false;
  // Trọng số mở rộng: Trái -4000, -2000, 0, 2000, 4000 Phải
  long weights[5] = {-4000, -2000, 0, 2000, 4000}; 

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]); // Giả sử 1 là ĐEN (có line)
    // Nếu cảm biến của bạn 0 là ĐEN thì dùng: sensorValues[i] = !digitalRead(sensorPins[i]);
    
    if (sensorValues[i] == 1) { 
      weightedSum += weights[i];
      sum++;
      onLine = true;
    }
  }

  if (!onLine) {
    // Memory: Mất line thì nhớ vị trí cũ để PID kéo lại cực mạnh
    if (lastError > 0) return 5000; 
    else return -5000;
  }
  return weightedSum / sum;
}

// ================================================================
// 5. CÁC HÀNH ĐỘNG THEO KỊCH BẢN (SCRIPTED ACTIONS)
// ================================================================

// Hành động 0: Đi thẳng bất chấp (Ghost Mode)
void actionGoStraight() {
  // Chạy mù (không dùng sensor) trong một khoảng ngắn để thoát giao lộ
  setMotor(baseSpeed, baseSpeed);
  delay(180); // Tinh chỉnh: 150-200ms tùy tốc độ xe
}

// Hành động 1: Rẽ Trái 90 độ (Tank Turn)
void actionTurnLeft() {
  // 1. Lao lên một chút cho tâm xe vào giữa giao lộ
  setMotor(baseSpeed, baseSpeed);
  delay(100); 
  
  // 2. Quay tại chỗ (Trái lùi, Phải tiến)
  setMotor(-turnSpeed, turnSpeed);
  delay(250); // Quay mù một đoạn
  
  // 3. Quay tiếp đến khi mắt GIỮA chạm line thì dừng
  // Timeout 1s để tránh kẹt
  unsigned long t = millis();
  while (digitalRead(sensorPins[2]) == 0 && (millis() - t < 1000)) {
     // Vẫn giữ quay
  }
  // Dừng chốt hạ để ổn định (tuỳ chọn, đua tốc độ thì bỏ dòng này)
  // stopCar(); delay(50); 
}

// Hành động 2: Rẽ Phải 90 độ (Tank Turn)
void actionTurnRight() {
  setMotor(baseSpeed, baseSpeed);
  delay(100);
  
  setMotor(turnSpeed, -turnSpeed); // Trái tiến, Phải lùi
  delay(250);
  
  unsigned long t = millis();
  while (digitalRead(sensorPins[2]) == 0 && (millis() - t < 1000)) {
     // Wait for line
  }
}

// ================================================================
// 6. 
// ================================================================

bool checkAndHandleIntersection() {
  // Nếu chưa hết thời gian chờ (Cooldown) từ lần đếm trước -> Bỏ qua
  if (millis() - cooldownTimer < 800) return false; 

  // Kiểm tra số lượng mắt đen
  int activeSensors = 0;
  for(int i=0; i<5; i++) if(sensorValues[i] == 1) activeSensors++;

  // ĐIỀU KIỆN PHÁT HIỆN:
  // 1. Gặp vạch ngang (Ngã 4 / Ngã 3 chữ T): >= 4 mắt đen
  // 2. Gặp nhánh rẽ (Ngã 3 lệch): 3 mắt lệch hẳn 1 bên
  bool isIntersection = false;
  if (activeSensors >= 4) isIntersection = true; // Ngã tư/Vạch ngang
  else if (sensorValues[0] && sensorValues[1] && sensorValues[2]) isIntersection = true; // Góc Trái
  else if (sensorValues[2] && sensorValues[3] && sensorValues[4]) isIntersection = true; // Góc Phải

  if (isIntersection) {
    // Đã phát hiện giao lộ! Thực thi kịch bản ngay.
    
    // Safety: Nếu đếm quá số lượng map thì thôi chạy PID thường (về đích rồi)
    if (checkPointIndex >= totalCheckpoints) return false;

    int action = mapActions[checkPointIndex];
    
    switch (action) {
      case 0: actionGoStraight(); break;
      case 1: actionTurnLeft();   break;
      case 2: actionTurnRight();  break;
    }

    // Cập nhật trạng thái
    checkPointIndex++;       // Đã xong 1 điểm
    cooldownTimer = millis(); // Reset bộ đếm thời gian nguội
    lastError = 0;            // Reset PID error để tránh giật sau khi rẽ
    
    return true; // Báo hiệu đã xử lý xong, Loop chính không cần chạy PID nữa
  }

  return false;
}

// ================================================================
// 7. SETUP & LOOP
// ================================================================

void setup() {
  // Setup Motor
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Setup Sensor
  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);
  pinMode(OBSTACLE_PIN, INPUT);
  
  // Khởi động
  stopCar();
  delay(1000); // 1s chuẩn bị tinh thần
}

void loop() {
  // --- 1. ƯU TIÊN VẬT CẢN ---
  if (digitalRead(OBSTACLE_PIN) == LOW) { 
    stopCar();
    // Logic né vật cản viết ở đây nếu cần (hoặc chỉ dừng chờ)
    delay(100); 
    return;
  }

  // --- 2. ĐỌC CẢM BIẾN ---
  int error = readError(); // Hàm này cập nhật luôn mảng sensorValues[]

  // --- 3. KIỂM TRA CHECKPOINT (QUAN TRỌNG NHẤT) ---
  // Nếu gặp giao lộ, hàm này sẽ tự lái xe theo kịch bản và return true.
  // Khi đó ta bỏ qua đoạn PID bên dưới.
  if (checkAndHandleIntersection() == true) {
    return;
  }

  // --- 4. PID CONTROLLER (CHẠY BÁM LINE THƯỜNG) ---
  int P = error;
  int D = error - lastError;
  int motorAdjust = (Kp * P) + (Kd * D);
  lastError = error;

  // Tính toán tốc độ 2 bánh
  int leftSpeed  = baseSpeed + motorAdjust;
  int rightSpeed = baseSpeed - motorAdjust;

  // --- 5. ĐIỀU KHIỂN ĐỘNG CƠ (CÓ CHO PHÉP SỐ ÂM) ---
  // Cho phép motorAdjust lớn hơn baseSpeed -> Bánh xe sẽ quay ngược chiều
  // Giúp ôm cua Snake cực gắt mà không bị văng
  
  leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
}