#include <Arduino.h>

// --- KHAI BÁO CHÂN MOTOR ---
#define IN1 18  // Trái
#define IN2 19
#define IN3 21  // Phải
#define IN4 22
#define ENA 16  // PWM Trái
#define ENB 23  // PWM Phải

// --- KHAI BÁO CẢM BIẾN ---
const int sensorPins[5] = {25, 34, 35, 32, 33}; 

// --- SỬA: KHAI BÁO SIÊU ÂM (ULTRASONIC) ---
#define TRIG_PIN 26
#define ECHO_PIN 27

// --- THÔNG SỐ TỐC ĐỘ ---
int baseSpeed = 200;    
int maxSpeed  = 255;    
int turnSpeed = 180;    

// --- PID Constants ---
float Kp = 35;   
float Kd = 140;  

int lastError = 0;
int sensorValues[5];
unsigned long cooldownTimer = 0; 

// Khoảng cách phát hiện vật cản (cm)
const int OBSTACLE_DISTANCE = 15; 

void setup() {
  // Cấu hình Motor
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Cấu hình Line Sensor
  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);
  
  // Cấu hình Siêu âm
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Khởi động Serial để debug nếu cần
  Serial.begin(9600);
  
  // Dừng xe 1s đầu
  setMotor(0, 0);
  delay(1000); 
}

// Hàm điều khiển động cơ
void setMotor(int speedLeft, int speedRight) {
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    speedLeft = -speedLeft;
  }
  analogWrite(ENA, constrain(speedLeft, 0, 255));

  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    speedRight = -speedRight;
  }
  analogWrite(ENB, constrain(speedRight, 0, 255));
}

// Hàm đo khoảng cách (Non-blocking tương đối)
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout 3000ms (~50cm) để robot không bị khựng nếu không thấy vật
  long duration = pulseIn(ECHO_PIN, HIGH, 5000); 
  
  if (duration == 0) return 999; // Không thấy gì thì trả về số lớn
  return duration * 0.034 / 2;
}

// Hàm đọc lỗi Line (Weighted Average)
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
    if (millis() - lastSeenTime < 100) {
      return lastError;
    }
    // mất lâu rồi: quay về phía cũ
    if (lastError > 0) return 5000;
    else return -5000;
  }
    else {
    lastSeenTime = millis();  // có line -> cập nhật thời gian
  }
  return weightedSum / sum;
}
// --- CÁC HÀM HÀNH ĐỘNG ---
void actionGoStraight() {
  setMotor(baseSpeed, baseSpeed);
  delay(200); // Chạy mù qua giao lộ
}

// Hàm né vật cản (Sửa lại logic một chút cho mượt hơn)
void avoidObstacle() {
  setMotor(0, 0); delay(200);
  setMotor(-150, -150); delay(200); // Lùi lại chút
  
  // 1. Rẽ sang phải (Giả sử né bên phải)
  setMotor(turnSpeed, -turnSpeed); delay(350);
  
  // 2. Đi thẳng
  setMotor(baseSpeed, baseSpeed); delay(500);
  
  // 3. Rẽ trái song song
  setMotor(-turnSpeed, turnSpeed); delay(350);
  
  // 4. Đi thẳng vượt qua
  setMotor(baseSpeed, baseSpeed); delay(500);
  
  // 5. Rẽ trái về line (Vừa đi vừa dò)
  setMotor(100, 180); // Cua trái nhẹ
  while(digitalRead(sensorPins[2]) == 0) {
   
  }
  
  // 6. Ổn định
  setMotor(-150, 150); delay(50); // Phanh nhẹ để thẳng xe
}

// --- LOGIC XỬ LÝ GIAO LỘ KHÔNG CẦN MAP ---
void handleIntersectionReflex() {
  // Nếu vừa xử lý xong trong 0.5s thì bỏ qua để tránh nhận diện kép
  if (millis() - cooldownTimer < 500) return; 

  int activeSensors = 0;
  for(int i=0; i<5; i++) if(sensorValues[i] == 1) activeSensors++;

  // Phát hiện: 4 mắt đen trở lên HOẶC 2 mắt bìa cùng đen
  bool isCross = (activeSensors >= 4) || (sensorValues[0] && sensorValues[4]);

  if (isCross) {
    // CHIẾN THUẬT: Gặp ngã tư -> Ưu tiên ĐI THẲNG để cắt qua chữ X
    actionGoStraight();
    
    cooldownTimer = millis();
    lastError = 0; 
  }
  // Lưu ý: Code này chưa xử lý các góc vuông bắt buộc phải rẽ (nếu map có).
  // Nếu map có góc vuông (không có đường thẳng), robot sẽ trượt ra ngoài
  // và hàm readError() sẽ trả về +/- 5000 để kéo nó lại.
}

void loop() {
  // 1. KIỂM TRA VẬT CẢN (Ưu tiên số 1)
  // Chỉ kiểm tra mỗi 50ms để đỡ tốn thời gian đo
  static unsigned long lastCheckDist = 0;
  if (millis() - lastCheckDist > 50) {
      int dist = getDistance();
      if (dist > 0 && dist < OBSTACLE_DISTANCE) {
        avoidObstacle();
        return; 
      }
      lastCheckDist = millis();
  }

  // 2. ĐỌC LINE
  int error = readError(); 

  // 3. XỬ LÝ GIAO LỘ (KHÔNG MAP)
  handleIntersectionReflex();

  // 4. PID CONTROL
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