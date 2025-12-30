#include <Arduino.h>

// ================================================================
// 1. CẤU HÌNH PHẦN CỨNG & THÔNG SỐ (TUNING)
// ================================================================

// --- Chân Motor L298N (ESP32) ---
#define IN1 18  // Trái
#define IN2 19
#define IN3 21   // Phải
#define IN4 22
#define ENA 16   // PWM Trái
#define ENB 23   // PWM Phải

// --- Cảm biến Line (5 Mắt) & Vật cản ---
const int sensorPins[5] = {25, 34, 35, 32, 33}; // Từ Trái -> Phải
#define Trig 26
#define Echo 27                         // Cảm biến vật cản

// --- Tốc độ (0 - 255) ---
int baseSpeed = 200;    // Tốc độ chạy thẳng tiêu chuẩn
int maxSpeed  = 255;    // Tốc độ tối đa khi Boost
int turnSpeed = 160;    // Tốc độ khi rẽ cứng (90 độ)

// --- PID Constants (Cần tinh chỉnh thực tế) ---
// Kp: Lỗi bao nhiêu sửa bấy nhiêu. Kd: Dự đoán tương lai để chống rung.
float Kp = 35;  
float Kd = 140; // Kd cao để xử lý khúc cua Snake và chống văng


int mapActions[8] = {0, 2, 0, 1, 2, 0, 0, 1}; 
int totalCheckpoints = 8;

int lastError = 0;
int sensorValues[5];
int checkPointIndex = 0;       // Đếm số giao lộ đã qua
unsigned long cooldownTimer = 0; // Chống đếm trùng 1 giao lộ

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
  long weights[5] = {-4000, -2000, 0, 2000, 4000}; 

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]); 
    if (sensorValues[i] == 1) { 
      weightedSum += weights[i];
      sum++;
      onLine = true;
    }
  }

  if (!onLine) {
    if (!onLine) {
    // nếu vừa mới mất line < 100ms: giữ error cũ
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

// ================================================================
// 5. CÁC HÀNH ĐỘNG THEO KỊCH BẢN (SCRIPTED ACTIONS)
// ================================================================
void runPIDLine() {
  int error = readError();
  int P = error;
  int D = error - lastError;
  int motorAdjust = Kp * P + Kd * D;
  lastError = error;

  int leftSpeed  = baseSpeed + motorAdjust;
  int rightSpeed = baseSpeed - motorAdjust;

  leftSpeed  = constrain(leftSpeed,  -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
}
bool handleCrossX() {
  int active = 0;
  for (int i = 0; i < 5; i++)
    if (sensorValues[i]) active++;

  // pattern X: nhiều mắt đen cả 2 bên, tương đối cân giữa
  bool isX = (active >= 4);   // có thể thêm điều kiện: s2 && s3 && s4

  if (!isX) return false;

  // chỉ cho phép hành động 0 = đi thẳng, không bao giờ rẽ ở X
  setMotor(baseSpeed, baseSpeed);

  // thời gian “ghost mode” tuỳ tốc độ:
  int ghostTime = 180; // thử 160–220 ms trên sân thật
  delay(ghostTime);

  lastError = 0;
  return true;
}



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
void avoidObstacle() {
  setMotor(100, -100); //
  delay(300);          // Thời gian tinh chỉnh: ~300ms (tùy xe)

  // BƯỚC 2: Chạy thẳng một đoạn ngắn để vượt qua vật cản
  setMotor(150, 150);
  delay(400);

obstacleAvoided = true;
leftTurnUsedAfterObstacle = false;
fullLineCount = 0;


  // BƯỚC 3: KHÔNG DỪNG! — Để vòng lặp chính tiếp tục chạy PID
  // PID sẽ tự động kéo xe quay lại line nếu còn thấy line
}
// ================================================================
// 6. XỬ LÝ GIAO LỘ & CHECKPOINT (TRÁI TIM CHIẾN THUẬT)
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

//Lo trinh retrai
// Đã từng né vật cản chưa?
bool obstacleAvoided = false;

// Sau khi né vật cản, ta đã quyết định rẽ trái tại checkpoint đặc biệt chưa?
bool leftTurnUsedAfterObstacle = false;

// Đếm số lần “gặp pattern 4-5 mắt sáng” sau đó (gần đích)
int fullLineCount = 0;

//===RE TRAI DE MAY
bool retrai() {
  bool s0 = sensorValues[0];
  bool s1 = sensorValues[1];
  bool s2 = sensorValues[2];
  bool s3 = sensorValues[3];
  bool s4 = sensorValues[4];

  // 1) Đếm số lần 4–5 mắt cùng sáng (gần đích)
  int active = s0 + s1 + s2 + s3 + s4;
  if (active >= 4) {
    fullLineCount++;
  }

  // Nếu đã qua nhiều đoạn “full line” (4 lần trở lên) -> bỏ qua logic rẽ, để PID tự chạy (gần đích)
  if (fullLineCount >= 4) {
    return false;  // không can thiệp nữa
  }

  // 2) Kiểm tra pattern 3 cảm biến trái sáng (tự rẽ trái)
  bool leftPattern = (s0 && s1 && s2);   // bạn có thể siết điều kiện thêm: s3,s4 = 0

  // 3) Nếu chưa từng né vật cản -> đây là rẽ trái bình thường theo kịch bản
  if (!obstacleAvoided) {
    if (leftPattern) {
      // rẽ trái “auto”
      setMotor(-turnSpeed, turnSpeed);
      delay(250); // chỉnh thực nghiệm
      // chờ đến khi mắt giữa thấy line
      unsigned long t0 = millis();
      while (digitalRead(sensorPins[2]) == 0 && millis() - t0 < 800) {
        setMotor(-turnSpeed, turnSpeed);
      }
      lastError = 0;
      return true;
    }
    return false;  // chưa có gì đặc biệt
  }

  // 4) ĐÃ né vật cản rồi:

  // 4a) Nếu sau khi né vật cản mà vẫn đang thấy line ổn (không mất line) -> ưu tiên đi thẳng
  if (obstacleAvoided && !leftTurnUsedAfterObstacle && leftPattern) {
    // Lần ĐẦU gặp pattern rẽ trái sau khi né vật cản: ép đi thẳng
    setMotor(baseSpeed, baseSpeed);
    delay(150);       // chạy thẳng khỏi vùng giao
    leftTurnUsedAfterObstacle = true;   // đã dùng “đi thẳng sau obstacle”
    lastError = 0;
    return true;
  }

  // 4b) Nếu đã né vật cản, đã cố đi thẳng nhưng BỊ MẤT LINE (line đứt / chập chờn)
  // -> cho phép rẽ trái để tìm lại line
  bool onLine = (active > 0);
  if (obstacleAvoided && leftTurnUsedAfterObstacle && !onLine) {
    // mất line sau obstacle -> quay phải tìm line trước (theo yêu cầu của bạn)
    unsigned long t0 = millis();
    setMotor(turnSpeed, -turnSpeed);         // quay phải tìm line
    while (digitalRead(sensorPins[2]) == 0 && millis() - t0 < 600) {
      // đợi đến khi mắt giữa thấy line
    }

    // nếu vẫn chưa thấy line -> thử rẽ trái mạnh
    if (digitalRead(sensorPins[2]) == 0) {
      t0 = millis();
      while (digitalRead(sensorPins[2]) == 0 && millis() - t0 < 600) {
        setMotor(-turnSpeed, turnSpeed);     // quay trái
      }
    }
    lastError = 0;
    return true;
  }

  return false;  // không rơi vào case nào -> PID bám line
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
  // 1. Đọc cảm biến để cập nhật sensorValues[]
  readError();
  if (handleObstacle()) return;    
  if (handleSmartLeftTurn()) return;  // logic rẽ trái đặc biệt

  // 2. Ưu tiên: vật cản
  if (digitalRead(TRIG == LOW, ECHO == LOW) {
    avoidObstacle();
    return;
  }

  // 3. Ưu tiên: X 120 độ
  if (handleCrossX()) return;

  // 4. Ưu tiên: checkpoint / ngã 3
  if (checkAndHandleIntersection()) return;

  // 5. Cuối cùng: PID bám line thường
  runPIDLine();
}
