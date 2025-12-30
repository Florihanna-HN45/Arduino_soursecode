#include <Arduino.h>

// ===== 1. KHAI BÁO CHÂN (ESP32 + L298N) =====
// L298N: kênh A điều khiển 2 motor bên TRÁI, kênh B điều khiển 2 motor bên PHẢI
#define IN1 27   // Trái
#define IN2 26
#define IN3 25   // Phải
#define IN4 33
#define ENA 14   // PWM trái
#define ENB 32   // PWM phải

// Cảm biến line 5 mắt (từ TRÁI sang PHẢI) – đổi theo đấu dây thực tế
const int sensorPins[5] = {13, 12, 21, 19, 18};

// Cảm biến vật cản (IR / công tắc) – LOW = có vật cản
#define OBSTACLE_PIN 23

// ===== 2. THÔNG SỐ TUNING =====
int baseSpeed = 200;     // tốc độ nền => Mạnh: tăng lên max 200
int turnSpeed = 160;     // tốc độ khi quay tại chỗ
int maxSpeed  = 255;     // giới hạn tốc độ tuyệt đối

// PID (chỉ dùng P + D)
// Tăng Kp, Kd lên vì ta sẽ cho phép motor quay ngược
float Kp = 35;          // Tăng độ nhạy
float Kd = 120;         // Tăng cực mạnh để chống văng ở cua gắt (Snake)

// Dùng analog nếu cảm biến hỗ trợ
int threshold = 2000;    // ngưỡng phân biệt trắng / đen (0–4095 ESP32)

// global val
int lastError = 0;
int sensorValues[5];

// Biến cho State Machine (Xử lý ngã tư)
unsigned long crossOverTimer = 0;
bool isCrossing = false;

// ===== 3. HÀM ĐIỀU KHIỂN ĐỘNG CƠ =====
void setMotor(int speedLeft, int speedRight) {
  // Bên trái
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speedLeft = -speedLeft; //dam bao gia tri luon duong
  }
  speedLeft = constrain(speedLeft, 0, 255); // ham constrain la ham gioi han gia tri
  analogWrite(ENA, speedLeft); // cong enable, kenh A - trai=> xung vuong dieu chinh toc do

  // Bên phải
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speedRight = -speedRight;
  }
  speedRight = constrain(speedRight, 0, 255);
  analogWrite(ENB, speedRight);
}

void stopCar() {
  setMotor(0, 0);
}

// ===== 4. ĐỌC CẢM BIẾN & TÍNH ERROR =====
// // trả về vị trí line: khoảng -2000..+2000
// int getPosition() {
//   long weightedSum = 0;
//   int sum = 0;
//   bool onLine = false;

//   long weights[5] = {-2000, -1000, 0, 1000, 2000};

//   for (int i = 0; i < 5; i++) {
//     // Nếu dùng digital:
//     // int raw = digitalRead(sensorPins[i]);
//     // sensorValues[i] = raw;

//     // Dùng analog cho ổn định hơn:
//     int raw = analogRead(sensorPins[i]);
//     // giả sử: gặp LINE ĐEN => giá trị THẤP hơn threshold
//     sensorValues[i] = (raw < threshold) ? 1 : 0;

//     if (sensorValues[i] == 1) {  // 1 = đang trên line đen
//       weightedSum += weights[i];
//       sum++;
//       onLine = true;
//     }
//   }

//   if (!onLine) {
//     // mất line: quay về phía lần trước đang lệch
//     if (lastError > 0) return 3000;   // line ở bên phải
//     else return -3000;                // line ở bên trái
//   }

//   int position = weightedSum / sum;
//   return position;
// }

// other
int lastError = 0;

int readSensorsAndGetError() {
  long weightedSum = 0;
  int sum = 0;
  bool onLine = false;
  // Trọng số mở rộng để PID nét hơn
  long weights[5] = {-4000, -2000, 0, 2000, 4000}; 

  for (int i = 0; i < 5; i++) {
    int val = digitalRead(sensorPins[i]); 
    // Tùy cảm biến: 1 là line hay 0 là line. Giả sử 1 là line đen
    sensorValues[i] = val; 
    
    if (val == 1) { 
      weightedSum += weights[i];
      sum++;
      onLine = true;
    }
  }
  if (!onLine) {
    // Logic "Memory": Nhớ vị trí cuối cùng
    // Trả về giá trị cực đại để PID phản ứng cực mạnh (kéo xe lại ngay)
    if (lastError > 0) return 5000; 
    else return -5000;
  }
  
  // Trả về Error
  return weightedSum / sum;
}

//001: Xử lý các bài toán trong model - X, .. (kích thước vật cản??)
// Thêm biến toàn cục để quản lý trạng thái
unsigned long intersectionTimer = 0;
bool isLocked = false; 

void handleIntersectionX() {
  // Đếm số lượng cảm biến nhìn thấy vạch đen (giả sử 1 là đen)
  int activeSensors = 0;
  for(int i=0; i<5; i++) {
    if(sensorValues[i] == 1) activeSensors++; 
  }

  // === GIAI ĐOẠN 1: PHÁT HIỆN GIAO LỘ ===
  // Bình thường chỉ có 1 hoặc 2 mắt đen. 
  // Nếu thấy >= 4 mắt đen => Đang ở tâm giao lộ X
  if (activeSensors >= 4) { 
    isLocked = true;
    intersectionTimer = millis(); // Bắt đầu đếm giờ
  }

  // === GIAI ĐOẠN 2: XỬ LÝ (GHOST MODE) ===
  if (isLocked) {
    // Trong chế độ khóa, ta cưỡng ép xe đi thẳng
    // Bỏ qua PID, set cứng tốc độ 2 bên bằng nhau
    setMotor(baseSpeed, baseSpeed); 
    
    // Debug cho ngầu (nếu có LED thì bật sáng báo hiệu)
    // digitalWrite(LED_BUILTIN, HIGH); 

    // === GIAI ĐOẠN 3: THOÁT HIỂM ===
    // Điều kiện thoát: 
    // 1. Đã qua 1 khoảng thời gian tối thiểu (ví dụ 100ms) để tránh nhiễu
    // 2. VÀ số lượng sensor hoạt động đã giảm xuống mức bình thường (<= 2)
    if ((millis() - intersectionTimer > 100) && (activeSensors <= 2)) {
      isLocked = false; // Mở khóa, trả quyền lại cho PID
      // digitalWrite(LED_BUILTIN, LOW);
    }
    
    // CỰC KỲ QUAN TRỌNG:
    // Return true để báo cho loop chính biết là "Tao đang xử lý rồi, đừng chạy PID nữa"
    return true; 
  }

  return false; // Không có gì đặc biệt, chạy PID đi
}
//002: 
// ===== 5. XỬ LÝ NGÃ TƯ / CUA VUÔNG =====
void handleSpecialCases() {
  int s1 = sensorValues[0]; // trái ngoài
  int s2 = sensorValues[1];
  int s3 = sensorValues[2]; // giữa
  int s4 = sensorValues[3];
  int s5 = sensorValues[4]; // phải ngoài

  // --- Ngã tư: hầu hết sensor thấy line ---
  if (s1 && s2 && s3 && s4 && s5) {
    // chiến thuật đơn giản: đi thẳng
    setMotor(baseSpeed, baseSpeed);
    delay(150);            // "mù" 0.15 s cho qua vạch ngang
    return;
  }

  // --- Cua vuông trái: 3 mắt trái đều 1 ---
  if (s1 && s2 && s3 && !s4 && !s5) {
    unsigned long t0 = millis();
    // quay tại chỗ sang trái
    while (analogRead(sensorPins[2]) >= threshold && millis() - t0 < 600) {
      setMotor(-turnSpeed, turnSpeed);
      delay(5);
    }
    return;
  }

  // --- Cua vuông phải: 3 mắt phải đều 1 ---
  if (!s1 && !s2 && s3 && s4 && s5) {
    unsigned long t0 = millis();
    while (analogRead(sensorPins[2]) >= threshold && millis() - t0 < 600) {
      setMotor(turnSpeed, -turnSpeed);
      delay(5);
    }
    return;
  }
}

// ===== 6. SETUP =====
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  pinMode(OBSTACLE_PIN, INPUT);

  Serial.begin(115200);
  delay(1000);
}

// ===== 7. LOOP CHÍNH =====
void loop() {
  // 1. Đọc cảm biến (Hàm optimize tôi viết ở comment trước)
  int error = readSensorsAndGetError(); 

  // 2. Kiểm tra giao lộ X TRƯỚC KHI tính PID
  // Nếu đang xử lý giao lộ (hàm trả về true) thì bỏ qua phần dưới, quay lại đầu loop
  if (handleIntersectionX() == true) {
      return; 
  }

  // 3. Tính toán PID (Chỉ chạy khi không ở giao lộ)
  int P = error;
  int D = error - lastError;
  int motorAdjust = (Kp * P) + (Kd * D);
  lastError = error;

  int leftSpeed  = baseSpeed + motorAdjust;
  int rightSpeed = baseSpeed - motorAdjust;
  
  // Constrain và setMotor như cũ...
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
  setMotor(leftSpeed, rightSpeed);
}
