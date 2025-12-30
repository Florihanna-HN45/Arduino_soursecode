#include <Arduino.h>

// --- 1. ĐỊNH NGHĨA CHÂN (PINS) ---
#define IN1 18   // Động cơ Trái
#define IN2 19
#define IN3 21   // Động cơ Phải
#define IN4 22
#define ENA 16   // PWM Trái
#define ENB 23   // PWM Phải

// Chân cảm biến
const int sensorPins[5] = {25, 34, 35, 32, 33}; 

// Cài đặt PWM cho ESP32
const int freq = 1000;      
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;

// --- 2. BIẾN TOÀN CỤC (GLOBALS) ---
// Tốc độ
int baseSpeed = 130;    // Tốc độ chạy thẳng (Tinh chỉnh từ 150-240)
int maxSpeed  = 150;    // Giới hạn max

// PID Parameters
float Kp = 35;  
float Ki = 0.0;         // Line follower thường để Ki rất nhỏ hoặc bằng 0
float Kd = 140; 

float P, I, D;
float previousError = 0;
float pidValue = 0;
int lastError = 0;
int sensorValues[5];

// --- 3. KHỞI TẠO (SETUP) ---
void setup() {
  Serial.begin(115200);

  // Cấu hình chân động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Cấu hình chân cảm biến
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Cấu hình PWM cho ESP32
  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(ENA, pwmChannelA);
  
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(ENB, pwmChannelB);
}

// --- 4. HÀM ĐIỀU KHIỂN ĐỘNG CƠ (ACTUATOR) ---
void setMotor(int speedLeft, int speedRight) {
  // Giới hạn tốc độ trong khoảng -255 đến 255
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  // --- MOTOR TRÁI ---
  if (speedLeft >= 0) {
    digitalWrite(IN1, LOW);   
    digitalWrite(IN2, HIGH);  
    ledcWrite(pwmChannelA, speedLeft);
  } else {
    digitalWrite(IN1, HIGH);  // Đảo chiều để lùi/phanh gấp
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannelA, abs(speedLeft));
  }

  // --- MOTOR PHẢI ---
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH);  
    digitalWrite(IN4, LOW);   
    ledcWrite(pwmChannelB, speedRight);
  } else {
    digitalWrite(IN3, LOW);   // Đảo chiều để lùi/phanh gấp
    digitalWrite(IN4, HIGH);
    ledcWrite(pwmChannelB, abs(speedRight));
  }
}

// --- 5. HÀM ĐỌC CẢM BIẾN (SENSOR LOGIC) ---
int readError() {
  bool onLine = false;
  int error = 0;
  int activeSensors = 0; // Đếm số lượng mắt thấy line
  
  // 1. Đọc cảm biến
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
    // sensorValues[i] = !sensorValues[i]; // Đảo ngược nếu cần (1 là đen)

    if (sensorValues[i] == 1) {
      onLine = true;
      activeSensors++;
    }
  }

  // --- ƯU TIÊN 1: MẤT LINE (LOST) ---
  // Yêu cầu: Không tìm thấy line thì quay phải để tìm
  if (!onLine) {
    // Trả về lỗi cực đại Dương (5) để PID điều khiển motor Trái chạy mạnh, Phải lùi
    // -> Xe sẽ xoay tròn sang phải tại chỗ để quét tìm line.
    return 5; 
  }

  // --- ƯU TIÊN 2: GIAO LỘ CHỮ X HOẶC VẠCH NGANG ---
  // Đặc điểm: Quá nhiều mắt sáng cùng lúc (>=3) hoặc 2 mắt bìa cùng sáng (bị kẹp giữa chữ X)
  // Hành động: Cưỡng ép đi thẳng (Error = 0) để vượt qua vùng nhiễu
  if (activeSensors >= 3 || (sensorValues[0] == 1 && sensorValues[4] == 1)) {
    lastError = 0; // Reset bộ nhớ lỗi
    return 0;      // Robot đi thẳng
  }

  // --- ƯU TIÊN 3: GÓC VUÔNG (RẼ GẤP) ---
  // Đặc điểm: Mắt giữa (2) MẤT line, nhưng mắt bìa (0,1 hoặc 3,4) LẠI CÓ line.
  
  // Góc vuông TRÁI (Mắt 0,1 sáng, Mắt 2 tắt)
  if (sensorValues[2] == 0 && (sensorValues[0] == 1 || sensorValues[1] == 1)) {
     // Kiểm tra thêm: Nếu bên phải hoàn toàn trắng -> Chắc chắn là rẽ trái
     if (sensorValues[3] == 0 && sensorValues[4] == 0) {
        lastError = -5;
        return -5; // Lỗi cực đại âm -> Rẽ trái gấp
     }
  }

  // Góc vuông PHẢI (Mắt 3,4 sáng, Mắt 2 tắt)
  if (sensorValues[2] == 0 && (sensorValues[3] == 1 || sensorValues[4] == 1)) {
     // Kiểm tra thêm: Nếu bên trái hoàn toàn trắng -> Chắc chắn là rẽ phải
     if (sensorValues[0] == 0 && sensorValues[1] == 0) {
        lastError = 5;
        return 5; // Lỗi cực đại dương -> Rẽ phải gấp
     }
  }

  // --- ƯU TIÊN 4: ĐƯỜNG CONG THƯỜNG (PID) ---
  // Logic cũ để bám line mượt mà
  
  // Lệch TRÁI (-)
  if      (sensorValues[0]==1 && sensorValues[1]==0) error = -4;
  else if (sensorValues[0]==1 && sensorValues[1]==1) error = -3;
  else if (sensorValues[0]==0 && sensorValues[1]==1) error = -2;
  else if (sensorValues[1]==1 && sensorValues[2]==1) error = -1;
  
  // Ở GIỮA (0)
  else if (sensorValues[2]==1) error = 0;

  // Lệch PHẢI (+)
  else if (sensorValues[2]==1 && sensorValues[3]==1) error = 1;
  else if (sensorValues[3]==1 && sensorValues[4]==0) error = 2;
  else if (sensorValues[3]==1 && sensorValues[4]==1) error = 3;
  else if (sensorValues[4]==1 && sensorValues[3]==0) error = 4;

  lastError = error;
  return error;
}
// --- 6. HÀM TÍNH TOÁN PID (BRAIN) ---
void calculatePID() {
  int error = readError();

  P = error;
  I = I + error;
  D = error - previousError;
  
  // Anti-windup cho I
  if (I > 50) I = 50;
  if (I < -50) I = -50;

  pidValue = (Kp * P) + (Ki * I) + (Kd * D);
  
  previousError = error;
}

// --- 7. CHƯƠNG TRÌNH CHÍNH (MAIN LOOP) ---
void loop() {
  // B1: Tính PID
  calculatePID();

  // B2: Tính tốc độ 2 bánh (Differential Steering)
  // Nếu muốn rẽ TRÁI (pid < 0) -> Giảm Trái, Tăng Phải
  int speedLeft = baseSpeed + pidValue;
  int speedRight = baseSpeed - pidValue;

  // B3: Đẩy ra động cơ
  setMotor(speedLeft, speedRight);
  
  // Không delay
}