// --- KHAI BÁO CHÂN KẾT NỐI (PIN DEFINITIONS) ---
// Chân điều khiển động cơ (Motor Driver L298N)
#define ENA 13 // PWM Motor A
#define IN1 12
#define IN2 14
#define IN3 27
#define IN4 26
#define ENB 25 // PWM Motor B

// Chân cảm biến (5 mắt: Trái -> Phải)
#define S1 32 // Cảm biến trái ngoài cùng
#define S2 33
#define S3 34 // Cảm biến giữa
#define S4 35
#define S5 39 // Cảm biến phải ngoài cùng

// --- THÔNG SỐ PID (CẦN TINH CHỈNH) ---
// Đây là phần quan trọng nhất để xe chạy mượt
float Kp = 30.0; // Hệ số P: Phản ứng nhanh với lỗi
float Ki = 0.00; // Hệ số I: Khắc phục lỗi tích lũy (thường để 0 với xe line)
float Kd = 20.0; // Hệ số D: Giảm rung lắc, dự đoán lỗi tương lai

// Tốc độ cơ bản
int baseSpeed = 150; // Tốc độ chạy thẳng (0 - 255)
int maxSpeed = 255;

// Biến toàn cục cho PID
int lastError = 0;
int P, I, D;
int PID_value;

// Setup PWM cho ESP32 (ESP32 dùng ledc thay vì analogWrite như Arduino Uno cũ)
const int freq = 30000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;

void setup() {
  Serial.begin(115200);

  // Cấu hình chân cảm biến
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // Cấu hình chân động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Cấu hình PWM cho ESP32
  ledcSetup(pwmChannelA, freq, resolution);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(ENA, pwmChannelA);
  ledcAttachPin(ENB, pwmChannelB);
}

void loop() {
  // 1. Đọc cảm biến và tính toán lỗi (Error)
  int error = getPosition();

  // 2. Tính toán PID
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  // Công thức PID: Output = Kp*P + Ki*I + Kd*D
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  // 3. Điều khiển động cơ dựa trên PID
  motorControl(PID_value);
  
  delay(1); // Delay nhỏ để ổn định
}

// --- HÀM ĐỌC VỊ TRÍ (TRỌNG SỐ) ---
// Trả về giá trị từ -4 đến 4 (0 là giữa line)
// Giả sử Line màu ĐEN (sensor = 1), Nền TRẮNG (sensor = 0)
int getPosition() {
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // Logic ưu tiên để xác định vị trí lệch bao nhiêu
  // Lệch càng nhiều, giá trị error càng lớn
  if (s1==1 && s2==0 && s3==0 && s4==0 && s5==0) return -4; // Lệch trái rất nhiều
  if (s1==1 && s2==1 && s3==0 && s4==0 && s5==0) return -3;
  if (s1==0 && s2==1 && s3==0 && s4==0 && s5==0) return -2; // Lệch trái vừa
  if (s1==0 && s2==1 && s3==1 && s4==0 && s5==0) return -1;
  
  if (s1==0 && s2==0 && s3==1 && s4==0 && s5==0) return 0;  // Đang ở giữa line
  
  if (s1==0 && s2==0 && s3==1 && s4==1 && s5==0) return 1;
  if (s1==0 && s2==0 && s3==0 && s4==1 && s5==0) return 2;  // Lệch phải vừa
  if (s1==0 && s2==0 && s3==0 && s4==1 && s5==1) return 3;
  if (s1==0 && s2==0 && s3==0 && s4==0 && s5==1) return 4;  // Lệch phải rất nhiều

  // Trường hợp mất line (ví dụ đi ra khỏi đường đua)
  // Giữ nguyên lỗi cũ để xe tiếp tục cua theo hướng cũ
  return lastError; 
}

// --- HÀM ĐIỀU KHIỂN ĐỘNG CƠ ---
void motorControl(int pid) {
  // Tính tốc độ cho 2 bánh
  int leftSpeed = baseSpeed + pid;
  int rightSpeed = baseSpeed - pid;

  // Giới hạn tốc độ (không quá 0-255)
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Điều khiển motor A (Trái)
  // Tùy cách đấu dây mà set IN1/IN2 là HIGH/LOW
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  ledcWrite(pwmChannelA, leftSpeed);

  // Điều khiển motor B (Phải)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(pwmChannelB, rightSpeed);
}

#UPDATE
switch (currentState) {
  case RUN_FAST:
    pid_control();
    if (detectCrossLine()) {
       currentState = IGNORE_CROSS; // Gặp ngã tư thì chuyển chế độ
    }
    break;

  case IGNORE_CROSS:
    motor_go_straight_max_speed(); // Cứ lao thẳng, mặc kệ cảm biến
    delay(150); // Chỉ trong tích tắc để vượt qua giao lộ
    currentState = RUN_FAST; // Quay lại chế độ dò line
    break;
    
  case TURN_90_LEFT:
    motor_brake(); // Phanh cháy đường
    motor_spin_left(); // Xoay tại chỗ
    if (sensor_center_sees_line()) {
       currentState = RUN_FAST;
    }
    break;
}