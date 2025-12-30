// --- KHAI BÁO CHÂN (CẤU HÌNH CHO ESP32 & L298N) ---
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 32

// Cảm biến Line (5 mắt) - Đổi lại GPIO theo thực tế của em
const int sensorPins[5] = {13, 12, 21, 19, 18}; // Từ Trái qua Phải
// Cảm biến vật cản
#define OBSTACLE_PIN 23 

// --- CÁC THÔNG SỐ TUNING (CHỈNH Ở ĐÂY) ---
int baseSpeed = 180;    // Tốc độ nền (150 - 200 tuỳ pin)
int turnSpeed = 160;    // Tốc độ khi quay tại chỗ
int maxSpeed = 240;     // Giới hạn tốc độ tối đa

float Kp = 25;   // Chỉnh cái này trước (độ nhạy)
float Kd = 60;   // Chỉnh cái này sau (chống rung)

// Biến toàn cục
int lastError = 0;
int sensorValues[5];
int threshold = 2000; // Ngưỡng phân biệt Trắng/Đen (Analog ESP32 là 0-4095)

void setup() {
  // Cấu hình Motor
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Cấu hình Cảm biến
  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);
  pinMode(OBSTACLE_PIN, INPUT);
  
  Serial.begin(115200);
  delay(1000); // Chờ 1s để ổn định
}

// Hàm điều khiển động cơ cơ bản
void setMotor(int speedLeft, int speedRight) {
  // Động cơ Trái
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    speedLeft = -speedLeft;
  }
  analogWrite(ENA, constrain(speedLeft, 0, 255));

  // Động cơ Phải
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    speedRight = -speedRight;
  }
  analogWrite(ENB, constrain(speedRight, 0, 255));
}

// Hàm đọc cảm biến & Tính Error cho PID
int getPosition() {
  long weightedSum = 0;
  int sum = 0;
  bool onLine = false;

  // Trọng số cho 5 mắt: -2000, -1000, 0, 1000, 2000
  long weights[5] = {-2000, -1000, 0, 1000, 2000};

  for (int i = 0; i < 5; i++) {
    // Đọc Digital hoặc Analog (Code này dùng Digital cho nhanh)
    sensorValues[i] = digitalRead(sensorPins[i]); 
    // Nếu dùng Analog: sensorValues[i] = (analogRead(sensorPins[i]) > threshold) ? 1 : 0;
    
    if (sensorValues[i] == 1) { // 1 là ĐEN (tuỳ cảm biến, nếu ngược thì đổi thành 0)
      weightedSum += weights[i];
      sum++;
      onLine = true;
    }
  }

  if (!onLine) {
    // Nếu mất line, giữ nguyên sai số cũ để biết hướng quay về
    if (lastError > 0) return 3000; // Mất bên phải
    else return -3000;              // Mất bên trái
  }

  // Trả về vị trí trung bình
  int position = weightedSum / sum;
  return position;
}

// Xử lý Cua Vuông Góc & Ngã Tư
void handleSpecialCases() {
  // Đọc lại trạng thái
  int s1 = digitalRead(sensorPins[0]); // Trái cùng
  int s2 = digitalRead(sensorPins[1]);
  int s3 = digitalRead(sensorPins[2]); // Giữa
  int s4 = digitalRead(sensorPins[3]);
  int s5 = digitalRead(sensorPins[4]); // Phải cùng

  // 1. NGÃ TƯ (CROSS LINE): Tất cả đều đen hoặc 4/5 đen
  if (s1 && s2 && s3 && s4 && s5) {
    setMotor(baseSpeed, baseSpeed); // Đi thẳng qua luôn
    delay(150); // Mù trong 0.15s để qua vạch ngang
    return;
  }

  // 2. CUA VUÔNG GÓC TRÁI (Mắt trái + Giữa đen)
  if (s1 && s2 && s3) {
    // Quay tại chỗ sang trái
    setMotor(-turnSpeed, turnSpeed); 
    delay(100); // Quay mù một chút cho thoát vạch hiện tại
    // Đợi đến khi mắt giữa vào line
    while (digitalRead(sensorPins[2]) == 0) {
       setMotor(-turnSpeed, turnSpeed);
    }
    return;
  }

  // 3. CUA VUÔNG GÓC PHẢI (Mắt phải + Giữa đen)
  if (s3 && s4 && s5) {
    // Quay tại chỗ sang phải
    setMotor(turnSpeed, -turnSpeed);
    delay(100);
    while (digitalRead(sensorPins[2]) == 0) {
       setMotor(turnSpeed, -turnSpeed);
    }
    return;
  }
}

void loop() {
  // 1. Kiểm tra vật cản
  if (digitalRead(OBSTACLE_PIN) == LOW) { // Nếu phát hiện vật cản
    setMotor(0, 0); // Dừng lại
    delay(1000);    // Chờ 1s (hoặc xử lý né)
    return;
  }

  // 2. Xử lý trường hợp đặc biệt (Ngã tư/Góc vuông)
  handleSpecialCases();

  // 3. Tính toán PID
  int error = getPosition();
  
  // Logic PID
  int P = error;
  int D = error - lastError;
  int motorAdjust = (Kp * P) + (Kd * D); // Bỏ I
  
  lastError = error;

  // 4. Áp dụng tốc độ
  int leftSpeed = baseSpeed + motorAdjust;
  int rightSpeed = baseSpeed - motorAdjust;
  
  // Giới hạn max speed
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Xe 4 bánh L298N yếu thì không nên cho quay lùi khi chạy thẳng (chỉ lùi khi cua gắt)
  // Nếu speed < 0 ở đoạn PID này, có thể set = 0 để tránh giật cục
  // setMotor(leftSpeed, rightSpeed); 
  
  // Code tối ưu cho L298N:
  setMotor(leftSpeed, rightSpeed);
}