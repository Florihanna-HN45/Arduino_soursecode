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
int baseSpeed = 190;     // tốc độ nền => Mạnh: tăng lên max 200
int turnSpeed = 160;     // tốc độ khi quay tại chỗ
int maxSpeed  = 240;     // giới hạn tốc độ tuyệt đối

// PID (chỉ dùng P + D)
float Kp = 25;           // độ nhạy
float Kd = 50;           // chống rung (60?)

// // Dùng analog nếu cảm biến hỗ trợ
int threshold = 2000;    // ngưỡng phân biệt trắng / đen (0–4095 ESP32)

// global val
int lastError = 0;
int sensorValues[5];

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

int getPosition() {
  long weightedSum = 0;
  int sum = 0;
  bool onLine = false;

  long weights[5] = {-2000, -1000, 0, 1000, 2000};

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]); 
    // nếu 1 = TRẮNG, 0 = ĐEN thì đảo:
    // sensorValues[i] = !digitalRead(sensorPins[i]);

    if (sensorValues[i] == 1) {   // 1 = đang trên line
      weightedSum += weights[i];
      sum++;
      onLine = true;
    }
  }

  if (!onLine) {
    // mất line: tiếp tục quay về phía lần trước có line
    if (lastError > 0) return 3000;
    else return -3000;
  }

  int position = weightedSum / sum;
  return position;                 // khoảng -2000..+2000
}


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
  // 1. Vật cản (LOW = có vật cản)
  if (digitalRead(OBSTACLE_PIN) == LOW) { // ==LOw - gnd -> kich hoat
    stopCar();
    // tại đây có thể thêm logic né, đổi làn...

    
    delay(1000);
    return;
  }

  // 2. Đọc cảm biến và xử lý ngã tư/cua vuông
  getPosition();          // cập nhật sensorValues[]
  handleSpecialCases();   // nếu không rơi vào case đặc biệt thì nó sẽ return ngay

  // 3. PID line follower
  int error = getPosition();   // đọc lại cho chắc vị trí mới

  int P = error;
  int D = error - lastError;
  int motorAdjust = (Kp * P) + (Kd * D);
  lastError = error;

  int leftSpeed  = baseSpeed + motorAdjust;
  int rightSpeed = baseSpeed - motorAdjust;

  // giới hạn tốc độ
  leftSpeed  = constrain(leftSpeed,  -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // với xe 4 bánh + L298N yếu: không cho lùi nhẹ do PID
  if (leftSpeed < 0)  leftSpeed = 0;
  if (rightSpeed < 0) rightSpeed = 0;

  setMotor(leftSpeed, rightSpeed);

  delay(5);  // vòng lặp nhanh cho PID mượt
}
