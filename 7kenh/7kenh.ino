
#include <Arduino.h>

// --- CẤU HÌNH CHÂN MOTOR ---
#define in1 18
#define in2 19
#define in3 21
#define in4 22
#define led 2 
#define ENA 16
#define ENB 23

// --- CẤU HÌNH SIÊU ÂM ---
#define TRIG_PIN 26   
#define ECHO_PIN 27  

// --- CẤU HÌNH CẢM BIẾN LINE (7 MẮT) ---
// Thứ tự: [0:Trái Cùng] [1:Trái] [2:Trái Giữa] [3:Giữa] [4:Phải Giữa] [5:Phải] [6:Phải Cùng]
const int socambien = 7; 
// Lưu ý: Đã thêm GPIO 4 và 5 cho 2 mắt mới. Hãy nối đúng thứ tự từ Trái sang Phải.
int cambien[socambien] = {4, 25, 34, 35, 32, 33, 5};  
int value[socambien];   

// --- CẤU HÌNH PWM ---
#define freq 5000
#define reslution 8
#define PWM_CHANNEL_A  0 
#define PWM_CHANNEL_B  1

// --- BIẾN ĐẾM & TRẠNG THÁI ---
int dem_ngaba = 0; 
int dem_nga4 = 0;  

// --- CHỈNH PID ---
// Với 7 mắt, sai số lớn hơn nên có thể giảm Kp một chút hoặc giữ nguyên tùy thực tế
float kp = 30;  
float ki = 0.002; 
float kd = 25;  

float setpoit = 0.0; // Mục tiêu là 0 (giữa)
float read_last_value = 0.0;

unsigned long last_timer = 0;
float last_error = 0;
float tichphan = 0;

int tocdo = 150; // Tốc độ nền (Giảm chút để test cho an toàn)
int tocdoc_max = 255; 

// --- KHAI BÁO HÀM ---
void dieukhiendongco();
void stop();
void tien(int left, int right);
void phai(int speed); 
void trai(int speed); 
void smart_turn_left(); // Hàm rẽ thông minh mới
void smart_turn_right(); // Hàm rẽ thông minh mới
void pass_crossline();   // Hàm đi qua vạch kẻ ngang
float getDistance();
void tinh_toan_loi();

// --- HÀM PID ---
float controller_pid(float setvalue, float readvalue, float kp, float ki, float kd){
  unsigned long now_timer = millis();
  float dt = (now_timer - last_timer) / 1000.0;
  if(dt <= 0) dt = 0.001;

  float error = setvalue - readvalue;
  tichphan += error * dt;
  float daoham = (error - last_error) / dt;

  float output = (kp * error) + (ki * tichphan) + (kd * daoham);

  last_error = error;
  last_timer = now_timer;

  return output;
}

// --- TÍNH TOÁN LỖI (7 MẮT) ---
void tinh_toan_loi() {
  float error_sum = 0;
  int active_sensors = 0;
  
  // Trọng số mở rộng: -3, -2, -1, 0, 1, 2, 3
  if (value[0] == 0) { error_sum -= 3; active_sensors++; } // Mắt trái xa mới
  if (value[1] == 0) { error_sum -= 2; active_sensors++; } 
  if (value[2] == 0) { error_sum -= 1; active_sensors++; } 
  if (value[3] == 0) { error_sum += 0; active_sensors++; } // Mắt giữa
  if (value[4] == 0) { error_sum += 1; active_sensors++; } 
  if (value[5] == 0) { error_sum += 2; active_sensors++; } 
  if (value[6] == 0) { error_sum += 3; active_sensors++; } // Mắt phải xa mới

  if (active_sensors > 0) {
    read_last_value = error_sum / active_sensors;
  }
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 5000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

// --- HÀM XỬ LÝ CHÍNH ---
void dieukhiendongco(){
  tinh_toan_loi();
  float distance = getDistance();

  // 1. ƯU TIÊN VẬT CẢN (< 15cm)
  if (distance > 0 && distance < 15) {
      digitalWrite(led, HIGH);
      stop(); delay(200);
      
      // Né vật cản (Logic cũ của bạn)
      phai(180); delay(450); // Quay phải
      tien(150, 150); delay(500); // Đi thẳng
      trai(180); delay(450); // Quay lại hướng cũ
      // Lưu ý: Nên viết lại đoạn né này bằng smart logic sau
      return; 
  }

  // 2. NHẬN DIỆN NGÃ 4 (Dựa vào 2 mắt ngoài cùng 0 và 6) /// sẽ sai nếu là ngẵ chứ Y
  if (value[0] == 0 && value[6] == 0) { 
      digitalWrite(led, HIGH);
      stop(); delay(100);
      dem_nga4++; 

      if (dem_nga4 == 4) {
          // Lần 4 -> Rẽ Trái (Dùng Smart Turn)
          smart_turn_left();
      } else {
          // Chưa đủ lần -> Đi thẳng qua vạch
          pass_crossline();
      }
      return; 
  }

  // 3. NHẬN DIỆN NGÃ 3 CHỮ T (Trái)
  // Mắt trái ngoài (0) đen, nhưng mắt phải ngoài (6) trắng
  else if (value[0] == 0 && value[6] == 1) {
      dem_ngaba++;
      stop(); delay(100);

      if (dem_ngaba == 3) {
          // Lần 3 -> Rẽ Trái
          smart_turn_left();
      } else {
          pass_crossline();
      }
      return;
  }
  
  // 4. NHẬN DIỆN NGÃ 3 CHỮ T (Phải)
  // Mắt phải ngoài (6) đen, mắt trái ngoài (0) trắng
  else if (value[6] == 0 && value[0] == 1) {
      dem_ngaba++;
      stop(); delay(100);
      
      // Giả sử T-Phải cũng đếm chung vào biến dem_ngaba
      // Nếu logic của bạn muốn rẽ Phải ở ngã 3 thì dùng smart_turn_right()
      if (dem_ngaba == 3) { 
         // Tùy map của bạn, nếu muốn rẽ trái ở T-Phải thì dùng trai()
         // Ở đây tôi để rẽ Phải cho đúng logic T-Phải
         smart_turn_right();
      } else {
         pass_crossline();
      }
      return;
  }

  // 5. CHẠY PID BÌNH THƯỜNG
  float output_pid = controller_pid(setpoit, read_last_value, kp, ki, kd);
  int left = tocdo - output_pid;  
  int right = tocdo + output_pid; 
  left = constrain(left, 0, tocdoc_max);
  right = constrain(right, 0, tocdoc_max);

  // Xử lý khi mất line hoàn toàn (tất cả đều 1)
  if(value[0]==1 && value[1]==1 && value[2]==1 && value[3]==1 && value[4]==1 && value[5]==1 && value[6]==1){
    // Dùng giá trị cũ để quyết định quay lại
    if (read_last_value < 0) { // Lần cuối lệch trái
       trai(140); // Quay nhẹ tìm đường
    } else {
       phai(140);
    }
  } else {
    digitalWrite(led, LOW);
    tien(left, right);
  }
}

// --- CÁC HÀM HỖ TRỢ RẼ THÔNG MINH ---

void pass_crossline() {
  // Đi thẳng cho đến khi 2 mắt ngoài cùng hết đen (thoát khỏi vạch ngang)
  tien(160, 160);
  delay(150); // Nhích lên 1 đoạn để chắc chắn vào vạch
  // Có thể thêm vòng while chờ thoát vạch nếu vạch quá dày
  // while(digitalRead(cambien[0]) == 0 || digitalRead(cambien[6]) == 0) { tien(160,160); }
}

void smart_turn_left() {
  // 1. Căn tâm: Đi thêm một chút để trục bánh xe trùng vạch rẽ
  tien(150, 150);
  delay(200); // Tinh chỉnh số này: Nếu rẽ sớm quá thì tăng, rẽ muộn quá thì giảm
  
  // 2. Phanh chết
  stop();
  delay(300);

  // 3. Quay mù (Blind turn) để thoát khỏi vạch hiện tại
  trai(160);
  delay(300); 

  // 4. Quay dò (Smart turn): Quay tiếp cho đến khi mắt GIỮA (3) thấy vạch đen
  while (digitalRead(cambien[3]) == 1) {
    trai(150); // Tốc độ chậm để bắt chính xác
  }

  // 5. Dừng lại ngay khi thấy vạch
  stop();
  delay(200); // Ổn định trước khi chạy PID tiếp
  
  // Reset các biến PID để tránh giật
  last_error = 0;
  tichphan = 0;
}

void smart_turn_right() {
  // Tương tự rẽ trái
  tien(150, 150);
  delay(200); 
  
  stop();
  delay(300);

  phai(160);
  delay(300); 

  while (digitalRead(cambien[3]) == 1) {
    phai(150);
  }

  stop();
  delay(200);
  
  last_error = 0;
  tichphan = 0;
}

// --- SETUP ---
void setup(){
  Serial.begin(9600);
  ledcSetup(PWM_CHANNEL_A, freq, reslution);
  ledcSetup(PWM_CHANNEL_B, freq, reslution);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);

  pinMode(led, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stop();

  // Khai báo 7 mắt
  for(int i = 0; i < socambien; i++){
    pinMode(cambien[i], INPUT);
  }
}

void loop(){
  // Đọc liên tục 7 cảm biến
  for(int i = 0; i < socambien; i++){
    value[i] = digitalRead(cambien[i]);                   
  }
  dieukhiendongco();
}

// --- CÁC HÀM CƠ BẢN ---

void stop(){
  // PHANH GẤP (BRAKE) thay vì thả trôi
  digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
  ledcWrite(PWM_CHANNEL_A, 255); // Lực phanh tối đa
  ledcWrite(PWM_CHANNEL_B, 255);
}

void tien(int left, int right){
  ledcWrite(PWM_CHANNEL_A, left);
  ledcWrite(PWM_CHANNEL_B, right);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void phai(int speed){ 
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
}

void trai(int speed){ 
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); 
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
}

void lui(int left, int right){
  ledcWrite(PWM_CHANNEL_A, left);
  ledcWrite(PWM_CHANNEL_B, right);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}