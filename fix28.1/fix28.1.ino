#include <Arduino.h>

// --- CẤU HÌNH CHÂN ---
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

// --- CẤU HÌNH PWM ---
#define freq 5000
#define reslution 8
#define PWM_CHANNEL_A  0 
#define PWM_CHANNEL_B  1

const int socambien = 5; 
int cambien[socambien] = {25, 34, 35, 32, 33};  
int value[socambien];   

// --- BIẾN ĐẾM ---
int dem_ngaba = 0; 
int dem_nga4 = 0;  

// --- CHỈNH PID ---
float kp = 35;  
float ki = 0.002; 
float kd = 5;  

float setpoit = 0.0;
float read_last_value = 0.0;

unsigned long last_timer = 0;
float last_error = 0;
float tichphan = 0;

int tocdo = 165; 
int tocdoc_max = 255; 

// --- KHAI BÁO HÀM ---
void doc_cambien(); // BẮT BUỘC CÓ
void dieukhiendongco();
void stop();
void tien(int left, int right);
void phai(int speed); 
void trai(int speed); 
void lui(int left, int right);
void tinh_toan_loi();
float getDistance();
void re_trai_90_do(); // <--- MỚI THÊM

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

  for(int i = 0; i < socambien; i++){
    pinMode(cambien[i], INPUT);
  }
}

// --- LOOP ---
void loop(){
  // Không cần đọc cảm biến ở đây nữa vì đã có hàm doc_cambien()
  dieukhiendongco();
}

// --- CÁC HÀM XỬ LÝ ---

void doc_cambien() {
  for(int i = 0; i < socambien; i++){
    value[i] = digitalRead(cambien[i]);                   
  }
}

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

void tinh_toan_loi() {
  doc_cambien(); // Gọi hàm đọc tại đây để có dữ liệu mới nhất
  float error_sum = 0;
  int active_sensors = 0;
  
  if (value[0] == 0) { error_sum -= 2; active_sensors++; } 
  if (value[1] == 0) { error_sum -= 1; active_sensors++; } 
  if (value[2] == 0) { error_sum += 0; active_sensors++; } 
  if (value[3] == 0) { error_sum += 1; active_sensors++; } 
  if (value[4] == 0) { error_sum += 2; active_sensors++; } 

  if (active_sensors > 0) {
    read_last_value = error_sum / active_sensors;
  }
}

float getDistance() {
  // Đo 3 lần lấy trung bình để chống nhiễu (Khuyên dùng)
  float total = 0;
  int count = 0;
  for(int i=0; i<3; i++){
      digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      long d = pulseIn(ECHO_PIN, HIGH, 5000); 
      if(d > 0) { total += d; count++; }
      delay(2);
  }
  if (count == 0) return 999; 
  return (total/count) * 0.034 / 2;
}

void dieukhiendongco(){
  tinh_toan_loi();
  float distance = getDistance();

  // --- 1. VẬT CẢN ---
  if (distance > 0 && distance < 22) { 
      digitalWrite(led, HIGH);
      stop(); delay(200); // Dừng hẳn để hết quán tính
      
      // Quy trình né: Phải -> Thẳng -> Trái (Về line)
      phai(180); delay(400); 
      tien(160, 160); delay(400); 
      
      // Đoạn quay về line: Dùng re_trai_90_do sẽ an toàn hơn delay mù
      re_trai_90_do(); 
      return; 
  }

  // --- 2. NGÃ TƯ (SÂN XANH: RẼ TRÁI) ---
  if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0) {
      digitalWrite(led, HIGH);
      stop(); delay(50);
      dem_nga4++; 

      if (dem_nga4 == 4) { 
          // [THAY ĐỔI]: Gọi hàm rẽ chuẩn xác
          re_trai_90_do();
      } else {
          // Đi thẳng qua ngã tư
          tien(180, 180);
          // Vòng lặp chờ: Chạy đến khi mắt giữa hết đen (thoát ngã tư)
          while(value[2] == 0) {
             doc_cambien(); // Cập nhật liên tục để biết khi nào thoát đen
          }
          delay(100); // Chạy lố thêm tí xíu cho chắc
      }
      return; 
  }

  // --- 3. NGÃ 3 (SÂN XANH) ---
  // T-TRÁI
  if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[4] == 1) {
      stop(); delay(50); 
      dem_ngaba++; 
      if (dem_ngaba == 3) { 
          // [THAY ĐỔI]: Gọi hàm rẽ chuẩn xác
          re_trai_90_do();
      } else {
          tien(180, 180); 
          while(value[2] == 0) { doc_cambien(); } // Chạy thoát vạch
          delay(100);
      }
      return; 
  }
  
  // T-PHẢI
  else if (value[4] == 0 && value[3] == 0 && value[2] == 0 && value[0] == 1) {
      stop(); delay(50);
      dem_ngaba++; 
      if (dem_ngaba == 4) {
           trai(200); 
           delay(450); 
      } else {
          tien(180, 180); 
          while(value[2] == 0) { doc_cambien(); } // Chạy thoát vạch
          delay(100);
      }
      return; 
  }

  // --- 4. PID ---
  float output_pid = controller_pid(setpoit, read_last_value, kp, ki, kd);
  int left = tocdo - output_pid;  
  int right = tocdo + output_pid; 
  left = constrain(left, 0, tocdoc_max);
  right = constrain(right, 0, tocdoc_max);

  // --- 5. MẤT LINE ---
  if(value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1){
    if (read_last_value < 0) trai(160); 
    else phai(160); 
  }
  else {
    digitalWrite(led, LOW);
    tien(left, right); 
  }
}

// --- CÁC HÀM MOTOR ---

void stop(){
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0); ledcWrite(PWM_CHANNEL_B, 0);
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
  delay(75);
}

// [HÀM MỚI]:
void re_trai_90_do() {
  trai(180); // Bắt đầu quay
  delay(300); // Quay mù để thoát line cũ (tinh chỉnh số này nếu xe quay quá ít/nhiều)
  
  // Quay đến khi mắt giữa (chân 35) gặp vạch đen (mức 0)
  // digitalRead(35) == 1 nghĩa là đang thấy trắng -> tiếp tục quay
  while(digitalRead(35) == 1) {
    // Không làm gì, chỉ chờ
  }
  stop(); // Thấy đen -> Dừng
}