#include <Arduino.h>
//2812: sua tang toc
'''
- Update:
1) Thêm PID curve
2) thêm tocdo_curve 140
3) them gioi han tich phan trong PID (tinh toan PID)
4) them ham reset_pid() sau khi ne vat can
'''
//=====
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
#define freq 5000 // 10000
#define reslution 8
#define PWM_CHANNEL_A  0 
#define PWM_CHANNEL_B  1

const int socambien = 5; 
int cambien[socambien] = {25, 34, 35, 32, 33};  
int value[socambien];   


// XỬ LÝ LINE ĐỨT
unsigned long line_lost_time = 0;
bool is_line_lost = false;
#define SHORT_LOSS_TIMEOUT 400
#define LONG_LOSS_TIMEOUT 1000

int last_dir = 0; // lưu hướng cuối -1 = trái, 0 = giữa , 1 = phải
int dead_speed = 150; // speed đi mù =))
// ======================================================1. 
// --- BIẾN ĐẾM NGÃ 3 (MỚI) ---
int dem_ngaba = 0; // Biến lưu số lần gặp ngã 3

// --- CHỈNH PID ---
float kp = 40;  
float ki = 0.002; 
float kd = 8;  
//12thJan
//PID cua gấp
float kp_curve = 50;
float kd_curve = 12;
//

float setpoit = 0.0;
float read_last_value = 0.0;

unsigned long last_timer = 0;
float last_error = 0;
float tichphan = 0;

int tocdo = 180; 
int tocdo_curve = 140;// tang toc 2812
int tocdoc_max = 255; 

// =========================--- KHAI BÁO HÀM ---==============================
void dieukhiendongco();
void stop();
void tien(int left, int right);
void phai(int speed); 
void trai(int speed); 
void lui(int left, int right);
void tinh_toan_loi();
float getDistance();
void reset_pid();
void phanh_gap();
void re_phai_90_do();
void re_trai_90_do();

//====================================TINH TOAN====================

float controller_pid(float setvalue, float readvalue, float kp, float ki, float kd){
  unsigned long now_timer = millis();
  float dt = (now_timer - last_timer) / 1000.0;
  if(dt <= 0) dt = 0.001;

  float error = setvalue - readvalue;
  tichphan += error * dt;
  // them gioi han tich luy
  tichphan = constrain(tichphan, -50, 50);

  float daoham = (error - last_error) / dt;

  float output = (kp * error) + (ki * tichphan) + (kd * daoham);

  last_error = error;
  last_timer = now_timer;

  return output;
}
//=========vị trí sensor==============================================
void tinh_toan_loi() {
  float error_sum = 0;
  int active_sensors = 0;
  
  if (value[0] == 0) { error_sum += (-2); active_sensors++; } 
  if (value[1] == 0) { error_sum += (-1); active_sensors++; } 
  if (value[2] == 0) { error_sum += (0); active_sensors++; } 
  if (value[3] == 0) { error_sum += (1); active_sensors++; } 
  if (value[4] == 0) { error_sum += (2); active_sensors++; } 

  if (active_sensors > 0) { // TINH VỊ TRÍ TRUNG BÌNH
    read_last_value = error_sum / active_sensors;
    update_direction(); // update hướng
  }
  // Neu khong thay line, doc theo vi tri cuoi
}
// =====================VỊ TRÍ SIÊU ÂM ===================================
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

//======================================MOTOR===========================
void dieukhiendongco(){
  tinh_toan_loi();
  float distance = getDistance();

  // --- 1. ƯU TIÊN VẬT CẢN (< 25cm) ---
  if (distance > 0 && distance < 25) {
      digitalWrite(led, HIGH);
      stop(); 
      delay(100); 
      phai(180);
      delay(400); // Né
      tien(160, 160); delay(400);

      reset_pid() // them ham reset PID
      return; 
  }

  // 2. PHÁT HIỆN NGÃ 3 CHỮ T
  // ========================================
  bool is_T_left = (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[4] == 1);
  bool is_T_right = (value[4] == 0 && value[3] == 0 && value[2] == 0 && value[0] == 1);
  bool is_intersection = is_T_left || is_T_right;
  
  // Kiểm tra cooldown để tránh đếm lại
  if (is_intersection && (millis() - lastIntersectionTime > INTERSECTION_COOLDOWN)) {
      digitalWrite(led, HIGH);
      lastIntersectionTime = millis();
      
      dem_ngaba++; // Tăng đếm
      
      Serial.print("Ngã 3 số: ");
      Serial.println(dem_ngaba);
      
      if (dem_ngaba == 3) {
          // ========================================
          // LẦN THỨ 3: RẼ THEO HƯỚNG
          // ========================================
          stop();
          delay(200);  // Dừng lại ổn định
          
          if (is_T_left) {
              Serial.println("Rẽ TRÁI tại ngã 3 thứ 3");
              re_trai_90_do();
          } else {
              Serial.println("Rẽ PHẢI tại ngã 3 thứ 3");
              re_phai_90_do();
          }
          
          reset_pid();  // QUAN TRỌNG: Reset PID sau khi rẽ
          // dem_ngaba = 0; // Uncomment nếu muốn lặp lại chu trình
          
      } else {
          // ========================================
          // LẦN 1, 2: ĐI THẲNG QUA
          // ========================================
          Serial.println("Đi thẳng qua ngã 3");
          tien(160, 160); 
          delay(300);  // Đi mù để thoát khỏi vạch ngang
          
          reset_pid();  // Reset PID để không bị shock khi ra khỏi ngã 3
      }
      
      return; // Thoát hàm
  }

  // ========================================
  // 3. ADAPTIVE PID - Điều chỉnh theo độ cong
  // ========================================
  float kp_use = kp;
  float kd_use = kd;
  
  // Nếu lỗi lớn (đang ở cua gấp) → dùng PID mạnh hơn
  if (abs(read_position) > 1.2) {
      kp_use = kp_curve;
      kd_use = kd_curve;
      tocdo = tocdo_cua;  // Giảm tốc
  } else {
      tocdo = tocdo_thang;  // Tăng tốc ở đường thẳng
  }
  
  // Tính PID
  float output_pid = controller_pid(setpoit, read_position, kp_use, ki, kd_use);
  
  int left = tocdo - output_pid;  
  int right = tocdo + output_pid; 
  left = constrain(left, 0, tocdoc_max);
  right = constrain(right, 0, tocdoc_max);

  // --- 4. CÁC TRƯỜNG HỢP KHÁC ---
  
  // Mất line -> Xoay tìm
  if(value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1){
    if (read_last_value < 0) {
      re_trai_90_do();
    }     
    else {
      re_phai_90_do();
    }
    reset_pid();
    return; //update
  }
  // Ngã tư hoặc đi thẳng
  else if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0) {
      tien(tocdo, tocdo); // Ngã 4 đi thẳng (không tính vào đếm ngã 3)
  }
  else {
    digitalWrite(led, LOW);
    tien(left, right); // Chạy PID
  }
}
//==================================SETUP=========================================
void setup(){
  // Serial.begin(9600);
  Serial.begin(115200); // tang baudrate cho nhanh? update
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
//======================================LOOP==============================
void loop(){
  for(int i = 0; i < socambien; i++){
    value[i] = digitalRead(cambien[i]);                   
  }
  dieukhiendongco();
}
// ========================================
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ
//=======================================
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

void phai(int speed){ // Quay tại chỗ
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH); // Trái Tiến
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH); // Phải Lùi
}

void trai(int speed){ // Quay tại chỗ
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); // Trái Lùi
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); // Phải Tiến
}

void lui(int left, int right){
  ledcWrite(PWM_CHANNEL_A, left);
  ledcWrite(PWM_CHANNEL_B, right);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(75);
}
void phanh_gap() {
  // Đảo chiều dòng điện để hãm động cơ
  // Giả sử: tien() là in2 HIGH, in3 HIGH
  // Thì phanh là: in1 HIGH, in4 HIGH
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  
  // Cấp lực phanh cực đại
  ledcWrite(PWM_CHANNEL_A, 255); 
  ledcWrite(PWM_CHANNEL_B, 255); 
  
  delay(60); // Thời gian phanh (Tùy chỉnh: 30ms - 60ms)
             // Đủ để dừng nhưng không chạy lùi
  
  stop(); // Ngắt điện hoàn toàn
  delay(50); // Dừng thêm 1 tí cho ổn định (tuỳ chọn)
}
// HÀM RẼ PHẢI CHUẨN (Dùng cảm biến)
void re_phai_90_do() {
  phai(200); 
  delay(60); 
  while(digitalRead(35) == 1) {
    // Chờ bắt line
  }
  phanh_gap();
}
// HÀM RẼ TRÁI CHUẨN (Dùng cảm biến)
void re_trai_90_do() {
  trai(200); // Bắt đầu quay nhanh
  delay(60); // Quay mù một đoạn để mắt giữa thoát khỏi line hiện tại
  
  // Quay tiếp cho đến khi mắt giữa (value[2] - chân 35) gặp màu ĐEN (0)
  while(digitalRead(35) == 1) {
    // Chờ cho đến khi bắt được line mới
    // Có thể thêm timeout để tránh quay vòng tròn mãi
  }
  phanh_gap(); // Dừng quay ngay khi thấy line
}
