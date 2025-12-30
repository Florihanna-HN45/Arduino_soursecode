//fix lan cuoi
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
void doc_cambien(); 
void dieukhiendongco();
void stop();
void tien(int left, int right);
void phai(int speed); 
void trai(int speed); 
void lui(int left, int right);
void tinh_toan_loi();
float getDistance();
void re_trai_90_do(); 

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
  doc_cambien(); 
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
      stop(); delay(200); 
      phai(180); delay(400); 
      tien(160, 160); delay(400); 
      re_trai_90_do(); 
      return; 
  }

  // --- 2. NGÃ TƯ ---
  if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0) {
      digitalWrite(led, HIGH);
      stop(); delay(50);
      dem_nga4++; 

      if (dem_nga4 == 4) { 
          re_trai_90_do();
      } else {
          tien(180, 180);
          while(value[2] == 0) { doc_cambien(); }
          delay(100); 
      }
      return; 
  }

  // --- 3. NGÃ 3 (LOGIC MỚI CHO LẦN 4) ---
  bool is_ngaba = false;
  
  // Phát hiện T-Trái
  if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[4] == 1) is_ngaba = true;
  // Phát hiện T-Phải
  else if (value[4] == 0 && value[3] == 0 && value[2] == 0 && value[0] == 1) is_ngaba = true;

  if (is_ngaba) {
      stop(); delay(50);
      dem_ngaba++; 

      if (dem_ngaba == 4) {
          // --- XỬ LÝ NGÃ 3 LẦN 3: VƯỢT BẬP BÊNH ---
          
          // // Bước 1: Rẽ trái 90 độ vào hướng bập bênh
          // re_trai_90_do();
          // delay(200); // Ổn định xe

          // // Bước 2: Tăng tốc 25% (165 * 1.25 = ~206 -> set 210)
          // tien(210, 210); 

          // // Bước 3: Chạy mù để leo qua bập bênh
          // // CẢNH BÁO: Cần chỉnh số 1500 này cho vừa độ dài bập bênh
          // delay(1500); 

          // // Bước 4: Dừng lại hoặc để PID tự hạ tốc ở vòng sau
          // stop();
          // delay(100); 
        // Bước 1: Rẽ trái 90 độ
          re_trai_90_do();
          delay(200); // Ổn định

          // Bước 2: Tăng tốc cực đại để leo dốc 14.5 độ
          // Với dốc này, tôi khuyên nên để max tốc hoặc gần max
          tien(220, 220); 

          // Bước 3: Delay theo tính toán (2.4 giây)
          // Thà chạy lố ra đường bằng còn hơn kẹt trên dốc
          delay(2200); //2400?

          // Bước 4: Hãm phanh
          stop();
          delay(200); // Dừng lâu hơn chút để triệt tiêu quán tính lao dốc mạnh
          
          // PID sẽ tự bắt line ở vòng loop tiếp theo
      } 
      else {
          // Các lần khác: Đi thẳng
          tien(180, 180); 
          while(value[2] == 0) { doc_cambien(); } 
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

void re_trai_90_do() {
  trai(180); 
  delay(300); 
  while(digitalRead(35) == 1) {}
  stop(); 
}