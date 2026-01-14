#include <Arduino.h>

// =============================================
// XỬ LÝ LINE NÉT ĐỨT - DEAD RECKONING
// Giải pháp không quay tròn tại chỗ
// =============================================

// --- CẤU HÌNH CHÂN ---
#define in1 18
#define in2 19
#define in3 21
#define in4 22
#define led 2 

#define ENA 16
#define ENB 23

#define TRIG_PIN 26   
#define ECHO_PIN 27  

// --- PWM ---
#define freq 5000
#define reslution 8
#define PWM_CHANNEL_A  0 
#define PWM_CHANNEL_B  1

const int socambien = 5; 
int cambien[socambien] = {25, 34, 35, 32, 33};  
int value[socambien];   

// --- BIẾN XỬ LÝ LINE ĐỨT (MỚI) ---
unsigned long line_lost_time = 0;      // Thời điểm mất line
bool is_line_lost = false;              // Đang trong trạng thái mất line
#define SHORT_LOSS_TIMEOUT 400          // 400ms = Coi là line đứt ngắn
#define LONG_LOSS_TIMEOUT 1000          // 1000ms = Mất line thật sự

int last_direction = 0;                 // Lưu hướng cuối: -1=trái, 0=giữa, 1=phải
int dead_reckoning_speed = 150;         // Tốc độ khi đi mù

// --- NGÃ 3 ---
int dem_ngaba = 0;
unsigned long lastIntersectionTime = 0;
#define INTERSECTION_COOLDOWN 1000

// --- PID ---
float kp = 40;  
float ki = 0.002; 
float kd = 8;  

float kp_curve = 50;
float kd_curve = 12;

float setpoit = 0.0;
float read_position = 0.0;

unsigned long last_timer = 0;
float last_error = 0;
float tichphan = 0;

int tocdo_thang = 180;
int tocdo_cua = 140;
int tocdo = tocdo_thang;
int tocdoc_max = 255; 

// --- KHAI BÁO HÀM ---
void dieukhiendongco();
void stop();
void tien(int left, int right);
void phai(int speed); 
void trai(int speed); 
void tinh_vi_tri_line();
float getDistance();
void reset_pid();
void phanh_gap();
void re_phai_90_do();
void re_trai_90_do();
void handle_line_loss();
void update_direction();

// =============================================
// CẬP NHẬT HƯỚNG ĐI CUỐI CÙNG
// =============================================
void update_direction() {
  // Lưu hướng dựa vào vị trí line
  if (read_position < -0.8) {
    last_direction = -1;  // Line lệch trái
  } else if (read_position > 0.8) {
    last_direction = 1;   // Line lệch phải
  } else {
    last_direction = 0;   // Line ở giữa
  }
}

// =============================================
// TÍNH VỊ TRÍ LINE
// =============================================
void tinh_vi_tri_line() {
  float weighted_sum = 0;
  int active_sensors = 0;
  
  if (value[0] == 0) { weighted_sum += (-2); active_sensors++; } 
  if (value[1] == 0) { weighted_sum += (-1); active_sensors++; } 
  if (value[2] == 0) { weighted_sum += (0);  active_sensors++; } 
  if (value[3] == 0) { weighted_sum += (1);  active_sensors++; } 
  if (value[4] == 0) { weighted_sum += (2);  active_sensors++; } 

  if (active_sensors > 0) {
    read_position = weighted_sum / active_sensors;
    update_direction();  // Cập nhật hướng
  }
  // Nếu mất line → giữ nguyên vị trí cuối
}

// =============================================
// PID CONTROLLER
// =============================================
float controller_pid(float setvalue, float readvalue, float kp_use, float ki_use, float kd_use){
  unsigned long now_timer = millis();
  float dt = (now_timer - last_timer) / 1000.0;
  if(dt <= 0) dt = 0.001;

  float error = setvalue - readvalue;
  
  tichphan += error * dt;
  tichphan = constrain(tichphan, -50, 50);
  
  float daoham = (error - last_error) / dt;
  float output = (kp_use * error) + (ki_use * tichphan) + (kd_use * daoham);

  last_error = error;
  last_timer = now_timer;

  return output;
}

void reset_pid() {
  tichphan = 0;
  last_error = 0;
  last_timer = millis();
}

// =============================================
// XỬ LÝ MẤT LINE - DEAD RECKONING
// =============================================
void handle_line_loss() {
  unsigned long lost_duration = millis() - line_lost_time;
  
  // ========================================
  // GIAI ĐOẠN 1: MẤT LINE NGẮN (< 400ms)
  // → CỐI LÀ LINE ĐỨT → ĐI THẲNG
  // ========================================
  if (lost_duration < SHORT_LOSS_TIMEOUT) {
    digitalWrite(led, HIGH);  // LED báo đang mất line
    
    // Đi thẳng với tốc độ vừa phải
    tien(dead_reckoning_speed, dead_reckoning_speed);
    
    Serial.print("Line đứt - Đi thẳng (");
    Serial.print(lost_duration);
    Serial.println("ms)");
    
    return;
  }
  
  // ========================================
  // GIAI ĐOẠN 2: MẤT LINE DÀI (400-1000ms)
  // → BẮT ĐẦU QUAY NHẸ THEO HƯỚNG CŨ
  // ========================================
  else if (lost_duration < LONG_LOSS_TIMEOUT) {
    digitalWrite(led, HIGH);
    
    // Quay nhẹ theo hướng cuối cùng
    if (last_direction < 0) {
      // Line cuối ở trái → Quay nhẹ sang trái
      tien(dead_reckoning_speed - 50, dead_reckoning_speed + 50);
      Serial.println("Quay nhẹ TRÁI tìm line");
    } 
    else if (last_direction > 0) {
      // Line cuối ở phải → Quay nhẹ sang phải
      tien(dead_reckoning_speed + 50, dead_reckoning_speed - 50);
      Serial.println("Quay nhẹ PHẢI tìm line");
    } 
    else {
      // Không biết hướng → Tiếp tục đi thẳng
      tien(dead_reckoning_speed, dead_reckoning_speed);
      Serial.println("Tiếp tục đi thẳng");
    }
    
    return;
  }
  
  // ========================================
  // GIAI ĐOẠN 3: MẤT LINE QUÁ LÂU (> 1000ms)
  // → THẬT SỰ MẤT LINE → QUAY TẠI CHỖ TÌM
  // ========================================
  else {
    Serial.println("MẤT LINE THẬT - Quay tại chỗ");
    
    // Quay mạnh theo hướng cuối
    if (last_direction < 0) {
      re_trai_90_do();
    } else {
      re_phai_90_do();
    }
    
    reset_pid();
  }
}

// =============================================
// SIÊU ÂM
// =============================================
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

// =============================================
// ĐIỀU KHIỂN ĐỘNG CƠ - LOGIC CHÍNH
// =============================================
void dieukhiendongco(){
  tinh_vi_tri_line();
  float distance = getDistance();

  // ========================================
  // 1. ƯU TIÊN VẬT CẢN
  // ========================================
  if (distance > 0 && distance < 25) {
      digitalWrite(led, HIGH);
      stop(); 
      delay(100); 
      phai(180); 
      delay(400);
      tien(160, 160); 
      delay(400);
      reset_pid();
      is_line_lost = false;  // Reset trạng thái mất line
      return; 
  }

  // ========================================
  // 2. KIỂM TRA TRẠNG THÁI LINE
  // ========================================
  bool all_white = (value[0]==1 && value[1]==1 && value[2]==1 && value[3]==1 && value[4]==1);
  
  if (all_white) {
    // ========================================
    // MẤT LINE - BẮT ĐẦU ĐẾM THỜI GIAN
    // ========================================
    if (!is_line_lost) {
      // Lần đầu mất line → ghi thời điểm
      is_line_lost = true;
      line_lost_time = millis();
      Serial.println(">>> BẮT ĐẦU MẤT LINE <<<");
    }
    
    // Xử lý theo chiến lược Dead Reckoning
    handle_line_loss();
    return;
    
  } else {
    // ========================================
    // TÌM LẠI LINE - RESET TRẠNG THÁI
    // ========================================
    if (is_line_lost) {
      unsigned long lost_duration = millis() - line_lost_time;
      Serial.print(">>> TÌM LẠI LINE sau ");
      Serial.print(lost_duration);
      Serial.println("ms <<<");
      
      is_line_lost = false;
      reset_pid();  // Reset PID để tránh giật
    }
    
    digitalWrite(led, LOW);
  }

  // ========================================
  // 3. PHÁT HIỆN NGÃ 3 CHỮ T
  // ========================================
  bool is_T_left = (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[4] == 1);
  bool is_T_right = (value[4] == 0 && value[3] == 0 && value[2] == 0 && value[0] == 1);
  bool is_intersection = is_T_left || is_T_right;
  
  if (is_intersection && (millis() - lastIntersectionTime > INTERSECTION_COOLDOWN)) {
      digitalWrite(led, HIGH);
      lastIntersectionTime = millis();
      dem_ngaba++;
      
      Serial.print("Ngã 3 số: ");
      Serial.println(dem_ngaba);
      
      if (dem_ngaba == 3) {
          stop();
          delay(200);
          
          if (is_T_left) {
              Serial.println("Rẽ TRÁI tại ngã 3 thứ 3");
              re_trai_90_do();
          } else {
              Serial.println("Rẽ PHẢI tại ngã 3 thứ 3");
              re_phai_90_do();
          }
          
          reset_pid();
          is_line_lost = false;  // Reset sau khi rẽ
          
      } else {
          Serial.println("Đi thẳng qua ngã 3");
          tien(160, 160); 
          delay(300);
          reset_pid();
      }
      
      return;
  }

  // ========================================
  // 4. ADAPTIVE PID
  // ========================================
  float kp_use = kp;
  float kd_use = kd;
  
  if (abs(read_position) > 1.2) {
      kp_use = kp_curve;
      kd_use = kd_curve;
      tocdo = tocdo_cua;
  } else {
      tocdo = tocdo_thang;
  }
  
  float output_pid = controller_pid(setpoit, read_position, kp_use, ki, kd_use);
  
  int left = tocdo - output_pid;  
  int right = tocdo + output_pid; 
  left = constrain(left, 0, tocdoc_max);
  right = constrain(right, 0, tocdoc_max);

  // ========================================
  // 5. NGÃ TƯ
  // ========================================
  if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0) {
      tien(tocdo, tocdo);
  }
  
  // ========================================
  // 6. FOLLOW LINE BÌNH THƯỜNG
  // ========================================
  else {
      tien(left, right);
  }
}

// =============================================
// SETUP
// =============================================
void setup(){
  Serial.begin(115200);
  
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
  
  Serial.println("=== XE DÒ LINE - XỬ LÝ LINE ĐỨT ===");
  Serial.println("Sẵn sàng!");
  delay(2000);
}

// =============================================
// LOOP
// =============================================
void loop(){
  for(int i = 0; i < socambien; i++){
    value[i] = digitalRead(cambien[i]);                   
  }
  
  dieukhiendongco();
}

// =============================================
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ
// =============================================
void stop(){
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); 
  digitalWrite(in4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0); 
  ledcWrite(PWM_CHANNEL_B, 0);
}

void tien(int left, int right){
  ledcWrite(PWM_CHANNEL_A, left);
  ledcWrite(PWM_CHANNEL_B, right);
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);
}

void phai(int speed){
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);
}

void trai(int speed){
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);
}

void phanh_gap() {
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  
  digitalWrite(in4, HIGH);
  
  ledcWrite(PWM_CHANNEL_A, 255); 
  ledcWrite(PWM_CHANNEL_B, 255); 
  
  delay(60);
  stop();
  delay(50);
}

void re_phai_90_do() {
  phai(200); 
  delay(60); 
  
  unsigned long startTime = millis();
  while(digitalRead(35) == 1) {
    if (millis() - startTime > 2000) break;
  }
  
  phanh_gap();
}

void re_trai_90_do() {
  trai(200);
  delay(60);
  
  unsigned long startTime = millis();
  while(digitalRead(35) == 1) {
    if (millis() - startTime > 2000) break;
  }
  
  phanh_gap();
}

// =============================================
// HƯỚNG DẪN TINH CHỈNH
// =============================================
/*
TINH CHỈNH CHO LINE ĐỨT:

1. SHORT_LOSS_TIMEOUT (400ms):
   - Thời gian xe đi thẳng khi mất line
   - CÔNG THỨC: timeout = (độ dài khoảng trống × số đoạn) / tốc độ
   - Ví dụ: 3cm × 4 đoạn = 12cm
           Tốc độ 150 (~30cm/s) → timeout = 12/30 = 0.4s = 400ms
   
   - Line đứt DÀI hơn (4cm) → Tăng lên 500ms
   - Line đứt NGẮN hơn (2cm) → Giảm xuống 300ms
   - Nhiều đoạn hơn (6 đoạn) → Tăng lên 600ms

2. LONG_LOSS_TIMEOUT (1000ms):
   - Thời gian bắt đầu quay tìm
   - Thường = SHORT_LOSS_TIMEOUT × 2-3
   - Tăng nếu xe chạy chậm
   - Giảm nếu muốn phản ứng nhanh

3. dead_reckoning_speed (150):
   - Tốc độ khi đi mù qua line đứt
   - Không nên QUÁ NHANH (dễ lạc hướng)
   - Không nên QUÁ CHẬM (mất thời gian)
   - Thường = 70-80% tốc độ bình thường

4. QUAN SÁT SERIAL MONITOR:
   - ">>> BẮT ĐẦU MẤT LINE <<<"
   - "Line đứt - Đi thẳng (XXms)"
   - ">>> TÌM LẠI LINE sau XXms <<<"
   
   Nếu thấy: "TÌM LẠI LINE sau 50ms"
   → Line đứt quá ngắn, không cần xử lý đặc biệt
   
   Nếu thấy: "TÌM LẠI LINE sau 800ms"
   → Timeout đang làm việc tốt!

TEST CASE:
1. Line liên tục → Không có message "MẤT LINE"
2. Line đứt 4 đoạn × 3cm → "Đi thẳng" và "TÌM LẠI"
3. Rẽ ra ngoài line → "MẤT LINE THẬT - Quay tại chỗ"

ƯU ĐIỂM:
✅ Không quay tròn tại chỗ
✅ Tiết kiệm thời gian (>2 giây/lần)
✅ Ổn định hơn
✅ Thích ứng với nhiều loại line đứt

LƯU Ý:
- Xe phải chạy THẲNG trước khi gặp line đứt
- Nếu gặp line đứt khi đang ở CUA → có thể lạc hướng
- Giải pháp: Giảm tốc ở cua (đã có trong adaptive PID)
*/