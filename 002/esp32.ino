
#define in1 18
#define in2 19
#define in3 21
#define in4 22
#define led 19

#define freq 5000
#define reslution 8

#define ENA 16
#define ENB 23


const int socambien = 5; 
int cambien[socambien] = {25, 34, 35, 32, 33};  
int value[socambien];   

void dieukhiendongco();
void stop();
void tien(int left, int right);
void phai(int left, int right);
void trai(int left, int right);

float kp = 0.001;
float ki = 0.1;
float kd = 0.01;

float setpoit = 0.0;
float read_last_value = 0.0;

unsigned long last_timer = 0;
float last_error = 0;
float tichphan = 0;

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

int tocdo = 145;
int tocdoc_max = 180;

void dieukhiendongco(){
  float output_pid = controller_pid(setpoit, read_last_value, kp, ki, kd);
  int left = tocdo - output_pid;
  int right = tocdo + output_pid;

  left = constrain(left, 0, tocdoc_max);
  right = constrain(right, 0, tocdoc_max);

  if((value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0) || (value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1)){
    digitalWrite(led, HIGH);
    stop();
  }
  else if(value[0] == 1 || value[1] == 1){
    digitalWrite(led, HIGH);
    trai(left, right);
  }
  else if(value[3] == 1 || value[4] == 1){
    digitalWrite(led, HIGH);
    phai(left, right);
  }
  else if(value[2] == 1){
    digitalWrite(led, HIGH);
    tien(left, right);
  }
  else{
    digitalWrite(led, LOW);
    stop();
  }
}

void setup(){
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(led, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  stop();
  ledcAttach(ENA, freq, reslution);
  ledcAttach(ENB, freq, reslution);

  for(int i = 0; i < socambien; i++){
    pinMode(cambien[i], INPUT);
  }

  Serial.println("Doc du lieu tu cam bien:");
}

void loop(){
  for(int i = 0; i < socambien; i++){
    value[i] = digitalRead(cambien[i]);                   
  }

  for(int i = 0; i < socambien; i++){
    Serial.print(value[i]);
    Serial.print("   |   ");
  }
  Serial.println();

  dieukhiendongco();
}

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void tien(int left, int right){
  ledcWrite(ENA, left);
  ledcWrite(ENB, right);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void phai(int left, int right){
  ledcWrite(ENA, left);
  ledcWrite(ENB, right);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void trai(int left, int right){
  ledcWrite(ENA, left);
  ledcWrite(ENB, right);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
