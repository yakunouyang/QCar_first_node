#include <Servo.h>
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断

// 使用前需在库管理器中下载tinyproto
#include <TinyProtocol.h>

/////////TB6612驱动引脚////
#define AIN1 11
#define AIN2 5
#define BIN1 6
#define BIN2 3
#define SERVO 9
/////////编码器引脚////////
#define ENCODER_L 8  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 4
#define ENCODER_R 7
#define DIRECTION_R 2

#define ENCODER_K 2.55 // p / 39.25 (cm) * 100 / 10ms * 100
/////////按键引脚////////
#define KEY 18

#define TRACK_WIDTH 0.156f
#define WHEEL_BASE 0.1445f

// ============ MSG DEFINATION =============
// 定义车辆ID
#define MSG_SELF 0

#define MSG_TYPE_TWIST 0xe1
#define MSG_TYPE_PID 0xe2

struct TwistMessage {
  int16_t velocity;
  int16_t angle;
};

struct PIDMessage {
  float kp;
  float ki;
};

volatile long pulseL_, pulseR_;
int velocityLeft_, velocityRight_, targetVelocity_ = 0; // cm/s
int leftPWM_, rightPWM_;
int targetAngle_ = 0;
int voltage_;
float velocityKp_ = 10.0, velocityKi_ =  1.0;
bool racing_ = true;

Tiny::ProtoLight proto;
Tiny::Packet<32> g_packet;
Servo servo;

void setup() {
  pinMode(AIN1, OUTPUT); //电机控制引脚
  pinMode(AIN2, OUTPUT); //电机控制引脚
  pinMode(BIN1, OUTPUT); //电机速度控制引脚
  pinMode(BIN2, OUTPUT); //电机速度控制引脚
  servo.attach(SERVO);

  pinMode(ENCODER_L, INPUT); //编码器引脚
  pinMode(DIRECTION_L, INPUT); //编码器引脚
  pinMode(ENCODER_R, INPUT); //编码器引脚
  pinMode(DIRECTION_R, INPUT); //编码器引脚
  pinMode(KEY, INPUT); //按键引脚
  pinMode(LED_BUILTIN, OUTPUT);
  delay(200); //延时等待初始化完成

  attachInterrupt(0, readEncoderR, CHANGE);           //开启外部中断 编码器接口1
  attachPinChangeInterrupt(4, readEncoderL, CHANGE);  //开启外部中断 编码器接口2

  //使用Timer2设置10ms定时中断
  MsTimer2::set(10, timer);
  MsTimer2::start();

  Serial.setTimeout(0);
  Serial.begin(57600);
  proto.enableCheckSum();
  proto.beginToSerial();
}

void loop() {
  digitalWrite(LED_BUILTIN, racing_ ? HIGH : LOW);

  if (racing_) recv();
}

void recv() {
  int len = proto.read(g_packet);

  if (len > 1) {
    byte* data = g_packet.data();
    if (data[0] != MSG_SELF) return;
    handleMessage(data[1], data + 2, len - 2);
  }
}

void handleMessage(byte type, byte *data, int len) {
  if (type == MSG_TYPE_TWIST) {
    if (len != sizeof(TwistMessage)) return;
    TwistMessage *ptr = (TwistMessage *)data;
    targetVelocity_ = min(max(ptr->velocity, -100), 100);
    targetAngle_ = min(max(ptr->angle, -40), 40);
  } else if (type == MSG_TYPE_PID) {
    if (len != sizeof(PIDMessage)) return;
    PIDMessage *pid = (PIDMessage *)data;
    velocityKp_ = pid->kp;
    velocityKi_ = pid->ki;
  }
}

void timer() {
  sei(); //全局中断开启

  measureVelocity();

  if (racing_) {
    control();
  } else {
    standby();
  }
  if (doesClick()) {
    racing_ = !racing_;
  }
}

void standby() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  servo.write(95);
}

char doesClick() {
  static byte flagKey = 1;
  if (flagKey && digitalRead(KEY) == 0) {
    flagKey = 0;
    if (digitalRead(KEY) == 0)  return 1;
  }
  else if (digitalRead(KEY) == 1) flagKey = 1;
  return 0;
}

void control() {
  int targetLeft, targetRight;
  kinematicAnalysis(&targetLeft, &targetRight);

  leftPWM_ = incrementalPI(velocityLeft_, targetLeft, 0);
  rightPWM_ = incrementalPI(velocityRight_, targetRight, 1);

  setPWM(leftPWM_, rightPWM_);
  servo.write(95 + 1.61437 * targetAngle_ - 0.86936);
}

void measureVelocity() {
  velocityLeft_ = pulseL_ * ENCODER_K;  pulseL_ = 0; //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  velocityRight_ = pulseR_ * ENCODER_K; pulseR_ = 0; //读取右轮编码器数据，并清零
}

/**
   函数功能：小车运动数学模型
**/
void kinematicAnalysis(int *targetA, int *targetB) {
  //后轮差速
  *targetA = targetVelocity_ * (1 + TRACK_WIDTH * tan(targetAngle_ * PI / 180) / 2 / WHEEL_BASE);
  *targetB = targetVelocity_ * (1 - TRACK_WIDTH * tan(targetAngle_ * PI / 180) / 2 / WHEEL_BASE);
}

/**
   增量PI控制器
   pwm += Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
   e(k) 代表本次偏差
   e(k-1) 代表上一次的偏差  以此类推
   pwm代表增量输出
   在我们的速度控制闭环系统里面，只使用PI控制
   pwm += Kp[e(k)-e(k-1)]+Ki*e(k)
**/
int incrementalPI(int current, int target, char index)
{
  static int output[2] = { 0 };
  static float lastError[2] = { 0.0 };
  float error = target - current;
  output[index] += velocityKp_ * (error - lastError[index]) + velocityKi_ * error;
  output[index] = min(max(output[index], -250), 250);
  lastError[index] = error;
  return output[index];
}

void setPWM(int motorA, int motorB) {
  if (motorA > 0) {
    analogWrite(AIN2, motorA);
    digitalWrite(AIN1, LOW);
  }
  else {
    digitalWrite(AIN1, HIGH);
    analogWrite(AIN2, 255 + motorA);
  }

  if (motorB > 0) {
    digitalWrite(BIN2, LOW);
    analogWrite(BIN1, motorB);
  }
  else {
    analogWrite(BIN1, 255 + motorB);
    digitalWrite(BIN2, HIGH);
  }
}

/**
   外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
 **/
void readEncoderL() {
  if (digitalRead(ENCODER_L) == LOW) {
    //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW) pulseL_--;  //根据另外一相电平判定方向
    else pulseL_++;
  }
  else {
    //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW) pulseL_++; //根据另外一相电平判定方向
    else pulseL_--;
  }
}

/**
   外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
 **/
void readEncoderR() {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW) pulseR_++; //根据另外一相电平判定方向
    else pulseR_--;
  }
  else { //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW) pulseR_--; //根据另外一相电平判定方向
    else pulseR_++;
  }
}