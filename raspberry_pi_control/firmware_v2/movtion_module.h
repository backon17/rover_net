// switch parts
int switch_pwm_A = 0;
int switch_pwm_B = 0;
bool usePIDCompute = false;
float spd_rate_A = 1.0;
float spd_rate_B = 1.0;
bool heartbeatStopFlag = false;

void movtionPinInit(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcSetup(channel_A, freq, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMA, channel_A);

  ledcSetup(channel_B, freq, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMB, channel_B);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchEmergencyStop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


// ======== PATCH START: 电机安全看门狗 (deadman) ========
// 背景: 主循环 loop() 同时负责 Web 服务/串口/I2C(IMU)。电机电流尖峰会干扰 I2C,
// 导致 loop() 永久卡死: HTTP 失去响应, 官方的心跳停转逻辑 (也在 loop() 里) 不再
// 执行, 电机保持最后的指令狂转 (实测 10~60 秒后才自行恢复)。
// 本补丁新增一个独立于 loop() 的 FreeRTOS 任务 (固定运行在 core 0, loop() 在 core 1):
//   1) deadman: 超过 HEART_BEAT_DELAY (默认3000ms, 可用 {"T":136,"cmd":400} 收紧)
//      没有新的运动指令 -> 立即刹车。loop() 卡死时也照常执行。
//      另外 T:1 指令可带 "ms" 字段 (如 {"T":1,"L":0.2,"R":0.2,"ms":600}):
//      本条指令只跑这 600ms, 到点自动刹车 —— 丢包/PC 崩溃时最多多转 ms 毫秒。
//   2) 卡死自愈: loopAliveMs 超过 15 秒未更新 (loop() 冻结) -> 刹车并重启 ESP32。
//      重启后电机引脚恢复默认关闭状态, WiFi 自动重连。
volatile unsigned long motorLastCmdMs = 0;   // 最近一次运动指令的时刻
volatile unsigned long loopAliveMs  = 0;     // loop() 最近一次运行的时刻
volatile unsigned long motorDeadlineMs = 0; // 本条指令的到期时刻 (0=无期限, 见 T:1 的 "ms" 字段)
volatile bool motorSafetyFired = false;      // 本轮超时是否已刹车(防重复执行)

void motorSafetyFeed() {
  motorLastCmdMs = millis();
  motorSafetyFired = false;
  motorDeadlineMs = 0;  // 新指令到达, 清除旧的到期时刻 (T:1 处理器随后可再设置新值)
}

void motorSafetyTask(void* pvParameters) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(50));
    unsigned long now = millis();
    // 1) 指令 deadman: 超时无新指令 -> 刹车 (不依赖 loop(), 冻结时仍生效)
    //    1a) 心跳超时: 超过 HEART_BEAT_DELAY 没有任何新指令
    //    1b) 指令到期: 本条指令自带 "ms" 时长, 到点自动停 (比心跳更细的保险)
    if (!motorSafetyFired && motorLastCmdMs != 0 &&
        ((now - motorLastCmdMs) > (unsigned long)HEART_BEAT_DELAY ||
         (motorDeadlineMs != 0 && (long)(now - motorDeadlineMs) >= 0))) {
      motorSafetyFired = true;
      // V2: PWM=0 with the direction pins left as-is is a TB6612 *short brake*
      // (same as the normal T:1 L=0,R=0 stop). switchEmergencyStop() pulls
      // IN1=IN2=LOW, which is coast/free-wheel and lets the car slide further.
      ledcWrite(channel_A, 0);
      ledcWrite(channel_B, 0);
    }
    // 2) loop() 冻结检测: 刹车并重启 (注意: 此任务里不能调 Serial, 防止死锁)
    // V2: 30 s (was 15 s). STA connect inside loop() may legally block for
    // connectionTimeout = 15 s, which used to trigger a spurious reboot.
    if (loopAliveMs != 0 && (now - loopAliveMs) > 30000) {
      switchEmergencyStop();
      esp_restart();
    }
  }
}
// ======== PATCH END ========


void switchPortCtrlA(float pwmInputA){
  int pwmIntA = round(pwmInputA * spd_rate_A);
  if(abs(pwmIntA) < 1e-6){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  if(pwmIntA > 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(channel_A, pwmIntA);
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(channel_A,-pwmIntA);
  }
}


void switchPortCtrlB(float pwmInputB){
  int pwmIntB = round(pwmInputB * spd_rate_B);
  if(abs(pwmIntB) < 1e-6){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if(pwmIntB > 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(channel_B, pwmIntB);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B,-pwmIntB);
  }
}


void switchCtrl(int pwmIntA, int pwmIntB) {
    switch_pwm_A = pwmIntA;
    switch_pwm_B = pwmIntB;
    switchPortCtrlA(switch_pwm_A);
    switchPortCtrlB(switch_pwm_B);
}


void lightCtrl(int pwmIn) {
  switch_pwm_A = pwmIn;
  switchPortCtrlA(-abs(switch_pwm_A));
}


void setSpdRate(float inputL, float inputR) {
  inputL = abs(inputL);
  if (inputL > 1) {
    inputL = 1;
  }
  inputR = abs(inputR);
  if (inputR > 1) {
    inputR = 1;
  }
  spd_rate_A = inputL;
  spd_rate_B = inputR;
}


void getSpdRate() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = CMD_GET_SPD_RATE;

  jsonInfoHttp["L"] = spd_rate_A;
  jsonInfoHttp["R"] = spd_rate_B;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}



// movtion parts.
// A-left, B-right

ESP32Encoder encoderA;
ESP32Encoder encoderB;

static unsigned long lastTime = 0;
static unsigned long lastLeftSpdTime = 0;
static unsigned long lastRightSpdTime = 0;
int lastEncoderA = 0;
int lastEncoderB = 0;

double speedGetA;
double speedGetB;

double plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;


void initEncoders() {
  // if(SET_MOTOR_DIR){
  //   encoderA.attachHalfQuad(AENCB, AENCA);
  //   encoderB.attachHalfQuad(BENCB, BENCA);
  // }else{
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  // }
  encoderA.setCount(0);
  encoderB.setCount(0);
}

void getWheelSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesA = encoderA.getCount();
  long encoderPulsesB = encoderB.getCount();

  if (!SET_MOTOR_DIR) {
    speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / ((double)(currentTime - lastTime) / 1000000);
    speedGetB = (plusesRate * (encoderPulsesB - lastEncoderB)) / ((double)(currentTime - lastTime) / 1000000);
  } else {
    speedGetA = (plusesRate * (lastEncoderA - encoderPulsesA)) / ((double)(currentTime - lastTime) / 1000000);
    speedGetB = (plusesRate * (lastEncoderB - encoderPulsesB)) / ((double)(currentTime - lastTime) / 1000000);
  }
  lastEncoderA = encoderPulsesA;
  lastEncoderB = encoderPulsesB;
  lastTime = currentTime;
}

void getLeftSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesA = encoderA.getCount();
  if (!SET_MOTOR_DIR) {
    speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
  } else {
    speedGetA = (plusesRate * (lastEncoderA - encoderPulsesA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
  }
  lastEncoderA = encoderPulsesA;
  lastLeftSpdTime = currentTime;
}

void getRightSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesB = encoderB.getCount();
  if (!SET_MOTOR_DIR) {
    speedGetB = (plusesRate * (encoderPulsesB - lastEncoderB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
  } else {
    speedGetB = (plusesRate * (lastEncoderB - encoderPulsesB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
  }
  lastEncoderB = encoderPulsesB;
  lastRightSpdTime = currentTime;
}





// --- PID Controller ---

PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
PID_v2 pidB(__kp, __ki, __kd, PID::Direct);

double outputA = 0;
double outputB = 0;
double setpointA = 0;
double setpointB = 0;

int setpoint_interval = 200;
unsigned long setpoint_cmd_recv = millis();
unsigned long setpoint_last_time = millis();
float setpointA_buffer;
float setpointB_buffer;
float setpointA_last;
float setpointB_last;
float change_offset = 0.005;
bool new_setpoint_flag = false;

void pidControllerInit() {
  pidA.Start(speedGetA,
             outputA,
             setpointA);
  pidA.SetOutputLimits(-255, 255);
  pidA.SetMode(PID::Automatic);

  pidB.Start(speedGetB,
             outputB,
             setpointB);
  pidB.SetOutputLimits(-255, 255);
  pidB.SetMode(PID::Automatic);
}

void leftCtrl(float pwmInputA){
  int pwmIntA = round(pwmInputA);
  if (mainType != 3) {
    speedGetA = pwmIntA;
  }
  if(SET_MOTOR_DIR){
    if(pwmIntA < 0){
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(channel_A, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(channel_A, abs(pwmIntA));
    }
  }else{
    if(pwmIntA < 0){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(channel_A, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(channel_A, abs(pwmIntA));
    }
  }
}

void rightCtrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);
  if (mainType != 3) {
    speedGetB = pwmIntB;
  }
  if(SET_MOTOR_DIR){
    if(pwmIntB < 0){
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(channel_B, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(channel_B, abs(pwmIntB));
    }
  }else{
    if(pwmIntB < 0){
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(channel_B, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(channel_B, abs(pwmIntB));
    }
  }
}

void setGoalSpeed(float inputLeft, float inputRight) {
  // setpoint_cmd_recv = millis();
  motorSafetyFeed();  // PATCH: 任何设定速度的指令都给安全看门狗喂狗 (T:1 / T:13 / 心跳停转)
  if (mainType == 3) {
    usePIDCompute = true;

    if(inputLeft < -2.0 || inputLeft > 2.0){
      return;
    }

    if(inputRight < -2.0 || inputRight > 2.0){
      return;
    }
    
    setpointA = inputLeft*spd_rate_A;
    setpointB = inputRight*spd_rate_B;

    if (setpointA != setpointA_buffer) {
      pidA.Setpoint(setpointA);
      setpointA_buffer = inputLeft;
    }
    
    if (setpointB != setpointB_buffer) {
      pidB.Setpoint(setpointB);
      setpointB_buffer = inputRight;
    }
  } else {
    usePIDCompute = false;
    leftCtrl(inputLeft * 512 * spd_rate_A);
    rightCtrl(inputRight * 512 * spd_rate_B);
  }
}

void pidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputA = pidA.Run(speedGetA);
  if (abs(outputA)<THRESHOLD_PWM) {
    outputA = 0;
  }
  if (setpointA == 0 && speedGetA == 0) {
    outputA = 0;
  }
  leftCtrl(outputA);

  outputB = pidB.Run(speedGetB);
  if (abs(outputB)<THRESHOLD_PWM) {
    outputB = 0;
  }
  if (setpointB == 0 && speedGetB == 0) {
    outputB = 0;
  }
  rightCtrl(outputB);
}

void LeftPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputA = pidA.Run(speedGetA);
  if (abs(outputA)<THRESHOLD_PWM) {
    outputA = 0;
  }
  if (setpointA == 0 && speedGetA == 0) {
    outputA = 0;
  }
  leftCtrl(outputA);
}

void RightPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputB = pidB.Run(speedGetB);
  if (abs(outputB)<THRESHOLD_PWM) {
    outputB = 0;
  }
  if (setpointB == 0 && speedGetB == 0) {
    outputB = 0;
  }
  rightCtrl(outputB);
}

void setPID(float inputP, float inputI, float inputD, float inputLimits) {
  __kp = inputP;
  __ki = inputI;
  __kd = inputD;
  windup_limits = inputLimits;
  pidA.SetTunings(__kp, __ki, __kd);
  pidB.SetTunings(__kp, __ki, __kd);
}

void rosCtrl(float rosX, float rosZ) {
  setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);
  setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);
  setGoalSpeed(setpointA, setpointB);
}

void heartBeatCtrl() {
  // V2 FIX: signed difference. currentTimeMillis is sampled early in loop(); a
  // command time stamped later in the same loop (e.g. by turnCtrlTick) made the
  // unsigned subtraction wrap to ~4e9 and stop the motors on every iteration.
  if ((long)(currentTimeMillis - lastCmdRecvTime) > (long)HEART_BEAT_DELAY) {
    if (!heartbeatStopFlag) {
      heartbeatStopFlag = true;
      setGoalSpeed(0, 0);
      // leftCtrl(0);
      // rightCtrl(0);
    }
  }
}

void changeHeartBeatDelay(int inputCmd) {
  HEART_BEAT_DELAY = inputCmd;
}

void mm_settings(byte inputMain, byte inputModule) {
  mainType = inputMain;
  moduleType = inputModule;

  if (mainType == 1) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 2100;
    TRACK_WIDTH = 0.125;
    SET_MOTOR_DIR = false; // checked
    usePIDCompute = false;
  } else if (mainType == 2) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 1650;
    TRACK_WIDTH = 0.172;
    SET_MOTOR_DIR = true; // checked
    usePIDCompute = false;
  } else if (mainType == 3) {
    WHEEL_D = 0.0523;
    ONE_CIRCLE_PLUSES = 1092;
    TRACK_WIDTH = 0.141;
    SET_MOTOR_DIR = true; // checked
    usePIDCompute = true;
  }
  plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;
  // initEncoders();

  if (mainType == 1) {
    screenLine_2 = "RaspRover";
  } else if (mainType == 2) {
    screenLine_2 = "UGV02";
  } else if (mainType == 3) {
    screenLine_2 = "UGV01";
  } 

  if (moduleType == 0) {
    screenLine_2 += " Null";
  } else if (moduleType == 1) {
    screenLine_2 += " Arm";
  } else if (moduleType == 2) {
    screenLine_2 += " PT";
  } 
}