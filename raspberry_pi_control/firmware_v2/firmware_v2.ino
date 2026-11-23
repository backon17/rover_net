#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<768> jsonInfoHttp;

// TaskHandle_t Pid_ctrl;

#include <SCServo.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <Adafruit_SSD1306.h>
#include <INA219_WE.h>
#include <ESP32Encoder.h>
#include <PID_v2.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>


// functions for barrery info.
#include "battery_ctrl.h"

// functions for oled.
#include "oled_ctrl.h"

// config for ugv.
#include "ugv_config.h"

// functions for the leds of UGV.
#include "ugv_led_ctrl.h"

// functions for RoArm-M2 ctrl.
#include "RoArm-M2_module.h"

// functions for gimbal ctrl.
#include "gimbal_module.h"

// define json cmd.
#include "json_cmd.h"

// functions for IMU ctrl.
#include "IMU_ctrl.h"

// functions for movtion ctrl. 
#include "movtion_module.h"

// V2: on-board gyro heading + closed-loop turn controller.
#include "turn_ctrl.h"

// functions for editing the files in flash.
#include "files_ctrl.h"

// advance functions for ugv ctrl.
#include "ugv_advance.h"

// functions for wifi ctrl.
#include "wifi_ctrl.h"

// functions for esp-now.
#include "esp_now_ctrl.h"

// functions for uart json ctrl.
#include "uart_ctrl.h"

// functions for http & web server.
#include "http_server.h"


void moduleType_RoArmM2() {
  unsigned long curr_time = millis();
  if (curr_time - prev_time >= 10){
    constantHandle();
    prev_time = curr_time;
  }

  RoArmM2_getPosByServoFeedback();
  
  // esp-now flow ctrl as a flow-leader.
  switch(espNowMode) {
  case 1: espNowGroupDevsFlowCtrl();break;
  case 2: espNowSingleDevFlowCtrl();break;
  }

  if (InfoPrint == 2) {
    RoArmM2_infoFeedback();
  }
}


void moduleType_Gimbal() {
  getGimbalFeedback();
  gimbalSteady(steadyGoalY);
}


void setup() {
  Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  Wire.setTimeOut(10);  // PATCH: I2C 事务 10ms 超时（默认 50ms），降低卡死概率
  while(!Serial) {}

  ina219_init();
  inaDataUpdate();

  // set mainType & moduleType.
  // mainType: 1.WAVE ROVER, 2.UGV02, 3.UGV01
  // moduleType: 0.Null, 1.RoArm, 2.PT
  mm_settings(mainType, moduleType);

  init_oled();
  if (mainType == 1) {
    screenLine_0 = "WAVE ROVER";
  } else if (mainType == 2) {
    screenLine_0 = "UGV";
  } else if (mainType == 3) {
    screenLine_0 = "UGV";
  } 
  
  screenLine_1 = "version: 0.90";
  screenLine_2 = "starting...";
  screenLine_3 = "";
  oled_update();

  delay(1200);

  // functions for IMU.
  imu_init();

  // functions for the leds on ugv.
  led_pin_init();

  // init the littleFS funcs in files_ctrl.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize LittleFS";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
  initFS();

  // init the funcs in switch_module.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize 12V-switch ctrl";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
  movtionPinInit();

  // servos power up
  screenLine_2 = screenLine_3;
  screenLine_3 = "Power up the servos";
  oled_update();
  if(InfoPrint == 1){Serial.println("Power up the servos.");}
  delay(500);
  
  // init servo ctrl functions.
  screenLine_2 = screenLine_3;
  screenLine_3 = "ServoCtrl init UART2TTL...";
  oled_update();
  if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
  RoArmM2_servoInit();

  // check the status of the servos.
  screenLine_2 = screenLine_3;
  screenLine_3 = "Bus servos status check...";
  oled_update();
  if(InfoPrint == 1){Serial.println("Bus servos status check...");}
  RoArmM2_initCheck(false);

  if(InfoPrint == 1 && RoArmM2_initCheckSucceed){
    Serial.println("All bus servos status checked.");
  }
  if(RoArmM2_initCheckSucceed) {
    screenLine_2 = "Bus servos: succeed";
  } else {
    screenLine_2 = "Bus servos: " + 
    servoFeedback[BASE_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
    servoFeedback[ELBOW_SERVO_ID - 11].status +
    servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }
  screenLine_3 = ">>> Moving to init pos...";
  oled_update();
  RoArmM2_resetPID();
  RoArmM2_moveInit();

  screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
  oled_update();
  if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

  screenLine_3 = "WiFi init";
  oled_update();
  if(InfoPrint == 1){Serial.println("WiFi init.");}
  initWifi();
  // V2.3: no WiFi modem sleep. Power save delays incoming packets to the next
  // DTIM beacon (100-300 ms), which made PC control loops stutter.
  WiFi.setSleep(false);

  screenLine_3 = "http & web init";
  oled_update();
  if(InfoPrint == 1){Serial.println("http & web init.");}
  initHttpWebServer();

  screenLine_3 = "ESP-NOW init";
  oled_update();
  if(InfoPrint == 1){Serial.println("ESP-NOW init.");}
  initEspNow();

  screenLine_3 = "UGV started";
  oled_update();
  if(InfoPrint == 1){Serial.println("UGV started.");}

  getThisDevMacAddress();

  updateOledWifiInfo();

  initEncoders();

  pidControllerInit();

  screenLine_2 = String("MAC:") + macToString(thisDevMac);
  oled_update();

  led_pwm_ctrl(0, 0);

  if(InfoPrint == 1){Serial.println("Application initialization settings.");}
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);

  // ======== PATCH START: 启动电机安全看门狗任务 ========
  // 独立于 loop() 的 FreeRTOS 任务 (见 movtion_module.h):
  //   1) 超过 HEART_BEAT_DELAY 无新运动指令 -> 自动刹车 (loop() 卡死时也生效)
  //   2) loop() 冻结超过 15 秒 -> 刹车并重启 ESP32 自愈
  motorSafetyFeed();
  loopAliveMs = millis();
  xTaskCreatePinnedToCore(motorSafetyTask, "motorSafety", 2048, NULL, 1, NULL, 0);
  // ======== PATCH END ========

  // V2: measure the gyro bias once at boot (the car is normally still here).
  gyroHead.startCalib(1.0f, true);
}


void loop() {
  loopAliveMs = millis();  // PATCH: 喂主循环存活标记 (电机安全看门狗任务检查此值)
  serialCtrl();
  server.handleClient();

  // read and compute the info of joints.
  switch (moduleType) {
  case 1: moduleType_RoArmM2();break;
  case 2: moduleType_Gimbal();break;
  }

  // recv esp-now json cmd.
  if(runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }

  getLeftSpeed();

  LeftPidControllerCompute();
  
  getRightSpeed();
  
  RightPidControllerCompute();
  
  oledInfoUpdate();

  updateIMUData();

  // V2: gyro heading integration + on-board turn control at loop rate.
  gyroHeadingTick();
  turnCtrlTick();
  straightCtrlTick();
  loopTimingTick();

  if (baseFeedbackFlow) {
    baseInfoFeedback();
  }

  heartBeatCtrl();

  size_t freeHeap = esp_get_free_heap_size();
}