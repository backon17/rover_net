// turn_ctrl.h -- firmware glue for turn_logic.h (V2).
//
// New JSON commands:
//   {"T":160,"deg":90,"spd":0.30,"tmo":8}  start a closed-loop in-place turn
//                                          (deg: left positive; spd: cruise PWM;
//                                           tmo: optional timeout in seconds)
//   {"T":161}                              cancel the turn (short brake)
//   {"T":162,"ms":1000,"zero":1}           stationary gyro bias calibration
//                                          (motors are braked; zero heading after)
//   {"T":163,...}                          set turn params (see turnSetParams)
//   {"T":164}                              status frame {"T":1010,...}
//   {"T":165,"ms":5500,"spd":0.16,"hd":0,"ramp":500}
//                                          V2.3: straight drive for ms with
//                                          on-board gyro heading hold (hd: heading
//                                          to hold, default = current; spd: signed
//                                          cruise PWM; ramp: soft-start ms)
//                                          V2.4: stops with ss=5 when the battery
//                                          current shows the car is blocked
//   {"T":1,...,"fb":1}                     T:1 that also returns the status frame
// Any T:1 / T:11 / T:13 motion command cancels a running turn.
#pragma once
#include "turn_logic.h"
#include <esp_ota_ops.h>

#define FW_VERSION "v2.4.3-sync"

#define CMD_TURN_START       160
#define CMD_TURN_STOP        161
#define CMD_GYRO_CALIB       162
#define CMD_TURN_PARAMS      163
#define CMD_TURN_STATUS      164
#define CMD_STRAIGHT_START   165
#define FEEDBACK_TURN_STATUS 1010

GyroHeading gyroHead;
TurnCtrl turnCtrl;
StraightCtrl straightCtrl;

unsigned long gyroLastUs = 0;
unsigned long loopLastUs = 0;
unsigned long loopWinStartMs = 0;
float loopMaxMsWin = 0.0f;   // max loop period in the current 1 s window
float loopMaxMs = 0.0f;      // max loop period in the previous 1 s window

// Call once per loop(), right after updateIMUData().
void gyroHeadingTick() {
  unsigned long nowUs = micros();
  float dt = 0.0f;
  if (gyroLastUs != 0) dt = (float)(nowUs - gyroLastUs) * 1e-6f;
  gyroLastUs = nowUs;
  gyroHead.update((float)gz, dt);
}

// Loop timing telemetry: lets us see whether loop() really stalls (I2C/HTTP).
void loopTimingTick() {
  unsigned long nowUs = micros();
  if (loopLastUs != 0) {
    float ms = (float)(nowUs - loopLastUs) * 1e-3f;
    if (ms > loopMaxMsWin) loopMaxMsWin = ms;
  }
  loopLastUs = nowUs;
  unsigned long nowMs = millis();
  if (nowMs - loopWinStartMs >= 1000) {
    loopMaxMs = loopMaxMsWin;
    loopMaxMsWin = 0.0f;
    loopWinStartMs = nowMs;
  }
}

#define TURN_MAX_GAP_MS      1000   // loop gap after which the turn is abandoned
unsigned long turnLastTickMs = 0;

// Call once per loop(), after gyroHeadingTick().
void turnCtrlTick() {
  if (!turnCtrl.running()) return;
  unsigned long nowMs = millis();
  // A loop() freeze longer than ~1 s makes the gyro integration across the gap
  // unreliable: give up. Shorter stalls are tolerated: the motors keep the last
  // command (steady rate -> trapezoidal integration across the gap stays
  // accurate); the 400 ms heartbeat deadman is the backstop. A short motor
  // lease was tried in simulation and made things worse: braking inside the
  // gap is invisible to the gyro integration.
  if (turnLastTickMs != 0 && nowMs - turnLastTickMs > TURN_MAX_GAP_MS) {
    turnCtrl.abort();
    setGoalSpeed(0, 0);
    return;
  }
  turnLastTickMs = nowMs;
  turnCtrl.step(nowMs * 1e-3, gyroHead.heading, gyroHead.rate);
  // The controller owns the motors while running: keep heartbeat/deadman fed.
  heartbeatStopFlag = false;
  lastCmdRecvTime = nowMs;      // heartBeatCtrl() compares with a signed diff (V2 fix)
  setGoalSpeed(turnCtrl.outL, turnCtrl.outR);
}

unsigned long straightLastTickMs = 0;
#define STRAIGHT_CURRENT_PERIOD_MS 25   // INA219 read period while driving straight
unsigned long straightCurLastMs = 0;

// Call once per loop(), after turnCtrlTick(). Same freeze rule as the turn.
void straightCtrlTick() {
  if (!straightCtrl.running()) return;
  unsigned long nowMs = millis();
  if (straightLastTickMs != 0 && nowMs - straightLastTickMs > TURN_MAX_GAP_MS) {
    straightCtrl.abort();
    setGoalSpeed(0, 0);
    return;
  }
  straightLastTickMs = nowMs;
  // V2.4: the stock firmware reads the INA219 only every 10 s (OLED refresh);
  // stall detection needs the battery current at ~40 Hz while driving.
  if (nowMs - straightCurLastMs >= STRAIGHT_CURRENT_PERIOD_MS) {
    straightCurLastMs = nowMs;
    current_mA = ina219.getCurrent_mA();
    straightCtrl.feedCurrent(nowMs * 1e-3, current_mA);
  }
  straightCtrl.step(nowMs * 1e-3, gyroHead.heading, gyroHead.rate);
  heartbeatStopFlag = false;
  lastCmdRecvTime = nowMs;
  setGoalSpeed(straightCtrl.outL, straightCtrl.outR);
}

void turnCancelByMotionCmd() {
  if (turnCtrl.running()) turnCtrl.abort();
  if (straightCtrl.running()) straightCtrl.abort();
}

void turnStatusFeedback() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_TURN_STATUS;
  jsonInfoHttp["gy"] = round(gyroHead.heading * 100.0) / 100.0;
  jsonInfoHttp["gr"] = round(gyroHead.rate * 10.0f) / 10.0f;
  jsonInfoHttp["ts"] = turnCtrl.state;
  jsonInfoHttp["tp"] = turnCtrl.phase;
  jsonInfoHttp["te"] = round(turnCtrl.lastErr * 10.0f) / 10.0f;
  jsonInfoHttp["tn"] = turnCtrl.segs;
  jsonInfoHttp["tk"] = turnCtrl.rekicks;
  jsonInfoHttp["tb"] = round(turnCtrl.boostPeak * 1000.0f) / 1000.0f;  // v2.4.3 rate-floor boost
  jsonInfoHttp["ck"] = round(turnCtrl.p.coastK * 1000.0f) / 1000.0f;
  jsonInfoHttp["gb"] = round(gyroHead.bias * 1000.0f) / 1000.0f;
  jsonInfoHttp["gn"] = round(gyroHead.noise * 1000.0f) / 1000.0f;
  jsonInfoHttp["ss"] = straightCtrl.state;
  jsonInfoHttp["sp"] = round(straightCtrl.progress(millis() * 1e-3) * 1000.0f) / 1000.0f;
  jsonInfoHttp["se"] = round(straightCtrl.lastErr * 10.0f) / 10.0f;
  jsonInfoHttp["sm"] = round(straightCtrl.maxAbsErr * 10.0f) / 10.0f;
  jsonInfoHttp["sa"] = round(straightCtrl.meanAbsErr() * 100.0f) / 100.0f;
  // V2.4: current (mA, filtered / baseline / peak) and remembered motor trim
  jsonInfoHttp["si"] = (int)round(straightCtrl.iFilt);
  jsonInfoHttp["sib"] = (int)round(straightCtrl.baseMa);
  jsonInfoHttp["sip"] = (int)round(straightCtrl.peakMa);
  jsonInfoHttp["stf"] = round(straightCtrl.iMem[0] * 1000.0f) / 1000.0f;
  jsonInfoHttp["stb"] = round(straightCtrl.iMem[1] * 1000.0f) / 1000.0f;
  jsonInfoHttp["cal"] = gyroHead.calibrating ? 1 : 0;
  jsonInfoHttp["lp"] = round(loopMaxMs * 10.0f) / 10.0f;
  jsonInfoHttp["v"] = loadVoltage_V;
  jsonInfoHttp["fw"] = FW_VERSION;
  jsonInfoHttp["bd"] = __DATE__ " " __TIME__;
  const esp_partition_t *running = esp_ota_get_running_partition();
  jsonInfoHttp["part"] = running ? running->label : "?";
  // V2.4.2: identify the car independently of its (DHCP) IP
  jsonInfoHttp["mac"] = thisMacStr.c_str();
  String out;
  serializeJson(jsonInfoHttp, out);
  Serial.println(out);
}

void turnStart(float deg, float spd, float tmo) {
  if (gyroHead.calibrating) {
    turnStatusFeedback();
    return;
  }
  straightCtrl.abort();
  turnCtrl.start(millis() * 1e-3, gyroHead.heading, deg, spd, tmo);
  turnLastTickMs = millis();
  motorSafetyFeed();
  turnStatusFeedback();
}

void turnStop() {
  turnCtrl.abort();
  straightCtrl.abort();
  setGoalSpeed(0, 0);
  turnStatusFeedback();
}

void gyroCalibStart(int ms, bool zero) {
  turnCtrl.abort();
  straightCtrl.abort();
  setGoalSpeed(0, 0);
  if (ms < 200) ms = 200;
  if (ms > 10000) ms = 10000;
  gyroHead.startCalib(ms * 1e-3f, zero);
  turnStatusFeedback();
}

// {"T":163,"kick":0.4,"kms":250,"minp":0.22,"corp":0.24,"slow":40,"tol":2,
//  "ck":0.1,"cc":0.5,"corr":3,"gs":1.0,"gsign":1}
void turnSetParams() {
  TurnParams &p = turnCtrl.p;
  if (jsonCmdReceive.containsKey("kick")) p.kick = jsonCmdReceive["kick"];
  if (jsonCmdReceive.containsKey("kms")) p.kickMs = jsonCmdReceive["kms"];
  if (jsonCmdReceive.containsKey("minp")) p.minPwm = jsonCmdReceive["minp"];
  if (jsonCmdReceive.containsKey("trc")) p.rateCruise = jsonCmdReceive["trc"];
  if (jsonCmdReceive.containsKey("trm")) p.rateMin = jsonCmdReceive["trm"];
  if (jsonCmdReceive.containsKey("bk")) p.boostK = jsonCmdReceive["bk"];
  if (jsonCmdReceive.containsKey("bmax")) p.boostMax = jsonCmdReceive["bmax"];
  if (jsonCmdReceive.containsKey("corp")) p.corrPwm = jsonCmdReceive["corp"];
  if (jsonCmdReceive.containsKey("slow")) p.slowDeg = jsonCmdReceive["slow"];
  if (jsonCmdReceive.containsKey("tol")) p.tol = jsonCmdReceive["tol"];
  if (jsonCmdReceive.containsKey("ck")) p.coastK = jsonCmdReceive["ck"];
  if (jsonCmdReceive.containsKey("cc")) p.coastC = jsonCmdReceive["cc"];
  if (jsonCmdReceive.containsKey("corr")) p.maxCorr = jsonCmdReceive["corr"];
  if (jsonCmdReceive.containsKey("gs")) gyroHead.scale = jsonCmdReceive["gs"];
  if (jsonCmdReceive.containsKey("gsign")) gyroHead.sign = jsonCmdReceive["gsign"];
  StraightParams &q = straightCtrl.p;
  if (jsonCmdReceive.containsKey("skp")) q.kp = jsonCmdReceive["skp"];
  if (jsonCmdReceive.containsKey("ski")) q.ki = jsonCmdReceive["ski"];
  if (jsonCmdReceive.containsKey("skd")) q.kd = jsonCmdReceive["skd"];
  if (jsonCmdReceive.containsKey("scf")) q.corrFrac = jsonCmdReceive["scf"];
  if (jsonCmdReceive.containsKey("sr0")) q.ramp0 = jsonCmdReceive["sr0"];
  // V2.4: integral carry-over / motor trim and stall detection
  if (jsonCmdReceive.containsKey("skeep")) q.keepI = (int)jsonCmdReceive["skeep"] != 0;
  if (jsonCmdReceive.containsKey("stf")) straightCtrl.iMem[0] = jsonCmdReceive["stf"];
  if (jsonCmdReceive.containsKey("stb")) straightCtrl.iMem[1] = jsonCmdReceive["stb"];
  if (jsonCmdReceive.containsKey("sst")) q.stallOn = (int)jsonCmdReceive["sst"] != 0;
  if (jsonCmdReceive.containsKey("ssr")) q.stallRatio = jsonCmdReceive["ssr"];
  if (jsonCmdReceive.containsKey("ssa")) q.stallAddMa = jsonCmdReceive["ssa"];
  if (jsonCmdReceive.containsKey("ssabs")) q.stallAbsMa = jsonCmdReceive["ssabs"];
  if (jsonCmdReceive.containsKey("ssms")) q.stallMs = jsonCmdReceive["ssms"];
  turnStatusFeedback();
}

// {"T":165,"ms":5500,"spd":0.16,"hd":12.5,"ramp":500}
void straightStart() {
  if (gyroHead.calibrating) {
    turnStatusFeedback();
    return;
  }
  long ms = jsonCmdReceive["ms"] | 0L;
  if (ms < 0) ms = 0;
  if (ms > 120000) ms = 120000;
  float spd = jsonCmdReceive["spd"] | 0.2f;
  float ramp = (jsonCmdReceive["ramp"] | 500L) * 1e-3f;
  double hd = jsonCmdReceive.containsKey("hd") ? (double)(jsonCmdReceive["hd"].as<float>())
                                                : gyroHead.heading;
  turnCtrl.abort();
  straightCtrl.start(millis() * 1e-3, hd, spd, ms * 1e-3f, ramp);
  straightLastTickMs = millis();
  heartbeatStopFlag = false;
  lastCmdRecvTime = millis();
  motorSafetyFeed();
  turnStatusFeedback();
}
