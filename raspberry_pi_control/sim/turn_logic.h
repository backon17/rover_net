// turn_logic.h -- on-board gyro heading + closed-loop in-place turn controller.
//
// Pure C++ (no Arduino dependencies) so the exact same code is compiled into
// the firmware and into the host-side simulator (sim/turn_sim.cpp).
//
// Conventions:
//   * heading / rate: degrees, degrees per second, left turn (CCW) positive
//   * motor outputs: same unit as the T:1 command (-0.5 .. +0.5), left/right
//   * time: seconds (double)
#pragma once
#include <math.h>

// ---------------------------------------------------------------------------
// Gyro-only heading: integrates the bias-corrected z gyro every loop with the
// real elapsed time. Independent of the magnetometer (which the motors disturb).
// ---------------------------------------------------------------------------
struct GyroHeading {
  float bias = 0.0f;      // dps, measured while stationary
  float scale = 1.0f;     // gyro scale correction (1.0 = trust datasheet)
  float sign = 1.0f;      // +1: positive gz means left turn
  double heading = 0.0;   // deg, unwrapped
  float rate = 0.0f;      // deg/s, low-pass filtered
  float rateTau = 0.02f;  // s, rate filter time constant

  bool calibrating = false;
  bool calZero = true;
  float calLeft = 0.0f;
  double calSum = 0.0;
  double calSumSq = 0.0;
  long calN = 0;
  float noise = 0.0f;     // dps, std of gz during the last calibration (stillness check)

  bool primed = false;
  float lastW = 0.0f;

  void startCalib(float secs, bool zero) {
    calibrating = true;
    calZero = zero;
    calLeft = secs;
    calSum = 0.0;
    calSumSq = 0.0;
    calN = 0;
  }

  // gzRaw: sensor z rate in dps; dt: seconds since the previous sample.
  void update(float gzRaw, float dt) {
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 1.0f) dt = 1.0f;   // a frozen loop longer than 1 s is not recoverable anyway
    if (calibrating) {
      calSum += gzRaw;
      calSumSq += (double)gzRaw * gzRaw;
      calN++;
      calLeft -= dt;
      if (calLeft <= 0.0f) {
        if (calN > 0) {
          double mean = calSum / calN;
          double var = calSumSq / calN - mean * mean;
          bias = (float)mean;
          noise = (float)sqrt(var > 0.0 ? var : 0.0);
        }
        calibrating = false;
        if (calZero) heading = 0.0;
        rate = 0.0f;
        primed = false;
      }
    }
    float w = (gzRaw - bias) * scale * sign;
    if (!primed) {
      lastW = w;
      primed = true;
    }
    // trapezoidal integration: robust to irregular loop timing
    heading += 0.5 * (double)(w + lastW) * dt;
    lastW = w;
    float a = dt / (rateTau + dt);
    rate += a * (w - rate);
  }
};

// ---------------------------------------------------------------------------
// In-place turn controller (skid-steer friendly):
//   KICK   : short high-PWM burst to break static friction
//   RUN    : cruise, slow down inside slowDeg, re-kick if the body stalls
//            cut power (short brake) when remaining <= predicted coast
//   PULSE  : small corrections (< pulseDeg): one timed kick, then brake.
//            Break-away from static friction makes the body jump, so a
//            "drive until close" approach overshoots small errors; the pulse
//            length is learned from the angle each pulse actually produced.
//   SETTLE : wait until the body is still, learn the coast, then either
//            finish or start a small correction segment
// ---------------------------------------------------------------------------
enum TurnState {
  TS_IDLE = 0,
  TS_RUNNING = 1,
  TS_DONE = 2,
  TS_ABORTED = 3,
  TS_TIMEOUT = 4,
  TS_STALLED = 5,
};

enum TurnPhase { PH_KICK = 0, PH_RUN = 1, PH_SETTLE = 2, PH_PULSE = 3 };

struct TurnParams {
  float kick = 0.40f;        // PWM used to break static friction
  float kickMs = 250.0f;     // max kick duration
  float kickRate = 15.0f;    // dps; leave KICK once the body turns this fast
  float minPwm = 0.22f;      // lowest PWM while approaching (just above dead zone)
  float corrPwm = 0.24f;     // cruise PWM for correction segments
  float slowDeg = 40.0f;     // slow-down zone before the target
  float tol = 1.5f;          // deg, success tolerance
  float coastK = 0.10f;      // s; predicted coast = rate * coastK + coastC (learned)
  float coastC = 0.5f;       // deg
  float learn = 0.5f;        // EMA weight for coastK learning
  int maxCorr = 4;           // max correction segments
  float pulseDeg = 6.0f;     // corrections smaller than this use a timed pulse
  float pulseGain = 0.10f;   // deg per ms of pulse (learned)
  float pulseMinMs = 15.0f;
  float pulseMaxMs = 120.0f;
  float stallMs = 250.0f;    // no motion for this long while driving -> re-kick
  int maxRekick = 6;
  float settleRate = 3.0f;   // dps considered "still"
  float settleMs = 120.0f;
  float settleMaxMs = 1500.0f;
  // V2.4.3: turn-rate floor. Cars differ in dead zone, so the same PWM turns
  // one car at 100 dps and another at 15 dps (90 deg took 1.8 s vs 5.1 s).
  // In RUN a desired rate rDes = rateCruise, tapering to rateMin inside the
  // slow-down zone, is compared with the gyro rate; a deficit integrates into
  // an extra PWM (>= 0, <= boostMax). Fast cars never get any boost.
  float rateCruise = 60.0f;  // dps
  float rateMin = 20.0f;     // dps, floor near the target
  float boostK = 0.010f;     // PWM per dps of deficit per second
  float boostMax = 0.15f;
};

struct TurnCtrl {
  TurnParams p;
  int state = TS_IDLE;
  int phase = PH_SETTLE;
  double target = 0.0;       // absolute heading target (deg, unwrapped)
  float cruise = 0.3f;
  float outL = 0.0f, outR = 0.0f;
  float segDir = 1.0f;
  int segs = 0;
  int rekicks = 0;
  double tStart = 0, tPhase = 0, tStill = -1, tDeadline = 0, tMotion = 0;
  double brakeHeading = 0.0;
  float brakeRate = 0.0f;
  double segStartHeading = 0.0;
  float pulseMs = 0.0f;
  bool segIsPulse = false;
  float lastErr = 0.0f;
  float lastPwm = 0.0f;
  float boost = 0.0f;        // rate-floor boost (PWM), reset per turn
  float boostPeak = 0.0f;
  double tLast = 0;

  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  bool running() const { return state == TS_RUNNING; }

  void start(double now, double heading, float deg, float spd, float timeoutS) {
    target = heading + deg;
    cruise = clampf(fabsf(spd), p.minPwm, 0.5f);
    if (timeoutS <= 0.0f) timeoutS = 3.0f + fabsf(deg) / 15.0f;
    tStart = now;
    tDeadline = now + timeoutS;
    segs = 0;
    rekicks = 0;
    lastErr = deg;
    boost = boostPeak = 0.0f;
    tLast = now;
    state = TS_RUNNING;
    if (fabsf(deg) <= p.tol) {
      finish(TS_DONE);
      return;
    }
    beginSegment(now, heading, deg);
  }

  void abort() {
    if (state == TS_RUNNING) finish(TS_ABORTED);
  }

  void beginSegment(double now, double heading, float err) {
    segDir = err >= 0.0f ? 1.0f : -1.0f;
    segs++;
    segStartHeading = heading;
    tPhase = now;
    tMotion = now;
    segIsPulse = fabsf(err) < p.pulseDeg;
    if (segIsPulse) {
      pulseMs = clampf(fabsf(err) / (p.pulseGain > 1e-3f ? p.pulseGain : 1e-3f),
                       p.pulseMinMs, p.pulseMaxMs);
      phase = PH_PULSE;
    } else {
      phase = PH_KICK;
    }
  }

  void finish(int st) {
    state = st;
    phase = PH_SETTLE;
    outL = outR = 0.0f;
    lastPwm = 0.0f;
  }

  void drive(float pwm) {
    lastPwm = pwm;
    outL = -pwm * segDir;
    outR = pwm * segDir;
  }

  void brake(double now, double heading, float alongRate) {
    outL = outR = 0.0f;
    lastPwm = 0.0f;
    brakeHeading = heading;
    brakeRate = alongRate;
    phase = PH_SETTLE;
    tPhase = now;
    tStill = -1;
  }

  // Call every control loop with the latest gyro heading and rate.
  void step(double now, double heading, float rate) {
    if (state != TS_RUNNING) {
      outL = outR = 0.0f;
      return;
    }
    float err = (float)(target - heading);
    lastErr = err;
    float dt = clampf((float)(now - tLast), 0.0f, 0.1f);
    tLast = now;
    if (now > tDeadline) {
      finish(TS_TIMEOUT);
      return;
    }
    float rem = err * segDir;          // remaining along this segment (<0: overshoot)
    float along = rate * segDir;       // rate along this segment

    if (phase == PH_PULSE) {
      float pred = (along > 0.0f ? along : 0.0f) * p.coastK + p.coastC;
      if ((now - tPhase) * 1000.0 >= pulseMs || rem <= pred) {
        brake(now, heading, along);
        return;
      }
      drive(p.kick > p.minPwm ? p.kick : p.minPwm);
      return;
    }

    if (phase == PH_KICK || phase == PH_RUN) {
      float pred = (along > 0.0f ? along : 0.0f) * p.coastK + p.coastC;
      if (rem <= pred) {
        brake(now, heading, along);
        return;
      }
      float segCruise = (segs > 1) ? clampf(p.corrPwm, p.minPwm, cruise) : cruise;
      if (phase == PH_KICK) {
        drive(p.kick > segCruise ? p.kick : segCruise);
        if (along >= p.kickRate || (now - tPhase) * 1000.0 >= p.kickMs) {
          phase = PH_RUN;
          tPhase = now;
          tMotion = now;
        }
        return;
      }
      // PH_RUN
      float pwm = segCruise;
      if (rem < p.slowDeg && p.slowDeg > 0.0f) {
        pwm = p.minPwm + (segCruise - p.minPwm) * (rem / p.slowDeg);
      }
      float rDes = p.rateCruise;
      if (rem < p.slowDeg && p.slowDeg > 0.0f) rDes = p.rateMin + (p.rateCruise - p.rateMin) * (rem / p.slowDeg);
      if (rDes < p.rateMin) rDes = p.rateMin;
      if ((now - tPhase) * 1000.0 >= 80.0) {
        boost = clampf(boost + p.boostK * (rDes - along) * dt, 0.0f, p.boostMax);
        if (boost > boostPeak) boostPeak = boost;
      }
      pwm = clampf(pwm + boost, p.minPwm, 0.5f);
      drive(pwm);
      if (along > 5.0f) {
        tMotion = now;
      } else if ((now - tMotion) * 1000.0 >= p.stallMs) {
        if (++rekicks > p.maxRekick) {
          finish(TS_STALLED);
          return;
        }
        phase = PH_KICK;
        tPhase = now;
      }
      return;
    }

    // PH_SETTLE: motors braked, wait until still
    outL = outR = 0.0f;
    bool settled = false;
    if (fabsf(rate) < p.settleRate) {
      if (tStill < 0) tStill = now;
      else if ((now - tStill) * 1000.0 >= p.settleMs) settled = true;
    } else {
      tStill = -1;
    }
    if ((now - tPhase) * 1000.0 >= p.settleMaxMs) settled = true;
    if (!settled) return;

    if (segIsPulse) {
      float achieved = (float)(heading - segStartHeading) * segDir;
      if (achieved > 0.3f) {
        float g = clampf(achieved / pulseMs, 0.005f, 1.0f);
        p.pulseGain = (1.0f - p.learn) * p.pulseGain + p.learn * g;
      } else {
        p.pulseGain *= 0.6f;     // did not break free: next pulse longer
      }
    } else if (brakeRate > 10.0f) {
      float coast = (float)(heading - brakeHeading) * segDir;
      float k = clampf((coast - p.coastC) / brakeRate, 0.0f, 0.5f);
      p.coastK = (1.0f - p.learn) * p.coastK + p.learn * k;
    }
    brakeRate = 0.0f;
    if (fabsf(err) <= p.tol || segs > p.maxCorr) {
      finish(TS_DONE);
      return;
    }
    beginSegment(now, heading, err);
  }
};

// ---------------------------------------------------------------------------
// Straight-line drive with on-board heading hold (V2.3, V2.4).
// Distance is open loop (no wheel encoders): the PC converts metres to a
// duration. The heading is held by a PID on the gyro heading at loop rate, so
// WiFi latency no longer affects the line. Both wheels always keep driving:
// the correction is limited to a fraction of the base PWM, so one side never
// drops into the motor dead zone (which made the old PC loop jerky).
//
// V2.4:
//  * The left/right motor imbalance is a property of the car, so the integral
//    is remembered between segments (normalised by the cruise PWM, one value
//    per direction) instead of being re-learned from zero every segment.
//  * Stall detection from the battery current (INA219, fed by the firmware
//    glue through feedCurrent()): after the soft start, a baseline current is
//    measured; a sustained rise well above it means the car is pushing
//    against something -> SS_STALL. The gyro cannot see this (the heading does
//    not change when the car is blocked head-on).
// ---------------------------------------------------------------------------
enum StraightState {
  SS_IDLE = 0,
  SS_RUNNING = 1,
  SS_DONE = 2,
  SS_ABORTED = 3,
  SS_FAULT = 4,      // heading error too large (wheel stuck / hit something)
  SS_STALL = 5,      // V2.4: current stayed far above the cruise baseline
};

struct StraightParams {
  float kp = 0.020f;       // PWM per deg of heading error (V2.4: was 0.010)
  float ki = 0.012f;       // PWM per deg*s (V2.4: was 0.006)
  float kd = 0.0008f;      // PWM per dps (damping on the measured yaw rate)
  float iMax = 0.06f;      // integral clamp (PWM)
  float corrFrac = 0.5f;   // |correction| <= corrFrac * |base|
  float ramp0 = 0.4f;      // ramp starts at this fraction of the cruise PWM
  float maxErr = 30.0f;    // deg; larger heading error -> SS_FAULT
  bool keepI = true;       // carry the integral (motor trim) across segments
  // stall detection (battery current)
  bool stallOn = true;
  float stallRatio = 1.5f;   // stall if I > baseline * ratio ...
  float stallAddMa = 120.0f; // ... and I > baseline + addMa
  float stallAbsMa = 0.0f;   // optional absolute limit (0 = off); also covers
                             // a car that starts already blocked
  float stallMs = 400.0f;    // condition must hold this long
  float baseDelayS = 0.25f;  // baseline window starts this long after the ramp
  float baseWinS = 0.5f;     // baseline window length
  float iTau = 0.06f;        // s, low-pass on the current samples
};

struct StraightCtrl {
  StraightParams p;
  int state = SS_IDLE;
  double target = 0.0;     // absolute heading to hold (deg, unwrapped)
  float pwm = 0.0f;        // signed cruise PWM (negative = backwards)
  float rampS = 0.5f;
  double t0 = 0, tEnd = 0, tLast = 0;
  float iTerm = 0.0f;
  float iMem[2] = {0.0f, 0.0f};  // remembered integral / |cruise PWM| (fwd, back)
  float outL = 0.0f, outR = 0.0f;
  float lastErr = 0.0f;
  float maxAbsErr = 0.0f;
  double errAbsSum = 0.0;
  long errN = 0;
  // current / stall
  float iFilt = 0.0f;       // filtered current, mA
  double iLastT = -1.0;
  double baseSum = 0.0;
  long baseN = 0;
  float baseMa = 0.0f;      // baseline current (0 = not measured yet)
  float peakMa = 0.0f;      // max filtered current after the baseline window
  double overSince = -1.0;

  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  bool running() const { return state == SS_RUNNING; }
  int dirIdx() const { return pwm >= 0.0f ? 0 : 1; }

  void start(double now, double hdTarget, float spd, float durS, float ramp) {
    target = hdTarget;
    pwm = clampf(spd, -0.5f, 0.5f);
    rampS = ramp < 0.0f ? 0.0f : ramp;
    t0 = tLast = now;
    tEnd = now + (durS > 0.0f ? durS : 0.0f);
    iTerm = p.keepI ? clampf(iMem[dirIdx()] * fabsf(pwm), -p.iMax, p.iMax) : 0.0f;
    lastErr = 0.0f;
    maxAbsErr = 0.0f;
    errAbsSum = 0.0;
    errN = 0;
    iFilt = 0.0f;
    iLastT = -1.0;
    baseSum = 0.0;
    baseN = 0;
    baseMa = 0.0f;
    peakMa = 0.0f;
    overSince = -1.0;
    state = SS_RUNNING;
  }

  void finish(int st) {
    // Remember the trim only from a clean, long enough segment.
    if (st == SS_DONE && p.keepI && fabsf(pwm) > 0.01f && errN > 100)
      iMem[dirIdx()] = clampf(iTerm / fabsf(pwm), -0.5f, 0.5f);
    state = st;
    outL = outR = 0.0f;
  }

  void abort() {
    if (state == SS_RUNNING) finish(SS_ABORTED);
  }

  float progress(double now) const {
    if (state == SS_IDLE) return 0.0f;
    if (state != SS_RUNNING) return 1.0f;
    double d = tEnd - t0;
    return d <= 0.0 ? 1.0f : clampf((float)((now - t0) / d), 0.0f, 1.0f);
  }

  float meanAbsErr() const { return errN > 0 ? (float)(errAbsSum / errN) : 0.0f; }

  // Battery current sample (mA), ~20-50 Hz while running.
  void feedCurrent(double now, float mA) {
    if (state != SS_RUNNING) return;
    if (iLastT < 0.0) {
      iFilt = mA;
    } else {
      float dt = clampf((float)(now - iLastT), 0.0f, 0.5f);
      float a = dt / (p.iTau + dt);
      iFilt += a * (mA - iFilt);
    }
    iLastT = now;
    if (!p.stallOn) return;
    double tb0 = t0 + rampS + p.baseDelayS;
    double tb1 = tb0 + p.baseWinS;
    if (now >= tb0 && now < tb1) {
      baseSum += iFilt;
      baseN++;
    } else if (now >= tb1 && baseMa == 0.0f && baseN > 0) {
      baseMa = (float)(baseSum / baseN);
      if (baseMa < 1.0f) baseMa = 1.0f;
    }
    bool over = false;
    if (p.stallAbsMa > 0.0f && iFilt > p.stallAbsMa) over = true;
    if (baseMa > 0.0f) {
      if (iFilt > peakMa) peakMa = iFilt;
      if (iFilt > baseMa * p.stallRatio && iFilt > baseMa + p.stallAddMa) over = true;
    }
    if (!over) {
      overSince = -1.0;
    } else if (overSince < 0.0) {
      overSince = now;
    } else if ((now - overSince) * 1000.0 >= p.stallMs) {
      finish(SS_STALL);
    }
  }

  void step(double now, double heading, float rate) {
    if (state != SS_RUNNING) {
      outL = outR = 0.0f;
      return;
    }
    if (now >= tEnd) {
      finish(SS_DONE);
      return;
    }
    float dt = clampf((float)(now - tLast), 0.0f, 0.1f);
    tLast = now;
    float err = (float)(target - heading);
    lastErr = err;
    if (fabsf(err) > maxAbsErr) maxAbsErr = fabsf(err);
    errAbsSum += fabsf(err);
    errN++;
    if (fabsf(err) > p.maxErr) {
      finish(SS_FAULT);
      return;
    }
    float k = 1.0f;
    if (rampS > 0.0f) k = clampf(p.ramp0 + (1.0f - p.ramp0) * (float)(now - t0) / rampS, 0.0f, 1.0f);
    float base = pwm * k;
    iTerm = clampf(iTerm + p.ki * err * dt, -p.iMax, p.iMax);
    float corr = p.kp * err + iTerm - p.kd * rate;
    float lim = p.corrFrac * fabsf(base);
    corr = clampf(corr, -lim, lim);
    // positive corr turns left (CCW) in both directions: omega ~ (vR - vL)
    outL = base - corr;
    outR = base + corr;
  }
};
