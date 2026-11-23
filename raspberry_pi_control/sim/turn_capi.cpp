// turn_capi.cpp -- C API around firmware_v2/turn_logic.h for the Python
// end-to-end simulator (fake_v2.py). Same code as compiled into the firmware.
#include "turn_logic.h"

extern "C" {
float gh_noise(void *h) { return ((GyroHeading *)h)->noise; }
void *sc_new() { return new StraightCtrl(); }
void sc_start(void *s, double now, double hd, float spd, float dur, float ramp) {
  ((StraightCtrl *)s)->start(now, hd, spd, dur, ramp);
}
void sc_step(void *s, double now, double heading, float rate) { ((StraightCtrl *)s)->step(now, heading, rate); }
void sc_abort(void *s) { ((StraightCtrl *)s)->abort(); }
int sc_state(void *s) { return ((StraightCtrl *)s)->state; }
float sc_l(void *s) { return ((StraightCtrl *)s)->outL; }
float sc_r(void *s) { return ((StraightCtrl *)s)->outR; }
float sc_progress(void *s, double now) { return ((StraightCtrl *)s)->progress(now); }
float sc_err(void *s) { return ((StraightCtrl *)s)->lastErr; }
float sc_maxerr(void *s) { return ((StraightCtrl *)s)->maxAbsErr; }
float sc_meanerr(void *s) { return ((StraightCtrl *)s)->meanAbsErr(); }
void sc_set(void *s, float kp, float ki, float kd, float cf, float r0) {
  StraightParams &p = ((StraightCtrl *)s)->p;
  if (kp >= 0) p.kp = kp; if (ki >= 0) p.ki = ki; if (kd >= 0) p.kd = kd;
  if (cf >= 0) p.corrFrac = cf; if (r0 >= 0) p.ramp0 = r0;
}
// V2.4: current feed / stall detection / integral carry-over
void sc_feed_current(void *s, double now, float mA) { ((StraightCtrl *)s)->feedCurrent(now, mA); }
float sc_base(void *s) { return ((StraightCtrl *)s)->baseMa; }
float sc_peak(void *s) { return ((StraightCtrl *)s)->peakMa; }
float sc_ifilt(void *s) { return ((StraightCtrl *)s)->iFilt; }
float sc_imem(void *s, int i) { return ((StraightCtrl *)s)->iMem[i ? 1 : 0]; }
void sc_set_imem(void *s, float f, float b) { ((StraightCtrl *)s)->iMem[0] = f; ((StraightCtrl *)s)->iMem[1] = b; }
void sc_set_v24(void *s, int keepI, int stallOn, float ratio, float addMa, float absMa, float ms) {
  StraightParams &p = ((StraightCtrl *)s)->p;
  if (keepI >= 0) p.keepI = keepI != 0;
  if (stallOn >= 0) p.stallOn = stallOn != 0;
  if (ratio >= 0) p.stallRatio = ratio;
  if (addMa >= 0) p.stallAddMa = addMa;
  if (absMa >= 0) p.stallAbsMa = absMa;
  if (ms >= 0) p.stallMs = ms;
}
void *gh_new() { return new GyroHeading(); }
void gh_update(void *h, float gz, float dt) { ((GyroHeading *)h)->update(gz, dt); }
double gh_heading(void *h) { return ((GyroHeading *)h)->heading; }
float gh_rate(void *h) { return ((GyroHeading *)h)->rate; }
float gh_bias(void *h) { return ((GyroHeading *)h)->bias; }
void gh_set_scale(void *h, float s) { ((GyroHeading *)h)->scale = s; }
void gh_calib(void *h, float secs, int zero) { ((GyroHeading *)h)->startCalib(secs, zero != 0); }
int gh_calibrating(void *h) { return ((GyroHeading *)h)->calibrating ? 1 : 0; }

void *tc_new() { return new TurnCtrl(); }
void tc_start(void *t, double now, double heading, float deg, float spd, float tmo) {
  ((TurnCtrl *)t)->start(now, heading, deg, spd, tmo);
}
void tc_step(void *t, double now, double heading, float rate) { ((TurnCtrl *)t)->step(now, heading, rate); }
void tc_abort(void *t) { ((TurnCtrl *)t)->abort(); }
float tc_l(void *t) { return ((TurnCtrl *)t)->outL; }
float tc_r(void *t) { return ((TurnCtrl *)t)->outR; }
int tc_state(void *t) { return ((TurnCtrl *)t)->state; }
int tc_phase(void *t) { return ((TurnCtrl *)t)->phase; }
float tc_err(void *t) { return ((TurnCtrl *)t)->lastErr; }
int tc_segs(void *t) { return ((TurnCtrl *)t)->segs; }
int tc_rekicks(void *t) { return ((TurnCtrl *)t)->rekicks; }
float tc_ck(void *t) { return ((TurnCtrl *)t)->p.coastK; }
float tc_boost(void *t) { return ((TurnCtrl *)t)->boostPeak; }
void tc_set_boost(void *t, float rcruise, float k, float bmax) {
  TurnParams &p = ((TurnCtrl *)t)->p;
  if (rcruise >= 0) p.rateCruise = rcruise; if (k >= 0) p.boostK = k; if (bmax >= 0) p.boostMax = bmax;
}
void tc_set(void *t, float kick, float minp) {
  ((TurnCtrl *)t)->p.kick = kick;
  ((TurnCtrl *)t)->p.minPwm = minp;
}
}
