// flight_core.cpp — the entry point the MCU firmware links against.
// Built with -ffreestanding -nostdlib in `make freestanding` to prove the
// flight code needs nothing beyond the compiler itself.
#include "rw_controller.hpp"

namespace {
adcs::ReactionWheelController g_ctrl;   // static storage, no heap
}

extern "C" {

void adcs_on_imu(const adcs::ImuSample* s)       { g_ctrl.ingest(*s); }
void adcs_on_fix(const adcs::AttitudeFix* f)     { g_ctrl.ingest(*f); }
void adcs_on_tach(const adcs::WheelTach* t)      { g_ctrl.ingest(*t); }
void adcs_set_target(float angle_rad)            { g_ctrl.set_target(angle_rad); }

// Called from the 1 kHz timer ISR; returns signed PWM compare value.
int16_t adcs_control_isr(void) { return g_ctrl.step().pwm; }

}
