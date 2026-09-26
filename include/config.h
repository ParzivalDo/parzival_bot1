#pragma once
#include <Arduino.h>
#include <initializer_list>
namespace cfg {
constexpr uint32_t LA=PA1, LB=PA0, RA=PA6, RB=PA7;
constexpr uint32_t PWM_L=PB14, PWM_R=PB13;
constexpr uint32_t L1=PB1, L2=PB0, R1=PB10, R2=PB11;
constexpr uint32_t SW2=PB9, SW3=PB8, LED=PC13;
constexpr uint32_t FORWARD=PA15;
// Direction depends on physical assembly: check README before floor test.
constexpr bool REVERSE_MOTOR_L=false, REVERSE_MOTOR_R=true;
constexpr int ENCODER_SIGN_L=1, ENCODER_SIGN_R=-1; // Forward motion verified via ST-Link.
// USER REPORT: 210 counts/output revolution. Vendor does not specify x1/x4.
// Provisional assumption: 210 already includes x4. Measure one full wheel turn.
// If 210 is single-channel pulses/rev instead, x4 counter yields 840.
constexpr float COUNTS_PER_REV_X4_L=210.0f, COUNTS_PER_REV_X4_R=210.0f;
// Wheel diameter 34 mm supplied by user; counts/rev still need calibration.
constexpr float WHEEL_DIAMETER_MM_L=34.0f, WHEEL_DIAMETER_MM_R=34.0f;
constexpr float COUNTS_PER_MM_L=COUNTS_PER_REV_X4_L/(3.14159265f*WHEEL_DIAMETER_MM_L);
constexpr float COUNTS_PER_MM_R=COUNTS_PER_REV_X4_R/(3.14159265f*WHEEL_DIAMETER_MM_R);
constexpr float SPEED_MM_S=500.0f;
constexpr uint32_t DRIVE_TIME_MS=6000; // Includes acceleration and deceleration
constexpr float ACCEL_MM_S2=300.0f, DECEL_MM_S2=500.0f;
constexpr float MAX_DUTY=60.0f; // percent, headroom for independent wheel PI
constexpr float FF_STATIC=10.0f, FF_PER_SPEED=0.05f;
constexpr float SPEED_KP=0.08f, SPEED_KI=0.12f;
constexpr float STRAIGHT_KP=8.0f, MAX_SYNC_MM_S=200.0f;
constexpr float SYNC_SPEED_KP=0.35f;
constexpr float SYNC_SOFT_MM=10.0f; // Correct skew continuously; no mismatch shutdown
constexpr float OVERSPEED_MM_S=800.0f; // Above 500 + 200 correction, with transient margin
constexpr uint32_t CONTROL_US=10000;
constexpr bool CALIBRATION_ONLY=false; // true: motors free, print raw counts
static_assert(ENCODER_SIGN_L==1 || ENCODER_SIGN_L==-1,"Left encoder sign must be +/-1");
static_assert(ENCODER_SIGN_R==1 || ENCODER_SIGN_R==-1,"Right encoder sign must be +/-1");
static_assert(COUNTS_PER_MM_L>0 && COUNTS_PER_MM_R>0,"Counts/mm must be positive");
static_assert(OVERSPEED_MM_S>SPEED_MM_S+MAX_SYNC_MM_S,"Overspeed guard must exceed wheel reference");
static_assert(DRIVE_TIME_MS>0 && DRIVE_TIME_MS<60000,"Invalid drive duration" );
static_assert(MAX_DUTY>0 && MAX_DUTY<=100,"Invalid PWM limit");
}
