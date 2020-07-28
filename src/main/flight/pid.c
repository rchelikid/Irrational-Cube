/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"

#include "config/config_reset.h"

#include "drivers/dshot_command.h"
#include "drivers/dshot.h"
#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/interpolated_setpoint.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT bool pidStabilisationEnabled;

static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

static FAST_RAM_ZERO_INIT uint8_t antiGravityMode;
static FAST_RAM_ZERO_INIT float antiGravityThrottleHpf;
static FAST_RAM_ZERO_INIT uint16_t itermAcceleratorGain;
static FAST_RAM_ZERO_INIT float antiGravityOsdCutoff;
static FAST_RAM_ZERO_INIT bool antiGravityEnabled;
static FAST_RAM_ZERO_INIT bool zeroThrottleItermReset;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#if defined(STM32F1)
#define PID_PROCESS_DENOM_DEFAULT       8
#elif defined(STM32F3)
#define PID_PROCESS_DENOM_DEFAULT       4
#elif defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif




PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);






#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15  // The anti gravity throttle highpass filter cutoff

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 15);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 90, 1 },
            [PID_PITCH] = { 46, 90, 38, 95, 1 },
            [PID_YAW] =   { 45, 90, 0, 90, 1 },
            [PID_LEVEL] = { 50, 50, 75, 0, 0 },
            [PID_MAG] =   { 40, 0, 0, 0, 0 },

        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 100,
        .vbatPidCompensation = 0,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 55,
        .feedForwardTransition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .itermThrottleThreshold = 250,
        .itermAcceleratorGain = 3500,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .antiGravityMode = ANTI_GRAVITY_SMOOTH,
        .dterm_lowpass_hz = 150,    // NOTE: dynamic lpf is enabled by default so this setting is actually
                                    // overridden and the static lowpass 1 is disabled. We can't set this
                                    // value to 0 otherwise Configurator versions 10.4 and earlier will also
                                    // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lowpass2_hz = 150,   // second Dterm LPF ON by default
        .dterm_filter_type = FILTER_PT1,
        .dterm_filter2_type = FILTER_PT1,
        .dyn_lpf_dterm_min_hz = 70,
        .dyn_lpf_dterm_max_hz = 170,
        .launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_min = { 23, 25, 0 },      // roll, pitch, yaw
        .d_min_gain = 37,
        .d_min_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .idle_min_rpm = 0,
        .idle_adjustment_speed = 50,
        .idle_p = 50,
        .idle_pid_limit = 200,
        .idle_max_increase = 150,
        .ff_interpolate_sp = FF_INTERPOLATE_AVG2,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .dyn_lpf_curve_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
        .setpointgain_cubli = 50,
        .midrpm_cubli = 750,
    );
#ifndef USE_D_MIN
    pidProfile->pid[PID_ROLL].D = 30;
    pidProfile->pid[PID_PITCH].D = 32;
#endif
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

static FAST_RAM_ZERO_INIT float itermAccelerator;

void pidSetItermAccelerator(float newItermAccelerator)
{
    itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (itermAccelerator > antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float cubliSetpoint[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermNotchApplyFn;
static FAST_RAM_ZERO_INIT biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpassApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpass2ApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr ptermYawLowpassApplyFn;
static FAST_RAM_ZERO_INIT pt1Filter_t ptermYawLowpass;

#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static uint8_t itermRelaxCutoff;
#endif






static FAST_RAM_ZERO_INIT pt1Filter_t antiGravityThrottleLpf;

static FAST_RAM_ZERO_INIT float ffBoostFactor;
static FAST_RAM_ZERO_INIT float ffSmoothFactor;
static FAST_RAM_ZERO_INIT float ffSpikeLimitInverse;

float pidGetSpikeLimitInverse()
{
    return ffSpikeLimitInverse;
}


float pidGetFfBoostFactor()
{
    return ffBoostFactor;
}

float pidGetFfSmoothFactor()
{
    return ffSmoothFactor;
}


void pidInitFilters(const pidProfile_t *pidProfile)
{
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        dtermNotchApplyFn = nullFilterApply;
        dtermLowpassApplyFn = nullFilterApply;
        ptermYawLowpassApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    } else {
        dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    uint16_t dterm_lowpass_hz = pidProfile->dterm_lowpass_hz;

#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz) {
        dterm_lowpass_hz = pidProfile->dyn_lpf_dterm_min_hz;
    }
#endif

    if (dterm_lowpass_hz > 0 && dterm_lowpass_hz < pidFrequencyNyquist) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lowpass_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
#ifdef USE_DYN_LPF
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1;
#else
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
#endif
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterInitLPF(&dtermLowpass[axis].biquadFilter, dterm_lowpass_hz, targetPidLooptime);
            }
            break;
        default:
            dtermLowpassApplyFn = nullFilterApply;
            break;
        }
    } else {
        dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lowpass2_hz == 0 || pidProfile->dterm_lowpass2_hz > pidFrequencyNyquist) {
    	dtermLowpass2ApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter2_type) {
        case FILTER_PT1:
            dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lowpass2_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
            dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterInitLPF(&dtermLowpass2[axis].biquadFilter, pidProfile->dterm_lowpass2_hz, targetPidLooptime);
            }
            break;
        default:
            dtermLowpass2ApplyFn = nullFilterApply;
            break;
        }
    }

    if (pidProfile->yaw_lowpass_hz == 0 || pidProfile->yaw_lowpass_hz > pidFrequencyNyquist) {
        ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, dT));
    }

#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif




    pt1FilterInit(&antiGravityThrottleLpf, pt1FilterGain(ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF, dT));

    ffBoostFactor = (float)pidProfile->ff_boost / 10.0f;
    ffSpikeLimitInverse = pidProfile->ff_spike_limit ? 1.0f / ((float)pidProfile->ff_spike_limit / 10.0f) : 0.0f;
}


typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float Kr; //KYLE added
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float feedForwardTransition;
static FAST_RAM_ZERO_INIT float levelGain, horizonGain, horizonTransition, horizonCutoffDegrees, horizonFactorRatio;
static FAST_RAM_ZERO_INIT float itermWindupPointInv;
static FAST_RAM_ZERO_INIT uint8_t horizonTiltExpertMode;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeLimitUs;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeDelayUs;
static FAST_RAM_ZERO_INIT int32_t crashRecoveryAngleDeciDegrees;
static FAST_RAM_ZERO_INIT float crashRecoveryRate;
static FAST_RAM_ZERO_INIT float crashDtermThreshold;
static FAST_RAM_ZERO_INIT float crashGyroThreshold;
static FAST_RAM_ZERO_INIT float crashSetpointThreshold;
static FAST_RAM_ZERO_INIT float crashLimitYaw;
static FAST_RAM_ZERO_INIT float itermLimit;

static FAST_RAM_ZERO_INIT bool itermRotation;



static FAST_RAM_ZERO_INIT bool levelRaceMode;

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
}

#ifdef USE_DYN_LPF
static FAST_RAM uint8_t dynLpfFilter = DYN_LPF_NONE;
static FAST_RAM_ZERO_INIT uint16_t dynLpfMin;
static FAST_RAM_ZERO_INIT uint16_t dynLpfMax;
static FAST_RAM_ZERO_INIT uint8_t dynLpfCurveExpo;
#endif


void pidInitConfig(const pidProfile_t *pidProfile)
{
    if (pidProfile->feedForwardTransition == 0) {
        feedForwardTransition = 0;
    } else {
        feedForwardTransition = 100.0f / pidProfile->feedForwardTransition;
    }
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
        pidCoefficient[axis].Kr = RTERM_SCALE * pidProfile->pid[axis].R;
    }
#ifdef USE_INTEGRATED_YAW_CONTROL
    if (!pidProfile->use_integrated_yaw)
#endif
    {
        pidCoefficient[FD_YAW].Ki *= 2.5f;
    }

    levelGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    horizonGain = pidProfile->pid[PID_LEVEL].I / 10.0f;
    horizonTransition = (float)pidProfile->pid[PID_LEVEL].D;
    horizonTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
    horizonCutoffDegrees = (175 - pidProfile->horizon_tilt_effect) * 1.8f;
    horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    itermWindupPointInv = 1.0f;
    if (pidProfile->itermWindupPointPercent < 100) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        itermWindupPointInv = 1.0f / (1.0f - itermWindupPoint);
    }
    itermAcceleratorGain = pidProfile->itermAcceleratorGain;
    crashTimeLimitUs = pidProfile->crash_time * 1000;
    crashTimeDelayUs = pidProfile->crash_delay * 1000;
    crashRecoveryAngleDeciDegrees = pidProfile->crash_recovery_angle * 10;
    crashRecoveryRate = pidProfile->crash_recovery_rate;
    crashGyroThreshold = pidProfile->crash_gthreshold;
    crashDtermThreshold = pidProfile->crash_dthreshold;
    crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    crashLimitYaw = pidProfile->crash_limit_yaw;
    itermLimit = pidProfile->itermLimit;
#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost * 0.1f;
#endif
    itermRotation = pidProfile->iterm_rotation;
    antiGravityMode = pidProfile->antiGravityMode;

    // Calculate the anti-gravity value that will trigger the OSD display.
    // For classic AG it's either 1.0 for off and > 1.0 for on.
    // For the new AG it's a continuous floating value so we want to trigger the OSD
    // display when it exceeds 25% of its possible range. This gives a useful indication
    // of AG activity without excessive display.
    antiGravityOsdCutoff = 0.0f;
    if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        antiGravityOsdCutoff += ((itermAcceleratorGain - 1000) / 1000.0f) * 0.25f;
    }

#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif




#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz > 0) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        default:
            dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        dynLpfFilter = DYN_LPF_NONE;
    }
    dynLpfMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynLpfMax = pidProfile->dyn_lpf_dterm_max_hz;
    dynLpfCurveExpo = pidProfile->dyn_lpf_curve_expo;
#endif



    levelRaceMode = pidProfile->level_race_mode;
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
#ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig());
#endif
}



void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}


// static float accelerationLimit(int axis, float currentPidSetpoint)
// {
//     static float previousSetpoint[XYZ_AXIS_COUNT];
//     const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];
//
//     if (fabsf(currentVelocity) > maxVelocity[axis]) {
//         currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
//     }
//
//     previousSetpoint[axis] = currentPidSetpoint;
//     return currentPidSetpoint;
// }

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (itermRotation) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }

        if (itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

// #if defined(USE_ITERM_RELAX)
// STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
//     const float gyroRate, float *itermErrorTorque, float *currentPidSetpoint)
// {
//     const float setpointLpf = pt1FilterApply(&windupLpf[axis], *currentPidSetpoint);
//     const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
//
//     if (itermRelax) {
//         if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC) {
//             const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
//             const bool isDecreasingI =
//                 ((iterm > 0) && (*itermErrorTorque < 0)) || ((iterm < 0) && (*itermErrorTorque > 0));
//             if ((itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
//                 // Do Nothing, use the precalculed itermErrorTorque
//             } else if (itermRelaxType == ITERM_RELAX_SETPOINT) {
//                 *itermErrorTorque *= itermRelaxFactor;
//             } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
//                 *itermErrorTorque = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
//             } else {
//                 *itermErrorTorque = 0.0f;
//             }
//
//             if (axis == FD_ROLL) {
//                 DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
//                 DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
//                 DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorTorque));
//             }
//         }
//     }
// }
// #endif

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];

    const float tpaFactor = getThrottlePIDAttenuation();

    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    const bool launchControlActive = isLaunchControlActive();

    // Dynamic i component,
    if ((antiGravityMode == ANTI_GRAVITY_SMOOTH) && antiGravityEnabled) {
        itermAccelerator = fabsf(antiGravityThrottleHpf) * 0.01f * (itermAcceleratorGain - 1000);
        DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(antiGravityThrottleHpf * 1000));
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(itermAccelerator * 1000));

    //float agGain = dT * itermAccelerator * AG_KI;

    // gradually scale back integration when above windup point
    float dynCi = dT;
    if (itermWindupPointInv > 1.0f) {
        dynCi *= constrainf((1.0f - getMotorMixRange()) * itermWindupPointInv, 0.0f, 1.0f);
    }

    // Precalculate gyro deta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
#ifdef USE_RPM_FILTER
        gyroRateDterm[axis] = rpmFilterDterm(axis,gyroRateDterm[axis]);
#endif
        gyroRateDterm[axis] = dtermNotchApplyFn((filter_t *) &dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpassApplyFn((filter_t *) &dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpass2ApplyFn((filter_t *) &dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateItermAndAxisError();
#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif



    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis); //GETS RATE IN DEG/SEC KYLE
        // if (maxVelocity[axis]) {
        //     currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint); //MAKES SURE MAX IS NOT EXCEEDED. DEG/SEC KYLE
        // }


// find currentPidSetpoint
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec

        float cubeTorque = (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency;
        float motorTorque = (motor[axis] - motorPrev[axis]) * K * pidFrequency; // what correlates cube torque to motor torque ??
        float balanceTorque = motorTorque - cubeTorque;

        if (featureIsEnabled(FEATURE_CUBLI)) {
        const int rpmSetpoint = (pidProfile->midrpm_cubli);

#ifdef USE_DSHOT_TELEMETRY
        if (motorConfig()->dev.useDshotTelemetry) {
            int rpm = (int)getDshotTelemetry(axis) * 100 * 2 / motorConfig()->motorPoleCount;

            // setpoint is zero
            // setpoint + and left of center = positive (small)
            // setpoint + and right of center = negative (small)
            // setpoint - and left of center = positive (small)
            // setpoint - and right of center = negative (small)

            // rpm negative left of balance (GO)
            if (rpm < rpmSetpoint && balanceTorque > 0.0f) {
              if (gyroRate < 0) {
                currentPidSetpoint += currentPidSetpoint * K * dT;
              } else {
                currentPidSetpoint = currentPidSetpoint;
              }
            }
            // rpm positive right of balance (GO)
            else if (rpm > rpmSetpoint && balanceTorque < 0.0f) {
              if (gyroRate > 0) {
                currentPidSetpoint += currentPidSetpoint * K * dT;
              } else {
                currentPidSetpoint = currentPidSetpoint;
              }
            }
            // rpm negative right of balance (fix) OR rpm positive and left of balance (fix)
            else { //(rpm < rpmSetpoint && balanceTorque < 0.0f) (rpm > rpmSetpoint && balanceTorque > 0.0f)
              currentPidSetpoint = errotTorque * currentPidSetpoint * K;
            }
          }
#endif
      }

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // When Race Mode is active PITCH control is also GYRO based in level or horizon mode

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below


        // -----calculate error rate


        float errorTorque = currentPidSetpoint - motorTorque; // current demand minus slope of motor output


        const float previousIterm = pidData[axis].I;
        float itermErrorTorque = errorTorque;


// #if defined(USE_ITERM_RELAX)
//         if (!launchControlActive && !inCrashRecoveryMode) {
//             applyItermRelax(axis, previousIterm, gyroRate, &itermErrorTorque, &currentPidSetpoint);
//             errorRate = currentPidSetpoint - gyroRate;
//         }
// #endif


        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component

        // error in torque
        pidData[axis].P = pidCoefficient[axis].Kp * errorTorque;
// if (!featureIsEnabled(FEATURE_CUBLI)) {
//         if (axis == FD_YAW) {
//             pidData[axis].P = ptermYawLowpassApplyFn((filter_t *) &ptermYawLowpass, pidData[axis].P);
//         }
// }
        // I component is velocity
        // -----calculate I component
        float Ki;
        float axisDynCi;


        Ki = pidCoefficient[axis].Ki;
        axisDynCi = dT; //(axis == FD_YAW) ? dynCi : dT; // only apply windup protection to yaw

                                                                //+ agGain
        pidData[axis].I = constrainf(previousIterm + (Ki * axisDynCi) * itermErrorTorque, -itermLimit, itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;

        pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];

        previousPidSetpoint[axis] = currentPidSetpoint;

        // -----calculate D component



        // Divide rate change by dT to get differential (ie dr/dt).
        // dT is fixed and calculated from the target PID loop time
        // This is done to avoid DTerm spikes that occur with dynamically
        // calculated deltaT whenever another task causes the PID
        // loop execution to be delayed.
        const float delta =
            - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency;

        float dMinFactor = 1.0f;

        pidData[axis].D = pidCoefficient[axis].Kd * delta * tpaFactor * dMinFactor;

        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component

        // Only enable feedforward for rate mode and if launch control is inactive
        const float feedforwardGain = (flightModeFlags || launchControlActive) ? 0.0f : pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
            // no transition if feedForwardTransition == 0
            float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1;
            float feedForward = feedforwardGain * transition * pidSetpointDelta * pidFrequency;


            pidData[axis].F = feedForward;

        } else {
            pidData[axis].F = 0;
        }




// calculate rpm error component

pidData[axis].R = 0;

// end rpm error component
        // calculating the PID sum
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F + pidData[axis].R;

        float cubliSetpointGain = (pidProfile->setpointgain_cubli) / 1000.0f; /// change this


        {
            pidData[axis].Sum = pidSum;
            cubliSetpoint[axis] += pidSum * cubliSetpointGain * dT; // add a coefficient
        }
    }

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].R = 0;

            pidData[axis].Sum = 0;
        }
    } else if (zeroThrottleItermReset) {
        pidResetIterm();
    }
}

bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}



void pidSetAntiGravityState(bool newState)
{
    if (newState != antiGravityEnabled) {
        // reset the accelerator on state changes
        itermAccelerator = 0.0f;
    }
    antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    static unsigned int cutoffFreq;
    if (dynLpfFilter != DYN_LPF_NONE) {
        if (dynLpfCurveExpo > 0) {
            cutoffFreq = dynDtermLpfCutoffFreq(throttle, dynLpfMin, dynLpfMax, dynLpfCurveExpo);
        } else {
            cutoffFreq = fmax(dynThrottle(throttle) * dynLpfMax, dynLpfMin);
        }

         if (dynLpfFilter == DYN_LPF_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, dT));
            }
        } else if (dynLpfFilter == DYN_LPF_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
        }
    }
}
#endif

float dynDtermLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
    const float expof = expo / 10.0f;
    static float curve;
    curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}

float pidGetDT()
{
    return dT;
}

float pidGetPidFrequency()
{
    return pidFrequency;
}
