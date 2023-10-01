// Fill out your copyright notice in the Description page of Project Settings.


#include "PIDController.h"

UPIDController::UPIDController()
{
}

UPIDController::~UPIDController()
{
}

// ------------------------------------------------------------------------
// Method:    UpdatePID
//            Run the PIDController loop, update state values, and compute
//            loop CV output value.
//            (See additional comments in PID.h)
// ------------------------------------------------------------------------
float UPIDController::UpdatePID(float CurrentValue, float DeltaSeconds)
{
    float e = DesiredValue - CurrentValue;

    check(DeltaSeconds > 0.f);

    float Output = 0.f;

    // Proportional term
    Output += PIDConfig.Kp * e;

    // Integral term
    PIDState.e_Accumulator += e * DeltaSeconds;
    Output += PIDConfig.Ki * PIDState.e_Accumulator;

    // Derivative term 
    float e_Rate = (e - PIDState.LastError) / DeltaSeconds;
    Output += PIDConfig.Kd * e_Rate;
    PIDState.LastError = e;

    return FMath::Clamp(Output, PIDConfig.MinOutput, PIDConfig.MaxOutput);
}

// ------------------------------------------------------------------------
// Method:    ResetPID
//            Resets PID State derivative and integral intermediates.
//            This is useful to avoid loop ill behavior across setpoint
//            (SP) discontinuities.
//            (See additional comments in PID.h)
// ------------------------------------------------------------------------
void UPIDController::ResetPID(float NewErrorValue, float NewAccumulatorValue)
{
    PIDState.LastError = NewErrorValue;
    PIDState.e_Accumulator = NewAccumulatorValue;
}

void UPIDController::IncreaseDesiredValue(float desiredChange)
{
    DesiredValue += desiredChange;
    DesiredValue = FMath::Clamp(DesiredValue, MinValue, MaxValue);
}
