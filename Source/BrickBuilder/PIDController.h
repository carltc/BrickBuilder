// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PIDController.generated.h"

USTRUCT(BlueprintType)
struct FPIDState
{
	GENERATED_BODY()

	UPROPERTY(Transient) float LastError;
	UPROPERTY(Transient) float e_Accumulator;
};

USTRUCT(BlueprintType)
struct FPIDConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) float Kp;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float Ki;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float Kd;

	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MinOutput;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxOutput;
};

/**
 * 
 */
UCLASS(Blueprintable, BlueprintType)
class BRICKBUILDER_API UPIDController : public UObject
{
	GENERATED_BODY()

public:
	UPIDController();
	~UPIDController();

protected:
	UPROPERTY(VisibleInstanceOnly, BlueprintReadWrite, Category = PID)
	uint32 bRunCppPID : 1;

	UPROPERTY(Category = PID, EditDefaultsOnly, BlueprintReadWrite)
	FPIDConfig PIDConfig;

	UPROPERTY(Category = PID, VisibleInstanceOnly, BlueprintReadOnly)
	FPIDState PIDState;

	UPROPERTY(Category = PID, EditDefaultsOnly, BlueprintReadWrite)
	float DesiredValue = 0.f;
	
	UPROPERTY(Category = PID, EditDefaultsOnly, BlueprintReadWrite)
	float MaxValue = 0.f;
	
	UPROPERTY(Category = PID, EditDefaultsOnly, BlueprintReadWrite)
	float MinValue = 0.f;

public:
	UFUNCTION(BlueprintCallable)
	float UpdatePID(float CurrentValue, float DeltaSeconds);
	void ResetPID(float NewErrorValue, float NewAccumulatorValue = 0.f);

protected:
	void IncreaseDesiredValue (float desiredChange);

};
