// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#pragma once

#include "CoreMinimal.h"
#include "FunctionalTest.h"
#include "Components/SplineComponent.h"
#include "BallSimulatorComponent.h"
#include "BallSimulatorTest.generated.h"

UCLASS()
class BALLSIMULATORDEMO_API ABallSimulatorTest : public AFunctionalTest
{
	GENERATED_BODY()
public:
    ABallSimulatorTest();

protected:
    virtual void StartTest() override;
    virtual void PrepareTest() override;

private:
    UPROPERTY()
    USplineComponent* SplineComp;

    UPROPERTY()
    UBallSimulatorComponent* BallSimulatorComp;

    void SetupSpline();

    void RunSimulationTests();
    void RunInterpolationTests();
};
