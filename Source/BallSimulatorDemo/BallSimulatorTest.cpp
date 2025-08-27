// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#include "BallSimulatorTest.h"
#include "DrawDebugHelpers.h"
#include "Misc/ScopeLock.h"
#include "ProfilingDebugging/ScopedTimers.h"  // SCOPE_CYCLE_COUNTER
#include "Trace/Trace.h"                     // TRACE_CPUPROFILER_EVENT_SCOPE
#include "BallSimulator.h"

// Sets default values
ABallSimulatorTest::ABallSimulatorTest()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    SplineComp = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));
    BallSimulatorComp = CreateDefaultSubobject<UBallSimulatorComponent>(TEXT("BallSimulatorComponent"));
    RootComponent = SplineComp;
}

void ABallSimulatorTest::PrepareTest()
{
    Super::PrepareTest();    
}

void ABallSimulatorTest::StartTest()
{
    Super::StartTest();

    SetupSpline();

    TArray<FBallSnapshot> Snapshots;
    FVector InitPos(0, 0, 300);
    FVector InitDir(0.5, 0, 0.5);
    FVector IntAngVel(0, 0, 200);    
    //FVector InitSpin(0, 0, 500);
    FVector InitSpinAxis(0, 0, 1);
    float Speed = 1500.f;
    float SpinSpeed = 500.f;
    FQuat InitRot(0,0,0, 1);
    float Mass = 1.f;
    float Radius = 11.0f;
    BallSimulatorComp->SimulateBallPhysics(GetWorld(), Mass, Radius, InitPos, InitRot, InitDir, Speed, InitSpinAxis, SpinSpeed, 100, 0.016f);

    if (Snapshots.Num() == 0)
    {
        FinishTest(EFunctionalTestResult::Failed, TEXT("No snapshots generated."));
        return;
    }

    RunSimulationTests();
    RunInterpolationTests();

    FinishTest(EFunctionalTestResult::Succeeded, TEXT("All ball sim tests passed."));
}

void ABallSimulatorTest::SetupSpline()
{
    SplineComp->ClearSplinePoints();

    for (int32 i = 0; i <= 10; ++i)
    {
        FVector Point(i * 100.f, 0.f, 200.f);
        SplineComp->AddSplinePoint(Point, ESplineCoordinateSpace::World);
    }

    SplineComp->UpdateSpline();
}

void ABallSimulatorTest::RunSimulationTests()
{
    // DeltaTime == 0    
    FVector Pos;
    FQuat Rot;
    BallSimulatorComp->GetBallPositionAndRotationAtSplineTime(SplineComp, 1.0f, Pos, Rot);
    check(Pos == FVector::ZeroVector);

    // Spline == nullptr
    BallSimulatorComp->GetBallPositionAndRotationAtSplineTime(nullptr, 1.0f, Pos, Rot);
    check(Pos == FVector::ZeroVector);
}

void ABallSimulatorTest::RunInterpolationTests()
{
    float SimDuration = BallSimulatorComp->SimulationEndTime;
    FVector Pos1, Pos2, Pos3;
    FQuat Rot1, Rot2, Rot3;

    BallSimulatorComp->GetBallPositionAndRotationAtSplineTime(SplineComp, -1.0f, Pos1, Rot1);  // < 0
    BallSimulatorComp->GetBallPositionAndRotationAtSplineTime(SplineComp, SimDuration + 1.0f, Pos2, Rot2);     // > Max
    BallSimulatorComp->GetBallPositionAndRotationAtSplineTime(SplineComp, SimDuration * 0.5f, Pos3, Rot3);  // 중간

    check(!Pos1.IsNearlyZero());
    check(!Pos2.IsNearlyZero());
    check(!Pos3.IsNearlyZero());
}

