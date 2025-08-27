// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "Components/SphereComponent.h"
#include "BallSimulatorComponent.h"
//#include "Chaos/ChaosSolver.h"
#include "BallSimulatorActor.generated.h"

UCLASS()
class BALLSIMULATOR_API ABallSimulatorActor : public AActor
{
    GENERATED_BODY()
public:
    ABallSimulatorActor();
    void Tick(float DeltaTime);

    virtual void BeginPlay();

    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Ball Physics Simulator")
    void SimulateBallPhysics(
        UObject* WorldContextObject,
        const FVector& InitialPosition,
        const FVector& InitialVelocity,
        const FVector& InitialSpin,
        float SphereCollisionRadius,
        float SimulationTime,
        float StepInterval,
        TArray<FBallSnapshot>& OutSnapshots);

    void InitializeSimPhysicsScene();
    void StepSimPhysicsScene(float DeltaTime);
    

protected:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    USplineComponent* SplineComp;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    UBallSimulatorComponent* BallSimulatorComp;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    USphereComponent* SphereCollisionComp;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    UStaticMeshComponent* BallMeshComp;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    USphereComponent* SimSphereComp;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsPlayingAnimation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float PlaybackTime;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    UWorld* SimWorld;

    // 별도 Physics 씬 멤버 변수
    //TUniquePtr<FPhysScene_Chaos> SimPhysicsScene;
    //TUniquePtr<FChaosSolver> SimSolver;
};
