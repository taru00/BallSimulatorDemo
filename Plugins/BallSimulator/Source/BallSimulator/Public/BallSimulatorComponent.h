// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SplineComponent.h"
#include "BallSimulatorComponent.generated.h"

DECLARE_CYCLE_STAT(TEXT("Ballistic Physics Simulator"), STAT_BallPhysicsSimulation, STATGROUP_Game);
DECLARE_CYCLE_STAT(TEXT("HandleCollision"), STAT_HandleCollision, STATGROUP_Game);

USTRUCT(BlueprintType)
struct FBallBounce
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    int SnapshotIndex;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector Direction;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float Speed;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector BouncedDirection;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float BouncedSpeed;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bWasStuck;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsSliding;
    
    UPROPERTY(BlueprintReadOnly)
    FHitResult Hit;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector StartPos;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector ImpactPoint;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float hitTimeRatio;

    // 이번 충돌에 대한 반사 후 추가 충돌이 없을때 사용될 NextPos 후보
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector NextPos;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float timeToHit;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float RemainingTime;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float vRel;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector LinearImpulse;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector AngularDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector FrictionDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float PenetrationDepth;
};

USTRUCT(BlueprintType)
struct FBallSnapshot
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    float Time;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FVector Position;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FQuat Rotation;

    // Direction * Speed
    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    //FVector LinearVeloticy;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FVector Direction;

    // 축구 기준 2000 cm/s ~ 4000 cm/s or 70 km/h ~ 145 km/h  (100 cm/s = 3.6 km/h)
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    float Speed;
    
    // SpinAxis * SpinSpeed
    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    //FVector AngularVelocity;
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    int hitCount;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    int BounceIndex;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FVector SpinAxis;

    // 축구 회전 킥 기준 20~90 rad/s
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    float SpinSpeed;
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class BALLSIMULATOR_API UBallSimulatorComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UBallSimulatorComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UFUNCTION(BlueprintCallable, Category = "Ballistic Physics Simulator", meta = (WorldContext = "Outer"))    
    void SimulateBallPhysics(        
        const UObject* WorldContextObject,
        const float BallMass,
        const float BallRadius,
        const FVector& InitialPosition,
        const FQuat& InitialRotation,
        const FVector& InitialDirection,            
        const float InitialSpeed,
        const FVector& InitialSpinAxis,
        const float InitialSpinSpeed,        
        const int32 SimulationSteps,
        const float StepInterval);

    int HandleCollision(
        UWorld* World,
        const float InvMass,
        const FCollisionShape& CollisionShape,
        FVector& pos,        
        FVector& LinearVelocity,
        FVector& AngularVelocity,        
        const float DeltaTime,
        int32 Depth);

    //bool ResolvePenetration(const FVector& ProposedAdjustment, const FHitResult& Hit, const FQuat& NewRotationQuat);

    UFUNCTION(BlueprintCallable, Category = "Ballistic Physics Simulator")
    void ConvertSnapshotsToBezierSpline(const TArray<FBallSnapshot>& Snapshots, USplineComponent* SplineComponent) const;    

    UFUNCTION(BlueprintCallable, Category = "Ballistic Physics Simulator")
    void GetBallPositionAndRotationAtTime(
        float playbackTime,
        FVector& OutPosition,
        FRotator& OutRotation,
		int32& IndexA, int32& IndexB) const;

    UFUNCTION(BlueprintCallable, Category = "Ballistic Physics Simulator")
    bool GetBallPositionAndRotationAtSplineTime(
        const USplineComponent* SplineComponent,        
        float playbackTime,
        FVector& OutPosition,
        FQuat& OutRotation) const;
  
    UFUNCTION(BlueprintCallable, Category = "Ballistic Physics Simulator")
    float GetBallSpeedAtTime(float playbackTime) const;

    UFUNCTION(BlueprintCallable, Category = "Ballistic Physics Simulator")    
    void GetBallVelocityAtTime(float playbackTime, FVector& LinearVelocity,
        FVector& AngularVeloticy) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float MinSpeed = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float MinSpinForMagnus = 10.f;

    // 중력 기본값 -980 cm/s²
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    FVector GravityVector = FVector(0, 0, -1200.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float SpinMagnusFactor = 0.005f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float LinearDamping = 0.02f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float AngularDamping = 0.02f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    TArray<FBallSnapshot> CachedSnapshots;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    TArray<FBallBounce> CachedHits;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    TArray<FBallBounce> CachedBounces;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float SimulationStepInterval = 0.0f; 

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")    
    float PullBackDistance = 0.01f;

    // 축구공 반지름 : 약 11 cm
    //UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    //float BallRadius = 11.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float SimulationEndTime = 0.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float DefaultRestitution = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float DefaultFriction = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float SpinFriction = 0.05f;
    
    float SpinFrictionScale = 0.95f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    FVector InertiaTensorScale = FVector(0.5f, 0.5f, 0.5f);

    FVector InvInertiaTensor;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    FVector ScaledInertia = FVector(0.f, 0.f, 0.f);

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float BouncedSpinMultiply = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float SpinToRotateMultiply = 3.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float MaxAllowedImpulse = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    int MaxAllowedBounce = 5;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    int BounceCount = 0;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float BounceThreshold = 50.f;   

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    bool bBounceAngleAffectsFriction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float MinFrictionFraction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
	float MaxAllowedSpeed = 10000.f;

    // Damping 만으로 단순화 가능
    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    //float CD = 0.5f;           // 항력 계수(튜닝)

    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    //float Rho = 0.000001225f;   // 공기 밀도 1.225 kg·m⁻³  ( 1.225f / 1e6f kg·cm⁻³ ) 

    static constexpr int MaxAllowedSimulationStep = 1000;
    static constexpr float SplineTangentLengh = 50.f;        
    
};