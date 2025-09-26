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
    float Spin;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector AngularVelocity;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector BouncedDirection;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float BouncedSpeed;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float BouncedSpin;   

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector BouncedAngularVelocity;
    
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
    FVector ImpactNormal;

    // 이번 충돌에 대한 반사 후 추가 충돌이 없을때 사용될 NextPos 후보
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector NextPos;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float TimeToBeforeHit;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float RemainingTime;

    // 접촉점의 상대 속도 ContactVelocity를 히트 노멀 방향 으로 프로젝션해서 얻은 값
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float vRel;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector NormalImpulse;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector FrictionImpulse;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector LinearImpulse;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector AngularDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float AngularDeltaSize;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector FrictionDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float PenetrationDepth;
};

USTRUCT(BlueprintType)
struct FBallSnapshot
{
    GENERATED_BODY()

	// 디버깅 편의를 위해 저장된 시간값 (고정 프레임율 이므로 시간 간격은 일정함)
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

    // spin 벡터를 회전 쿼터니언으로 변환하는 함수    
    void ApplySpinToRotation(const FVector& InAngularDelta, FQuat& OutRotation) const;

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

	// 최소 속도 이하로 떨어지면 시뮬레이션 종료
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float MinSpeed = 1.0f;

	// 최소 스핀값 이하로 떨어지면 마그누스 효과 미적용
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float MinSpinForMagnus = 10.f;

    // 중력 기본값 -980 cm/s²
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    FVector GravityVector = FVector(0, 0, -980.0f);

    // 강한 스핀에 의한 횡력 조절 (감아차기 효과)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float SpinMagnusFactor = 0.01f;

	// 이동에 대한 저항 계수 (0.05 이면 초당 5% 감쇠)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float LinearDamping = 0.05f;

	// 회전에 대한 저항 계수 (0.1 이면 초당 10% 감쇠)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float AngularDamping = 0.1f;

	// 바운스시 회전 속도 감쇠 조절 (0.7 이면 70% 유지됨)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float BouncedSpinMultiplier = 0.65f;

    UPROPERTY(BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    TArray<FBallSnapshot> CachedSnapshots;
	
    UPROPERTY(BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    TArray<FBallBounce> CachedHits;

    UPROPERTY(BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    TArray<FBallBounce> CachedBounces;
    
	// 시뮬레이션 스텝 시간 간격 (0.033 = 30Hz)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float SimulationStepInterval = 0.033f; 

    // 축구공 반지름 : 약 11 cm
    //UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    //float BallRadius = 11.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float SimulationEndTime = 0.0f;
    
	// 탄성 1.0 에 가까워 질수록 완전 탄성 운동에 가까워짐 
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float DefaultRestitution = 0.7f;

    // 이동에 대한 감속 계수	
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float DefaultFriction = 0.1f;

	// 접촉시 회전 감속 계수
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float SpinFriction = 0.1f;
    
    // 감속 계수에 의해서 자동 계산되는 내부 변수
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float SpinFrictionScale = 0.9f;
    
	// 관성 모멘트 텐서 스케일 (0.4 ~ 0.5 정도가 현실적인 축구공에 근접)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    FVector InertiaTensorScale = FVector(0.5f, 0.5f, 0.5f);

	// InertiaTensorScale 값에 의해서 자동 계산되는 내부 변수
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    FVector InvInertiaTensor;

	// InertiaTensorScale 값에 의해서 자동 계산되는 내부 변수
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    FVector ScaledInertia = FVector(0.f, 0.f, 0.f);   

	// 접촉 지점 및 마찰에 의해 발생되는 회전력 튜닝 (바운스 및 슬라이딩 에서 회전 변화량에 곱해짐)
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float SpinToRotateMultiply = 1.0f;

	// 충돌시 최대 선형 임펄스 제한
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    float MaxAllowedImpulse = 1000.f;

	// 최대 바운스 횟수 제한 (시뮬레이션 정지 조건)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ballistic Physics Simulator")
    int MaxAllowedBounce = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    int BounceCount = 0;

	// 법선 방향으로 프로젝트된 임펄스가 이 값 이하인 경우 접촉 상태에서의 슬라이딩 (Rolling Contact) 으로 간주
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float BounceThreshold = 10.f;   

	// 최대 허용 속도 
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
	float MaxAllowedSpeed = 10000.f;

    // Damping 만으로 단순화 가능
    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    //float CD = 0.5f;           // 항력 계수(튜닝)

    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    //float Rho = 0.000001225f;   // 공기 밀도 1.225 kg·m⁻³  ( 1.225f / 1e6f kg·cm⁻³ ) 

	// 슬라이딩 접촉 상태 확인용 내부 변수
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    float PreviousHitTime;
    
    // 슬라이딩 접촉 상태 확인용 내부 변수
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ballistic Physics Simulator")
    FVector PreviousHitNormal;

    static constexpr int MaxAllowedSimulationStep = 1000;
    static constexpr float SplineTangentLengh = 50.f;        
    
};