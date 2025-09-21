// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#include "BallSimulatorComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "CollisionShape.h"

DECLARE_LOG_CATEGORY_EXTERN(LogBallSimulatorComponent, Log, All);
DEFINE_LOG_CATEGORY(LogBallSimulatorComponent);

// Sets default values for this component's properties
UBallSimulatorComponent::UBallSimulatorComponent()
{	
	PrimaryComponentTick.bCanEverTick = false;
}

// Called when the game starts
void UBallSimulatorComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UBallSimulatorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UBallSimulatorComponent::SimulateBallPhysics(
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
	const float StepInterval)
{	
	TRACE_CPUPROFILER_EVENT_SCOPE(UBallSimulatorComponent::SimulateBallPhysics);

	UWorld* World = WorldContextObject ? WorldContextObject->GetWorld() : nullptr;
	if (!World) return;

	CachedSnapshots.Reset();
	CachedBounces.Reset();
	BounceCount = 0;

	//CachedSnapshots.Reserve(SimulationSteps);
	const float InvMass = (BallMass > KINDA_SMALL_NUMBER) ? (1.0f / BallMass) : 0.01f;
	FCollisionShape CollisionShape = FCollisionShape::MakeSphere(BallRadius);	
	FVector pos = InitialPosition;
	FVector linearVelocity = InitialDirection * InitialSpeed;
	float speed = linearVelocity.Size();
	FVector direction = linearVelocity.GetSafeNormal();
	FVector spinAxis = InitialSpinAxis.GetSafeNormal();
	float spinSpeed = InitialSpinSpeed;
	float ballArea = PI * BallRadius * BallRadius;
	FVector angularVelocity = spinAxis * spinSpeed;
	FQuat Rotation = InitialRotation;
	float EndTime = 0.0f;
	SimulationStepInterval = StepInterval;
	FBallSnapshot snapshot;

	// 구체 관성 텐서 공식 (대각행렬 성분)
	// I = (2/5) * m * r^2  |  m = 0.5kg (질량)  |  r = 0.11m (반지름)
	// I = 2/5 * 0.5 * 0.11^2 = 0.00242 kg·m²
	float BaseInertia = 0.4f * BallMass * BallRadius * BallRadius;
	ScaledInertia = FVector(BaseInertia) * InertiaTensorScale;	
	InvInertiaTensor.X = (ScaledInertia.X > KINDA_SMALL_NUMBER) ? (1.0f / ScaledInertia.X) : 0.0f;
	InvInertiaTensor.Y = (ScaledInertia.Y > KINDA_SMALL_NUMBER) ? (1.0f / ScaledInertia.Y) : 0.0f;
	InvInertiaTensor.Z = (ScaledInertia.Z > KINDA_SMALL_NUMBER) ? (1.0f / ScaledInertia.Z) : 0.0f;

	// Initial Snapshot
	snapshot.Position = pos;
	snapshot.Direction = direction;
	//snapshot.SpinQuat = FQuat::Identity;
	snapshot.Rotation = InitialRotation;
	snapshot.Speed = speed;
	snapshot.SpinSpeed = spinSpeed;
	CachedSnapshots.Add(snapshot);	
	
	SpinFrictionScale = FMath::Clamp(1.0f - SpinFriction, 0.0f, 1.0f);

	for (int32 i = 1; i < SimulationSteps; ++i)
	{		
		// 위치, 속도 업데이트, 중력, 마찰력, 충돌 처리 (재귀)
		int collisionCount = HandleCollision(World, InvMass, CollisionShape, pos, linearVelocity, angularVelocity, StepInterval, 0);		
		direction = linearVelocity.GetSafeNormal();
				
		// 마그누스로 인한 횡력 적용
		if (spinSpeed > MinSpin)
		{			
			FVector magnusForce = FVector::CrossProduct(-direction * speed, angularVelocity) * SpinMagnusFactor;
			linearVelocity += magnusForce * StepInterval;
			//spinSpeed *= SpinFrictionScale;
		}
		
		// Damping 만으로 단순화 가능
		//if (speed > SMALL_NUMBER)              
		//{
		//  // 너무 느릴 땐 항력 생략,
		//	FVector drag = -0.5f * Rho * CD * ballArea * speed * linearVelocity; // F_D
		//	FVector accel = drag * InvMass;										 // a = F/m
		//	linearVelocity += accel * StepInterval;                              
		//}

		// Δt 동안 회전 (AngularVelocity 로 Rotation 업데이트)
		FQuat DeltaRotation = FQuat(spinAxis, spinSpeed * StepInterval);
		// 회전 누적, 새 회전 = Δ회전 * 이전 회전
		Rotation = Rotation * DeltaRotation * SpinToRotateMultiply;
		
		// 스냅샷 저장				
		snapshot.Position = pos;
		snapshot.Direction = linearVelocity.GetSafeNormal();
		snapshot.Rotation = Rotation;		
		snapshot.Speed = linearVelocity.Size();
		snapshot.SpinAxis = angularVelocity.GetSafeNormal();
		snapshot.SpinSpeed = angularVelocity.Size();
		CachedSnapshots.Add(snapshot);

		EndTime = (i + 1) * StepInterval;

		// 8. 시뮬레이션 정지 조건
		if (snapshot.Speed < MinSpeed && spinSpeed < MinSpin && BounceCount > MaxAllowedBounce)
		{
			break;
		}
	}

	// 시뮬레이션 종료 시간 저장
	SimulationEndTime = EndTime;
}

int UBallSimulatorComponent::HandleCollision(
	UWorld* World,
	const float InvMass,
	const FCollisionShape& CollisionShape,
	FVector& pos,
	FVector& linearVelocity,
	FVector& angularVelocity,	
	const float DeltaTime,
	int32 Depth)
{	
	TRACE_CPUPROFILER_EVENT_SCOPE(UBallSimulatorComponent::HandleCollision);

	// SubStep 정지 조건
	if (Depth > 10 || DeltaTime <= KINDA_SMALL_NUMBER) return Depth;	
	if(BounceCount > MaxAllowedBounce)
		return Depth;

	// 다음 속도 및 위치 계산 (오일러 적분)	
	// 속도 변화 업데이트 (중력가속도 적용)
	linearVelocity += GravityVector * DeltaTime;

	// 선형 감쇠 적용 (선형 감쇠는 Chaos에서 damping factor로 처리)	
	linearVelocity *= FMath::Clamp(1.0f - LinearDamping * DeltaTime, 0.0f, 1.0f);
	angularVelocity *= FMath::Clamp(1.0f - AngularDamping * DeltaTime, 0.0f, 1.0f);
	
	FVector nextPos = pos + linearVelocity * DeltaTime;
	
	FHitResult hit;
	bool bHit = World->SweepSingleByChannel(
		hit,
		pos,
		nextPos,
		FQuat::Identity,               // 회전 불필요
		ECC_WorldStatic,
		CollisionShape,
		FCollisionQueryParams(FName(TEXT("BallSimSweep")), true)
	);

	if (bHit && hit.bBlockingHit)
	{
		FBallBounce bounce;		
		bounce.Direction = linearVelocity.GetSafeNormal();
		bounce.Speed = linearVelocity.Size();
		bounce.Hit = hit;

		/*
		pos---------*----------------nextPos
					↑
					hit.Location(≈ Lerp(pos, nextPos, hit.Time))
		*/
		float hitTimeRatio = hit.Time;
		float timeToHit = DeltaTime * hitTimeRatio;		

		// 충돌 시점까지 이동
		pos += linearVelocity * timeToHit;

		float Friction = DefaultFriction;
		float Restitution = DefaultRestitution;
		
		// 충돌한 물리 재질에서 속성 가져오기
		if (hit.PhysMaterial.IsValid())
		{
			UPhysicalMaterial* PhysMat = hit.PhysMaterial.Get();

			// 커스텀 탄성/마찰 계수 가져오기 (기본 엔진 값 or 사용자 정의)
			// 예: PhysicalMaterial 에서 SurfaceType으로 분기하거나
			// 사용자 정의 UPhysicalMaterial 서브클래스에서 속성 직접 사용 가능
			// 기본 엔진 속성 (Material Editor에서 설정 가능)
			Friction = PhysMat->Friction;
			Restitution = PhysMat->Restitution;
		}

		// 마찰로 인한 감쇠. 마찰이 크면 접선 속도는 작아진다.
		//float frictionScale = FMath::Clamp(1.0f - Friction, 0.f, 1.0f);
		
		// 접촉점 P 에서 구 질량중심 C 로 가는 벡터 (접촉점 - 구 중심) r = P - C
		const FVector r = hit.ImpactPoint - pos;

		// 접촉점의 상대 속도 계산 (스핀 방향에 따른 속도 변화 적용)
		// 구의 접촉점 속도 = 구의 선형 속도 + (구의 각속도 × (접촉점 - 구의 중심))		
		const FVector ContactVelocity = linearVelocity + FVector::CrossProduct(angularVelocity, r);

		// 접촉점의 상대 속도를 히트 노멀 방향 으로 프로젝션해서 얻은 값 (vRel > 0 이면 멀어지는중)
		const float vRel = FVector::DotProduct(ContactVelocity, hit.ImpactNormal);
		bounce.vRel = vRel;

		// 임펄스 계산 분모 (질량, 관성 텐서 반영)
		// 1) r × n
		FVector rCrossN = FVector::CrossProduct(r, hit.ImpactNormal);
		// 2) I⁻¹ * (r × n)
		FVector inertiaTerm = InvInertiaTensor * rCrossN ;
		// 3) (I⁻¹ * (r × n)) × r
		FVector crossTerm = FVector::CrossProduct(inertiaTerm, r);
		// 최종 분모
		float denom = InvMass + FVector::DotProduct(hit.ImpactNormal, crossTerm);

		// 충돌 임펄스 크기 계산 (질량, 관성 텐서, 회전 마찰 등 생략, 단순화 가능)
		// 0 = 완전 비탄성, 1 = 완전 탄성.
		float impulseMagnitude = -(1.0f + Restitution) * vRel / denom;
		
		// 임펄스 크기(또는 상대 속도 vRel)가 충분히 크면 bounce, 아니라면 Rolling contact 
		bool bBounced = (impulseMagnitude > BounceThreshold);

		// (선택) 충격 임펄스 클램핑으로 과도한 임펄스 방지
		impulseMagnitude = FMath::Clamp(impulseMagnitude, 0.f, MaxAllowedImpulse);

		// 임펄스 벡터 (법선 방향으로 impulseMagnitude 곱)
		const FVector impulse = impulseMagnitude * hit.ImpactNormal;		
		bounce.LinearDelta = impulse * InvMass;
		// 선형 속도 업데이트  v = v + impulse * InvMass
		linearVelocity += impulse * InvMass;

		// 각속도 업데이트
		FVector angularImpulse = FVector::CrossProduct(r, impulse);
		
		FVector angularDelta = InvInertiaTensor * angularImpulse * BouncedSpinMultiply;
		angularVelocity += angularDelta;
		bounce.AngularDelta = angularDelta;

		// 쿠롱 마찰 임펄스 계산 (접선 임펄스)
		FVector tangentVelocity = ContactVelocity - vRel * hit.ImpactNormal;
		if (!tangentVelocity.IsNearlyZero())
		{
			FVector t = tangentVelocity.GetSafeNormal();

			// 접선 임펄스 분모 (같은 denom 사용 가능)
			float tangentImpulse = -FVector::DotProduct(ContactVelocity, t) / denom;

			// 마찰 임펄스 최대값 (μ * 정반사 임펄스)
			float maxFrictionImpulse = impulseMagnitude * Friction;

			// 클램핑
			tangentImpulse = FMath::Clamp(tangentImpulse, -maxFrictionImpulse, maxFrictionImpulse);

			FVector frictionImpulse = tangentImpulse * t;
			bounce.FrictionDelta = frictionImpulse * InvMass;
			// 속도 업데이트
			linearVelocity += frictionImpulse * InvMass;

			FVector angularFrictionImpulse = FVector::CrossProduct(r, frictionImpulse);
			FVector angularDeltaFriction = InvInertiaTensor * angularFrictionImpulse;
			angularVelocity += angularDeltaFriction;
		}

		// 스핀 감쇠 적용		
		angularVelocity *= SpinFrictionScale;		
		
		if(bBounced)
		{
			BounceCount++;
			bounce.SnapshotIndex = CachedSnapshots.Num() - 1;
			bounce.BouncedDirection = linearVelocity.GetSafeNormal();		
			bounce.BouncedSpeed = linearVelocity.Size();				
			CachedBounces.Add(bounce);
		}

		const float PenetrationDepthThreshold = 0.1f;     // 끼인 것으로 판단할 최소 깊이
		const float SmallMargin = 0.01f;                  // 밀어낼 여유 마진
		const float PenetrationVelocityDamping = 0.5f;    // 감속 계수
		bounce.PenetrationDepth = hit.PenetrationDepth;

		bool bIsStuck = hit.bStartPenetrating || hit.PenetrationDepth > PenetrationDepthThreshold;
		if (bIsStuck)
		{
			// 침투 깊이만큼 푸시백
			const FVector PenetrationDirection = hit.Normal.IsNearlyZero() ? FVector::UpVector : hit.Normal;
			const float PushBack = hit.PenetrationDepth + KINDA_SMALL_NUMBER; // 소량 여유 마진 추가
			pos += PenetrationDirection * PushBack;

			//// 속도 감쇠 (너무 튀지 않도록)
			//linearVelocity *= 0.5f;
			//angularVelocity *= 0.5f;

			// 디버그용 출력 또는 로그
			UE_LOG(LogBallSimulatorComponent, Verbose, TEXT("Penetration resolved: depth = %.3f, push = %s"), hit.PenetrationDepth, *PenetrationDirection.ToString());
		}

		// 남은 시간으로 재귀 호출
		float remainingTime = DeltaTime - timeToHit;
		return HandleCollision(World, InvMass, CollisionShape, pos, linearVelocity, angularVelocity, remainingTime, Depth + 1);
	}
	else
	{
		pos = nextPos;
		return Depth;
	}
}

void UBallSimulatorComponent::ConvertSnapshotsToBezierSpline(
	const TArray<FBallSnapshot>& Snapshots,
	USplineComponent* SplineComponent) const
{
	if (!SplineComponent || Snapshots.Num() < 2)
	{
		UE_LOG(LogBallSimulatorComponent, Warning, TEXT("Invalid SplineComponent or insufficient snapshot count"));
		return;
	}

	SplineComponent->ClearSplinePoints(false);

	for (int32 i = 0; i < Snapshots.Num(); ++i)
	{
		FVector Pos = Snapshots[i].Position;

		// 접선 벡터 방향 설정 (길이 50은 곡률 조절용, 필요시 조절 가능)
		FVector Tangent = Snapshots[i].Direction.GetSafeNormal() * SplineTangentLengh;

		// Spline에 포인트 추가
		SplineComponent->AddSplinePoint(Pos, ESplineCoordinateSpace::World, false);
		SplineComponent->SetTangentAtSplinePoint(i, Tangent, ESplineCoordinateSpace::World, false);
		SplineComponent->SetSplinePointType(i, ESplinePointType::Curve, false);

		//FSplinePoint Point(i, Pos, Tangent, Tangent, ESplinePointType::Type::Curve);
		//SplineComponent->AddPoint(Point, false);
	}

	SplineComponent->UpdateSpline();
}

float UBallSimulatorComponent::GetBallSpeedAtTime(float playbackTime) const
{
	if (CachedSnapshots.Num() < 2 || SimulationStepInterval <= 0.f)
	{
		return 0.f;
	}

	float TotalDuration = (CachedSnapshots.Num() - 1) * SimulationStepInterval;
	float ClampedTime = FMath::Clamp(playbackTime, 0.f, TotalDuration);

	int32 IndexA = FMath::Clamp(FMath::FloorToInt(ClampedTime / SimulationStepInterval), 0, CachedSnapshots.Num() - 2);
	int32 IndexB = IndexA + 1;
	float LocalAlpha = (ClampedTime - IndexA * SimulationStepInterval) / SimulationStepInterval;

	float SpeedA = CachedSnapshots[IndexA].Speed;
	float SpeedB = CachedSnapshots[IndexB].Speed;

	return FMath::Lerp(SpeedA, SpeedB, LocalAlpha);
}

void UBallSimulatorComponent::GetBallVelocityAtTime(float playbackTime,
	FVector& LinearVelocity,
	FVector& AngularVelocity) const
{
	if (CachedSnapshots.Num() < 2 || SimulationStepInterval <= 0.f)
	{
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
		return;
	}

	float TotalDuration = (CachedSnapshots.Num() - 1) * SimulationStepInterval;
	float ClampedTime = FMath::Clamp(playbackTime, 0.f, TotalDuration);

	int32 IndexA = FMath::Clamp(FMath::FloorToInt(ClampedTime / SimulationStepInterval), 0, CachedSnapshots.Num() - 2);
	int32 IndexB = IndexA + 1;
	float LocalAlpha = (ClampedTime - IndexA * SimulationStepInterval) / SimulationStepInterval;

	// 방향 보간
	FVector DirA = CachedSnapshots[IndexA].Direction;
	FVector DirB = CachedSnapshots[IndexB].Direction;
	FVector InterpDir = FMath::Lerp(DirA, DirB, LocalAlpha).GetSafeNormal();

	// 속도(스칼라) 보간
	float SpeedA = CachedSnapshots[IndexA].Speed;
	float SpeedB = CachedSnapshots[IndexB].Speed;
	float InterpSpeed = FMath::Lerp(SpeedA, SpeedB, LocalAlpha);

	// 실제 velocity = 방향 * 속도
	LinearVelocity = InterpDir * InterpSpeed;
	
	// TBD - 충돌 후 회전 변화가 큰 경우를 고려해야 함
	const FVector SpinAxisA = CachedSnapshots[IndexA].SpinAxis;
	const float SpinSpeedA = CachedSnapshots[IndexA].SpinSpeed;
	const FVector AngularVelocityA = SpinAxisA * SpinSpeedA;
	const FVector SpinAxisB = CachedSnapshots[IndexB].SpinAxis;
	const float SpinSpeedB = CachedSnapshots[IndexB].SpinSpeed;
	const FVector AngularVelocityB = SpinAxisB * SpinSpeedB;

	// 스핀 보간
	AngularVelocity = FMath::Lerp(AngularVelocityA, AngularVelocityB, LocalAlpha);
}

bool UBallSimulatorComponent::GetBallPositionAndRotationAtSplineTime(
	const USplineComponent* SplineComponent,	
	float playbackTime,
	FVector& OutPosition,
	FQuat& OutRotation) const
{
	OutPosition = FVector::ZeroVector;
	OutRotation = FQuat::Identity;

	if (!SplineComponent || CachedSnapshots.Num() < 2 || SimulationStepInterval <= 0.f)
	{
		return false;
	}
	
	float TotalDuration = (SplineComponent->GetNumberOfSplinePoints() - 1) * SimulationStepInterval;
	float ClampedTime = FMath::Clamp(playbackTime, 0.f, TotalDuration);
	float Alpha = ClampedTime / TotalDuration;

	// 위치 계산: 스플라인 기반
	float SplineLength = SplineComponent->GetSplineLength();
	float DistanceOnSpline = SplineLength * Alpha;
	OutPosition = SplineComponent->GetLocationAtDistanceAlongSpline(DistanceOnSpline, ESplineCoordinateSpace::World);

	// 회전 보간 계산: Snapshot 기반
	int32 IndexA = FMath::Clamp(FMath::FloorToInt(ClampedTime / SimulationStepInterval), 0, CachedSnapshots.Num() - 2);
	int32 IndexB = IndexA + 1;
	float LocalAlpha = (ClampedTime - IndexA * SimulationStepInterval) / SimulationStepInterval;

	const FQuat& RotA = CachedSnapshots[IndexA].Rotation;
	const FQuat& RotB = CachedSnapshots[IndexB].Rotation;

	OutRotation = FQuat::Slerp(RotA, RotB, LocalAlpha).GetNormalized();

	return true;
}

void UBallSimulatorComponent::GetBallPositionAndRotationAtTime(
	float playbackTime,
	FVector& OutPosition,
	FRotator& OutRotation,
	int32& OutIndexA, int32& OutIndexB) const
{
	if (CachedSnapshots.Num() < 2 || SimulationStepInterval <= 0.f)
	{
		OutPosition = FVector::ZeroVector;
		OutRotation = FRotator::ZeroRotator;
		return;
	}

	float TotalDuration = (CachedSnapshots.Num() - 1) * SimulationStepInterval;
	float ClampedTime = FMath::Clamp(playbackTime, 0.f, TotalDuration);
	int32 IndexA = FMath::Clamp(FMath::FloorToInt(ClampedTime / SimulationStepInterval), 0, CachedSnapshots.Num() - 2);
	int32 IndexB = IndexA + 1;
	float LocalAlpha = (ClampedTime - IndexA * SimulationStepInterval) / SimulationStepInterval;

	OutIndexA = IndexA;
	OutIndexB = IndexB;

	// 위치 보간
	FVector PosA = CachedSnapshots[IndexA].Position;
	FVector PosB = CachedSnapshots[IndexB].Position;
	OutPosition = FMath::Lerp(PosA, PosB, LocalAlpha);

	// 회전 보간 (Quaternion 사용)
	const FQuat& RotA = CachedSnapshots[IndexA].Rotation;
	const FQuat& RotB = CachedSnapshots[IndexB].Rotation;
	OutRotation = FQuat::Slerp(RotA, RotB, LocalAlpha).Rotator();
}