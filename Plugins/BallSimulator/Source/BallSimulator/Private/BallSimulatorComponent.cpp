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
	if(!World)
	{
		return;
	}

	CachedSnapshots.Reset();
	CachedBounces.Reset();
	CachedHits.Reset();
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
	snapshot.Time = 0.0f;
	snapshot.Position = pos;
	snapshot.Direction = direction;
	//snapshot.SpinQuat = FQuat::Identity;
	snapshot.Rotation = InitialRotation;
	snapshot.Speed = speed;
	snapshot.SpinSpeed = spinSpeed;
	CachedSnapshots.Add(snapshot);	
	
	//SpinFrictionScale = FMath::Clamp(1.0f - SpinFriction, 0.0f, 1.0f);

	for (int32 i = 1; i < SimulationSteps; ++i)
	{		
		// 위치, 속도 업데이트, 중력, 마찰력, 충돌 처리 (재귀)
		int hitCount = HandleCollision(World, InvMass, CollisionShape, pos, linearVelocity, angularVelocity, StepInterval, 0);		
		if(hitCount > 0) 
		{
			// 충돌 SubStep 처리 후 남은 현재 Step의 최종 바운스만 기록
			BounceCount++;
			const FBallBounce& BallBounce = CachedHits.Last();
			CachedBounces.Add(BallBounce);
			
			snapshot.BounceIndex = CachedHits.Num() - 1;			

			if(BallBounce.bIsSliding)
			{
				// 시뮬레이션 정지, 물리 상태로 전환 필요
				SimulationEndTime = i * StepInterval;
				break;
			}
		}

		// 새로운 속도 및 방향, 스냅샷 저장용
		direction = linearVelocity.GetSafeNormal();
		speed = linearVelocity.Size();
		// 바운스로 인해 축이 변경될 수 있음, 스냅샷 저장용
		spinAxis = angularVelocity.GetSafeNormal();
		spinSpeed = angularVelocity.Size();

		// 마그누스로 인한 횡력 적용
		if (spinSpeed > MinSpinForMagnus)
		{			
			FVector magnusForce = FVector::CrossProduct(-direction * speed, angularVelocity) * SpinMagnusFactor;
			linearVelocity += magnusForce * StepInterval;
			
			// 회전 속도의 감쇠: 시간 간격에 따라 회전 속도 감소
			//spinSpeed *= FMath::Pow(SpinFrictionScale, StepInterval);
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

		// AngularDamping 과 SpinToRotateMultiply 는 HandleCollision 에서 적용
		// 회전 누적
		Rotation += DeltaRotation;
		
		// 스냅샷 저장			
		snapshot.Time = i * StepInterval;
		snapshot.Position = pos;
		snapshot.Direction = direction;
		snapshot.Rotation = Rotation;		
		snapshot.Speed = speed;
		snapshot.SpinAxis = spinAxis;
		snapshot.SpinSpeed = spinSpeed;
		snapshot.hitCount = hitCount;
		CachedSnapshots.Add(snapshot);
		
		// 시뮬레이션 정지, 물리 상태로 전환 필요
		if (snapshot.Speed < MinSpeed && spinSpeed < MinSpinForMagnus && BounceCount > MaxAllowedBounce)
		{
			SimulationEndTime = i * StepInterval;
			break;
		}
	}
	
	// 시뮬레이션 종료 시간 저장
	SimulationEndTime = SimulationSteps * StepInterval;
}

#if 0 
bool UBallSimulatorComponent::ResolvePenetration(const FCollisionShape& CollisionShape, const FVector& ProposedAdjustment, const FHitResult& Hit, const FQuat& NewRotationQuat)
{
	// SceneComponent can't be in penetration, so this function really only applies to PrimitiveComponent.
	const FVector Adjustment = ConstrainDirectionToPlane(ProposedAdjustment);
	if (Adjustment.IsZero())
	{
		return false;
	}

	TRACE_CPUPROFILER_EVENT_SCOPE(UBallSimulatorComponent::ResolvePenetration);

	// We really want to make sure that precision differences or differences between the overlap test and sweep tests don't put us into another overlap,
	// so make the overlap test a bit more restrictive.
	const float OverlapInflation = MovementComponentCVars::PenetrationOverlapCheckInflation;
	bool bEncroached = OverlapTest(Hit.TraceStart + Adjustment, NewRotationQuat, ECollisionChannel::ECC_WorldStatic, CollisionShape, ActorOwner);
	if (!bEncroached)
	{
		// Move without sweeping.
		MoveUpdatedComponent(Adjustment, NewRotationQuat, false, nullptr, ETeleportType::TeleportPhysics);
		UE_LOG(LogMovement, Verbose, TEXT("ResolvePenetration:   teleport by %s"), *Adjustment.ToString());
		return true;
	}
	else
	{
		// Disable MOVECOMP_NeverIgnoreBlockingOverlaps if it is enabled, otherwise we wouldn't be able to sweep out of the object to fix the penetration.
		//TGuardValue<EMoveComponentFlags> ScopedFlagRestore(MoveComponentFlags, EMoveComponentFlags(MoveComponentFlags & (~MOVECOMP_NeverIgnoreBlockingOverlaps)));

		// Try sweeping as far as possible...
		FHitResult SweepOutHit(1.f);
		bool bMoved = MoveUpdatedComponent(Adjustment, NewRotationQuat, true, &SweepOutHit, ETeleportType::TeleportPhysics);
		UE_LOG(LogTemp, Verbose, TEXT("ResolvePenetration:   sweep by %s (success = %d)"), *Adjustment.ToString(), bMoved);

		// Still stuck?
		if (!bMoved && SweepOutHit.bStartPenetrating)
		{
			// Combine two MTD results to get a new direction that gets out of multiple surfaces.
			const FVector SecondMTD = GetPenetrationAdjustment(SweepOutHit);
			const FVector CombinedMTD = Adjustment + SecondMTD;
			if (SecondMTD != Adjustment && !CombinedMTD.IsZero())
			{
				bMoved = MoveUpdatedComponent(CombinedMTD, NewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
				UE_LOG(LogTemp, Verbose, TEXT("ResolvePenetration:   sweep by %s (MTD combo success = %d)"), *CombinedMTD.ToString(), bMoved);
			}
		}

		// Still stuck?
		if (!bMoved)
		{
			// Try moving the proposed adjustment plus the attempted move direction. This can sometimes get out of penetrations with multiple objects
			const FVector MoveDelta = ConstrainDirectionToPlane(Hit.TraceEnd - Hit.TraceStart);
			if (!MoveDelta.IsZero())
			{
				bMoved = MoveUpdatedComponent(Adjustment + MoveDelta, NewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
				UE_LOG(LogTemp, Verbose, TEXT("ResolvePenetration:   sweep by %s (adjusted attempt success = %d)"), *(Adjustment + MoveDelta).ToString(), bMoved);

				// Finally, try the original move without MTD adjustments, but allowing depenetration along the MTD normal.
				// This was blocked because MOVECOMP_NeverIgnoreBlockingOverlaps was true for the original move to try a better depenetration normal, but we might be running in to other geometry in the attempt.
				// This won't necessarily get us all the way out of penetration, but can in some cases and does make progress in exiting the penetration.
				if (!bMoved && FVector::DotProduct(MoveDelta, Adjustment) > 0.f)
				{
					bMoved = MoveUpdatedComponent(MoveDelta, NewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
					UE_LOG(LogTemp, Verbose, TEXT("ResolvePenetration:   sweep by %s (Original move, attempt success = %d)"), *(MoveDelta).ToString(), bMoved);
				}
			}
		}

		return bMoved;
	}

	return false;
}
#endif

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
	if (Depth > 10 || BounceCount > MaxAllowedBounce || DeltaTime <= KINDA_SMALL_NUMBER) 
	{		
		return Depth;
	}

	// 다음 속도 및 위치 계산 (오일러 적분)	
	// 속도 변화 업데이트 (중력가속도 적용)
	linearVelocity += GravityVector * DeltaTime;

	// 선형 감쇠 적용 (선형 감쇠는 Chaos에서 damping factor로 처리)	
	linearVelocity *= FMath::Clamp(1.0f - LinearDamping * DeltaTime, 0.0f, 1.0f);
	angularVelocity *= FMath::Clamp(1.0f - AngularDamping * DeltaTime, 0.0f, 1.0f);
	
	// 충돌이 없을 경우 사용될 nextPos 후보
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
		FBallBounce HitCache;		
		HitCache.Direction = linearVelocity.GetSafeNormal();
		HitCache.Speed = linearVelocity.Size();
		HitCache.Spin = angularVelocity.Size();
		HitCache.StartPos = pos; // hit.TraceStart;
		HitCache.ImpactPoint = hit.ImpactPoint;
		HitCache.ImpactNormal = hit.ImpactNormal;
		HitCache.Hit = hit;

		float hitTimeRatio = hit.Time;
		float timeToHit = DeltaTime * hitTimeRatio;

		// 남은 시간으로 재귀 호출
		float remainingTime = DeltaTime - timeToHit;

		/* 출동 직전 지점 까지 위치 업데이트
		pos---------*----------------nextPos
					↑
					hit.Location(≈ Lerp(pos, nextPos, hit.Time))
		*/		
		pos = pos + linearVelocity * (timeToHit - KINDA_SMALL_NUMBER);

		// 약간 더 이동 (충돌면에 살짝 박히는 현상 방지)
		//pos += HitCache.BouncedDirection * HitCache.BouncedSpeed * KINDA_SMALL_NUMBER;
		
		//pos = hit.Normal * hit.PenetrationDepth + PullBackDistance;

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
		float frictionScale = FMath::Clamp(1.0f - Friction, 0.f, 1.0f);
		
		// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics6collisionresponse/2017%20Tutorial%206%20-%20Collision%20Response.pdf
		// 충돌 임펄스 계산 (질량, 관성 텐서 반영)
		// J = −((1+e)vRel) ​​/ (m⁻¹​+n⋅((I⁻¹(r×n))×r)(1+e))		
		// 1) 접촉점 P 에서 구 질량중심 C 로 가는 벡터 (접촉점 - 구 중심) r = P - C
		const FVector r = hit.ImpactPoint - pos;

		// 접촉점의 상대 속도 계산 (스핀 방향에 따른 속도 변화 적용)
		// 구의 접촉점 속도 = 구의 선형 속도 + (구의 각속도 × (접촉점 - 구의 중심))		
		const FVector ContactVelocity = linearVelocity + FVector::CrossProduct(angularVelocity, r);

		// 접촉점의 상대 속도를 히트 노멀 방향 으로 프로젝션해서 얻은 값 (vRel > 0 이면 멀어지는중)
		const float vRel = FVector::DotProduct(ContactVelocity, hit.Normal);
		HitCache.vRel = vRel;

		// 접촉점이 서로 멀어지는 중이면 충돌 처리 불필요			
		if (vRel > 0.f)
		{			
			pos = nextPos;
			return Depth;
		}
		const float LVdotN = (linearVelocity.GetSafeNormal() | hit.ImpactNormal);
		bool bIsSliding = false;
		{
			// 짧은 시간에 연속적으로 hit가 발생 중인가? 
			const bool bMultiHit = (PreviousHitTime < 1.f && hit.Time <= UE_KINDA_SMALL_NUMBER);

			// if velocity still into wall (after HandleBlockingHit() had a chance to adjust), slide along wall
			const float DotTolerance = 0.01f;
			bIsSliding = (bMultiHit && FVector::Coincident(PreviousHitNormal, hit.ImpactNormal)) ||
				(LVdotN <= DotTolerance);

			PreviousHitTime = hit.Time;
			PreviousHitNormal = hit.ImpactNormal;

			if (bIsSliding)
			{
				// TBD
				
				//FVector ProjectedNormal = hit.ImpactNormal * -vRel;
				// DotProduct(Delta, ProjectedNormal)
				//float dot = FVector::DotProduct(linearVelocity, ProjectedNormal);
				//// 평면 위로 사영된 벡터
				//FVector projectedVelocity = linearVelocity - dot * ProjectedNormal;
				
				//// 위치 업데이트
				//pos = pos + projectedVelocity * DeltaTime;
				//return Depth;
			}
		}

		// 2) r × n
		FVector rCrossN = FVector::CrossProduct(r, hit.ImpactNormal);
		
		// 3) I⁻¹ * (r × n)
		FVector inertiaTerm = InvInertiaTensor * rCrossN ;
		
		// 4) (I⁻¹ * (r × n)) × r
		FVector crossTerm = FVector::CrossProduct(inertiaTerm, r);
		
		// 5) 최종 분모 denom = m⁻¹+ [(I⁻¹ * (r × n)) × r]⋅n
		float denom = InvMass + FVector::DotProduct(crossTerm, hit.ImpactNormal);
		
		// 6) Restitution : 0 = 완전 비탄성, 1 = 완전 탄성.
		float impulseMagnitude = -(1.0f + Restitution) * vRel / denom;
		
		// (선택) 충격 임펄스 클램핑으로 과도한 임펄스 방지
		impulseMagnitude = FMath::Clamp(impulseMagnitude, 0.f, MaxAllowedImpulse);
		
		// 7) 임펄스 벡터 (법선 방향으로 impulseMagnitude 곱)
		const FVector impulse = impulseMagnitude * hit.ImpactNormal;		
		
		// 8) 선형 속도 업데이트  v = v + impulse * InvMass
		HitCache.LinearImpulse = impulse * InvMass;
		linearVelocity += HitCache.LinearImpulse;
		
		// 임펄스 크기(또는 상대 속도 vRel)가 충분히 크면 bounce, 아니라면 Rolling contact 		
		bIsSliding = (impulseMagnitude <= BounceThreshold);
		
		if(bIsSliding)
		{
			FVector ProjectedNormal = hit.ImpactNormal * -vRel;
			
			// DotProduct(Delta, ProjectedNormal)
			float dot = FVector::DotProduct(linearVelocity, ProjectedNormal);

			// 평면 위로 사영된 벡터
			FVector projectedVelocity = linearVelocity - dot * ProjectedNormal;

			//// 위치 업데이트
			//pos = pos + projectedVelocity * DeltaTime;
			//return Depth;
		}

		// 쿠롱 마찰 임펄스 계산 (접선 방향 임펄스)
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
			HitCache.FrictionDelta = frictionImpulse * InvMass;
	
			// 선형 속도 업데이트 (마찰 임펄스 적용)
			linearVelocity += frictionImpulse * InvMass;
			
			// 각속도 업데이트 Δω = I⁻¹ * (r × J)			
			FVector angularFrictionImpulse = FVector::CrossProduct(r, frictionImpulse);
			FVector angularDelta = InvInertiaTensor * angularFrictionImpulse;
			angularVelocity += angularDelta * SpinToRotateMultiply;
						
			HitCache.AngularDelta = angularDelta;
			HitCache.AngularDeltaSize = angularDelta.Size();
		}

		// 각속도 업데이트 (시간 간격에 따라 회전 속도 감소) - 시뮬레이션 루프에서 처리함
		// angularVelocity *= FMath::Pow(SpinFrictionScale, timeToHit);

		//const float PenetrationVelocityDamping = 0.5f;    // 감속 계수		
		const float PenetrationDepthThreshold = 0.1f;     // 끼인 것으로 판단할 최소 깊이

		// trace started in penetration, i.e. with an initial blocking overlap.
		bool bIsStuck = hit.bStartPenetrating || hit.PenetrationDepth > PenetrationDepthThreshold;
		if (bIsStuck)
		{
			// TBD - 재현 방법 및 동작 여부 확인 필요
			HitCache.bWasStuck = bIsStuck;

			//const float SmallMargin = KINDA_SMALL_NUMBER;         
			const float SmallMargin = 0.1f;                  // 밀어낼 여유 마진

			// 침투 깊이만큼 푸시백
			const FVector PenetrationDirection = hit.Normal.IsNearlyZero() ? FVector::UpVector : hit.Normal;
			const float PushBack = hit.PenetrationDepth + SmallMargin; // 소량 여유 마진 추가
			pos += PenetrationDirection * PushBack;
			
			// 디버그용 출력 또는 로그
			UE_LOG(LogBallSimulatorComponent, Verbose, TEXT("Penetration resolved: depth = %.3f, push = %s"), hit.PenetrationDepth, *PenetrationDirection.ToString());
		}

		HitCache.NextPos = hit.Location + linearVelocity * remainingTime;
		HitCache.hitTimeRatio = hitTimeRatio;
		HitCache.timeToHit = timeToHit;
		HitCache.RemainingTime = remainingTime;
		HitCache.SnapshotIndex = CachedSnapshots.Num();
		HitCache.BouncedDirection = linearVelocity.GetSafeNormal();
		HitCache.BouncedSpeed = linearVelocity.Size();
		HitCache.BouncedSpin = angularVelocity.Size();		
		HitCache.PenetrationDepth = hit.PenetrationDepth;
		HitCache.bWasStuck = bIsStuck;
		HitCache.bIsSliding = bIsSliding;
		CachedHits.Add(HitCache);

		return HandleCollision(World, InvMass, CollisionShape, pos, linearVelocity, angularVelocity, remainingTime, Depth + 1);
	}
	else
	{
		pos = nextPos;
		return Depth;
	}
}


void UBallSimulatorComponent::PerformRaycastCollision(
	const FVector& startPos,  // 시작 위치
	const FVector& velocity,  // 이동 속도
	float radius,             // 구체의 반지름
	float maxRayLength,       // 최대 레이 길이
	FHitResult& hitResult,    // 충돌 정보
	FTransform& motion        // 물리적인 위치 및 속도 정보
)
{
	// 1. 레이의 끝 위치 계산: 속도 방향에 최대 레이 길이를 곱하여 계산
	FVector rayDirection = velocity.GetSafeNormal() * maxRayLength;  // 이동 방향에 최대 길이만큼 곱해서 레이 방향 계산
	FVector endPos = startPos + rayDirection;  // 시작 위치와 끝 위치 계산

	// 2. 레이캐스트 수행
	FCollisionQueryParams collisionParams;
	collisionParams.AddIgnoredActor(GetOwner());  // 레이캐스트에서 자기 자신은 제외
	bool bHit = GetWorld()->LineTraceSingleByChannel(hitResult, startPos, endPos, ECC_Visibility, collisionParams);

	// 3. 충돌이 발생한 경우 처리
	if (bHit)
	{
		// 4. 충돌 정보 업데이트
		// 예시로 마찰력과 복원력을 설정 (충돌한 재질에 따라 동적으로 설정 가능)
		float friction = 0.5f;  // 예시 마찰력
		float restitution = 0.8f;  // 예시 복원력

		// 5. 침투 해결: 충돌 지점과 법선 벡터를 이용하여 위치 보정
		FVector hitPosition = hitResult.ImpactPoint;  // 충돌 지점
		FVector correction = hitResult.ImpactNormal * radius;  // 침투 깊이만큼 보정 벡터 생성
		motion.SetLocation(hitPosition - correction);  // 보정된 위치로 이동

		// 6. 속도 반사: 충돌 법선 방향으로 반사된 속도를 계산
		FVector reflectedVelocity = velocity - 2 * FVector::DotProduct(velocity, hitResult.ImpactNormal) * hitResult.ImpactNormal;

		// 7. 물체의 새로운 속도 업데이트
		motion.SetRotation(FQuat::Identity);  // 회전은 그대로 두고 속도만 반영
		FVector newVelocity = reflectedVelocity;

		// 8. 속도가 매우 작은 경우 정지 처리
		if (newVelocity.SizeSquared() < 0.1f)  // 속도가 거의 0인 경우
		{
			newVelocity = FVector::ZeroVector;  // 속도 0으로 설정하여 정지
		}

		// 9. 반사된 속도 적용
		motion.SetScale3D(newVelocity);  // 속도 업데이트
	}
}

void SolveVertexContact(
	FVector& LinearVelocity,        // 선형 속도
	FVector& AngularVelocity,       // 각속도
	const FVector& ContactPlaneNormal,  // 접촉 평면의 법선
	const FVector& ContactPoint,    // 접촉 지점
	const FVector& ContactVelocity, // 접촉 지점에서의 상대 속도
	const FVector& RelativePosition,  // 접촉 지점에 대한 상대 위치
	const float Restitution,              // 복원력
	const float Friction,                 // 마찰력
	const float PenetrationDepth,         // 침투 깊이
	const float InverseMass,              // 질량의 역수
	const float InverseInertia,           // 관성의 역수
	const float DeltaTime )
{
	// Step 1: 접촉 지점에서의 상대 속도 계산
	FVector RelativeVelocity = LinearVelocity + FVector::CrossProduct(AngularVelocity, RelativePosition) - ContactVelocity;

	// Step 2: 상대 속도를 법선 방향과 수평 방향으로 분해
	float NormalVelocity = FVector::DotProduct(RelativeVelocity, ContactPlaneNormal); // 법선 방향 속도
	FVector TangentialVelocity = RelativeVelocity - (NormalVelocity * ContactPlaneNormal); // 수평 방향 속도

	// Step 3: 법선 방향 충격량 (바운스) 해결
	FVector NormalImpulse = FVector::ZeroVector;
	if (NormalVelocity < 0.0f) // 접촉 평면으로 접근하는 경우에만 해결
	{
		float RestitutionImpulse = -NormalVelocity * Restitution;  // 복원력에 의한 충격량
		NormalImpulse = ContactPlaneNormal * (RestitutionImpulse + PenetrationDepth / DeltaTime);  // 충돌 반응으로 인한 법선 방향 충격
	}

	// Step 4: 마찰력에 의한 충격량 해결
	FVector FrictionImpulse = -TangentialVelocity;
	float MaxFriction = Friction * NormalImpulse.Size(); // 최대 마찰력
	if (FrictionImpulse.Size() > MaxFriction)  // 마찰력 제한
	{
		FrictionImpulse = FrictionImpulse.GetSafeNormal() * MaxFriction;  // 마찰력을 최대값으로 클램프
	}

	// Step 5: 선형 및 각속도에 충격량 적용
	FVector TotalImpulse = NormalImpulse + FrictionImpulse;  // 총 충격량 계산

	// 선형 속도 업데이트
	LinearVelocity += TotalImpulse * InverseMass * DeltaTime;

	// 각속도 업데이트 (충격량에 의해 발생한 토크 적용)
	FVector Torque = FVector::CrossProduct(RelativePosition, TotalImpulse);  // 회전 토크 계산
	AngularVelocity += Torque * InverseInertia * DeltaTime;  // 각속도 업데이트
}

void SolveVertexFriction(
	FVector& LinearVelocity,        // 선형 속도
	FVector& AngularVelocity,       // 각속도
	const FVector& ContactPlaneNormal,  // 접촉 평면의 법선
	const FVector& RelativePosition,   // 접촉 지점에 대한 상대 위치
	const FVector& TangentialContactVelocity, // 접촉 지점에서의 수평 속도
	const float FrictionCoefficient,      // 마찰 계수 (마찰력)
	const float NormalForceMagnitude,     // 법선 방향의 힘 크기
	const float InverseMass,              // 질량의 역수
	const float InverseInertia,            // 관성의 역수
	const float DeltaTime)

{
	// Step 1: 상대적인 수평 속도 계산
	FVector RelativeTangentialVelocity = LinearVelocity + FVector::CrossProduct(AngularVelocity, RelativePosition) - TangentialContactVelocity;

	// Step 2: 마찰력에 의한 충격량 계산
	FVector FrictionImpulse = -RelativeTangentialVelocity; // 마찰력은 수평 속도의 반대 방향
	float MaxFrictionImpulse = FrictionCoefficient * NormalForceMagnitude;  // 최대 마찰력

	// Step 3: 마찰력을 Coulomb의 법칙에 따라 최대값으로 제한
	if (FrictionImpulse.Size() > MaxFrictionImpulse) 
	{
		FrictionImpulse = FrictionImpulse.GetSafeNormal() * MaxFrictionImpulse;  // 마찰력을 최대값으로 클램프
	}

	// Step 4: 선형 속도에 마찰력 적용
	LinearVelocity += FrictionImpulse * InverseMass;

	// Step 5: 각속도에 마찰력에 의한 토크 적용
	FVector Torque = FVector::CrossProduct(RelativePosition, FrictionImpulse);  // 마찰력에 의한 회전 토크 계산
	AngularVelocity += Torque * InverseInertia * DeltaTime;  // 각속도 업데이트
}

void UBallSimulatorComponent::SolveSphereContact(const float Radius, FHitResult& contact, FVector& position, FVector& velocity, float dt)
{
	// 법선 방향 속도 계산 (속도와 충돌 법선 벡터의 내적을 통해 구합니다)
	float normalVelocity = FVector::DotProduct(velocity, contact.ImpactNormal);

	// 충돌 평면을 넘어서 들어간 경우 (penetration) 해결
	// 충돌 점에서 평면에 대한 거리 계산
	float distance = FVector::DotProduct(contact.ImpactNormal, position - contact.ImpactPoint) - Radius;

	if (distance < 0.0f)
	{
		// 침투된 만큼 위치를 수정하여 충돌을 해결합니다
		FVector correction = contact.ImpactNormal * -distance;
		position += correction;  // 위치 보정
	}

	// 법선 방향 충격량 적용 (법선 속도가 음수인 경우 충돌을 해결합니다)
	if (normalVelocity < 0.0f) // 법선 속도가 음수일 경우, 즉 충격이 있을 경우에만 해결
	{
		// 복원력에 의한 충격량 계산 (탄성 충돌)
		float restitutionImpulse = normalVelocity * DefaultRestitution;
		// 최종 충격량 계산 (법선 속도 + 복원력에 의한 충격)
		float normalImpulse = -(normalVelocity + restitutionImpulse);
		// 속도에 법선 방향 충격량 적용
		velocity += contact.ImpactNormal * normalImpulse;
	}

	// 마찰력에 의한 충격량 적용 (수평 방향 속도 해결)
	FVector tangentialVelocity = velocity - normalVelocity * contact.ImpactNormal;  // 수평 속도 계산

	// 수평 속도의 크기 계산
	float tangentialVelocitySquared = tangentialVelocity.SizeSquared();

	// 마찰력에 의한 최대 속도 크기 (maxFriction) 계산
	float maxFriction = DefaultFriction * FMath::Abs(normalVelocity);

	// 마찰력 크기 제어: 수평 속도가 maxFriction보다 크면, 마찰력을 적용하여 속도를 제한
	if (tangentialVelocitySquared > maxFriction * maxFriction)
	{
		tangentialVelocity.Normalize();
		tangentialVelocity *= maxFriction;  // 마찰력 크기만큼 속도 제한
	}

	// 마찰력을 적용한 속도 업데이트
	velocity -= tangentialVelocity;
}



#if 0
bool HandleSliding(FHitResult& Hit, const FVector& OldVelocity, const uint32 NumBounces, float& SubTickTimeRemaining)
{
	const FVector Normal = ConstrainNormalToPlane(Hit.Normal);

	// Multiple hits within very short time period?
	const bool bMultiHit = (PreviousHitTime < 1.f && Hit.Time <= UE_KINDA_SMALL_NUMBER);

	// if velocity still into wall (after HandleBlockingHit() had a chance to adjust), slide along wall
	const float DotTolerance = 0.01f;
	bIsSliding = (bMultiHit && FVector::Coincident(PreviousHitNormal, Normal)) ||
		((Velocity.GetSafeNormal() | Normal) <= DotTolerance);

	if (bIsSliding)
	{
		if (bMultiHit && (PreviousHitNormal | Normal) <= 0.f)
		{
			// 90 degree or less corner, so use cross product for direction
			FVector NewDir = (Normal ^ PreviousHitNormal).GetSafeNormal();
			Velocity = Velocity.ProjectOnToNormal(NewDir);
			if ((OldVelocity | Velocity) < 0.f)
			{
				Velocity *= -1.f;
			}
			Velocity = ConstrainDirectionToPlane(Velocity);
		}
		else
		{
			// === ComputeSlideVector 풀어쓰기 ===
			const FVector SlideResult = Velocity - (Velocity | Normal) * Normal;
			Velocity = ConstrainDirectionToPlane(SlideResult);
		}

		// Check min velocity.
		if (IsVelocityUnderSimulationThreshold())
		{
			StopSimulating(Hit);
			return false;
		}

		// === HandleSliding 풀어쓰기 ===
		if (SubTickTimeRemaining > UE_KINDA_SMALL_NUMBER)
		{
			// 기본 이동 (벽에 붙어서 미끄러짐)
			const FVector MoveDelta = Velocity * SubTickTimeRemaining;

			// Sweep 이동
			FHitResult SlideHit;
			SafeMoveUpdatedComponent(MoveDelta, UpdatedComponent->GetComponentQuat(), true, SlideHit);

			if (SlideHit.bBlockingHit)
			{
				// 충돌한 경우 다시 슬라이딩 방향 보정
				const FVector NewNormal = ConstrainNormalToPlane(SlideHit.Normal);
				FVector NewSlide = Velocity - (Velocity | NewNormal) * NewNormal;
				Velocity = ConstrainDirectionToPlane(NewSlide);

				// SubTickTimeRemaining 갱신
				const float PercentTimeApplied = FMath::Max(0.f, SlideHit.Time);
				SubTickTimeRemaining *= (1.f - PercentTimeApplied);

				// Velocity 너무 작으면 중단
				if (IsVelocityUnderSimulationThreshold())
				{
					StopSimulating(SlideHit);
					return false;
				}
			}
			else
			{
				// 충돌이 없으면 이동 완료 → 더 이상 처리 필요 없음
				SubTickTimeRemaining = 0.f;
			}
		}
	}

	return true;
}
#endif

void UBallSimulatorComponent::ConvertSnapshotsToBezierSpline(
	const TArray<FBallSnapshot>& Snapshots,	USplineComponent* SplineComponent) const
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