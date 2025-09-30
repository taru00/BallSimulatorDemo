// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#include "BallSimulatorActor.h"
#include "DrawDebugHelpers.h"

ABallSimulatorActor::ABallSimulatorActor()
{    
    PrimaryActorTick.bCanEverTick = false;
 
    // Root
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

    // Spline Component
    SplineComp = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));
    SplineComp->SetupAttachment(RootComponent);

    // Ball Simulator Component
    BallSimulatorComp = CreateDefaultSubobject<UBallSimulatorComponent>(TEXT("BallSimulatorComponent"));
    
    // Ball Mesh Component
    BallMeshComp = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BallMesh"));
    BallMeshComp->SetupAttachment(RootComponent);

    // SimSphereComp
    SimSphereComp = CreateDefaultSubobject<USphereComponent>(TEXT("SimSphereComp"));
    SimSphereComp->SetupAttachment(RootComponent);

    // Sphere Collision Component
    SphereCollisionComp = CreateDefaultSubobject<USphereComponent>(TEXT("CollisionComponent"));
    SphereCollisionComp->InitSphereRadius(16.0f); 
    SphereCollisionComp->SetCollisionProfileName(TEXT("OverlapAll"));
    SphereCollisionComp->SetupAttachment(BallMeshComp);    
}

void ABallSimulatorActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bIsPlayingAnimation)
        return;

    PlaybackTime += DeltaTime;

    FVector Position;
    FQuat Rotation;

    if (BallSimulatorComp->GetBallPositionAndRotationAtSplineTime(SplineComp, PlaybackTime, Position, Rotation))
    {        
        BallMeshComp->SetWorldLocationAndRotation(Position, Rotation);
    }
    else
    {
        bIsPlayingAnimation = false; // 끝났으면 정지
    }
}

void ABallSimulatorActor::BeginPlay()
{

}

void ABallSimulatorActor::InitializeSimPhysicsScene()
{
#if 0
    // 별도의 물리 월드를 만들어서 시뮬레이트 해야할 경우 참고
    // ChaosPhysicsFactory 가 현재 엔진 Physics Factory임을 가정
    IPhysicsFactory* PhysicsFactory = Chaos::FChaosEngineInterface::Get()->GetPhysicsFactory();

    // Chaos Solver 생성
    SimSolver = MakeUnique<FChaosSolver>();
    SimSolver->SetSolverMode(Chaos::ESolverMode::SingleThread);
    SimSolver->SetEnableAsyncMode(false);

    // Physics Scene 생성
    SimPhysicsScene = MakeUnique<FPhysScene_Chaos>(PhysicsFactory, SimSolver.Get());

    // 씬 초기화
    SimPhysicsScene->Init();

    // Solver에 씬 등록
    SimSolver->AddScene(SimPhysicsScene.Get());
#endif
}
void ABallSimulatorActor::StepSimPhysicsScene(float DeltaTime)
{
#if 0
    // 별도의 물리 월드에서 시뮬레이션 Step 예제
    if (!SimPhysicsScene.IsValid())
        return;

    // 물리 시뮬레이션 수행
    SimPhysicsScene->Simulate(DeltaTime);

    // 필요 시 SimPhysicsScene 내 액터, 컴포넌트의 위치/회전값 읽기
    // 예: SimPhysicsScene->GetRigidBodies() 등
#endif
}

void ABallSimulatorActor::SimulateBallPhysics(    
    UObject* WorldContextObject,
    const FVector& InitialPosition,
    const FVector& InitialVelocity,
    const FVector& InitialSpin,
    float SphereCollisionRadius,
    float SimulationTime,
    float StepInterval,
    TArray<FBallSnapshot>& OutSnapshots)
{    
    UWorld* World = WorldContextObject ? WorldContextObject->GetWorld() : nullptr;
    if (!World) return;

}
