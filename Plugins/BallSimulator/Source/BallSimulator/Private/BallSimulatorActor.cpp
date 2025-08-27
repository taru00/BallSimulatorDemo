// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#include "BallSimulatorActor.h"
#include "DrawDebugHelpers.h"
//#include "Chaos/ChaosEngineInterface.h"
//#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
//#include "Chaos/ChaosSolver.h"
//#include "Chaos/PhysicsMaterials.h"

// Sets default values
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
    //if (!SimWorld)
    //{
    //    SimWorld = UWorld::CreateWorld(EWorldType::None, false);      
    //    NewWorld->bIsWorldInitialized = false;

    //    NewWorld->InitializeNewWorld();

    //    NewWorld->SetFlags(RF_Standalone);
    //}
}

void ABallSimulatorActor::InitializeSimPhysicsScene()
{
    //// ChaosPhysicsFactory 가 현재 엔진 Physics Factory임을 가정
    //IPhysicsFactory* PhysicsFactory = Chaos::FChaosEngineInterface::Get()->GetPhysicsFactory();

    //// Chaos Solver 생성
    //SimSolver = MakeUnique<FChaosSolver>();
    //SimSolver->SetSolverMode(Chaos::ESolverMode::SingleThread);
    //SimSolver->SetEnableAsyncMode(false);

    //// Physics Scene 생성
    //SimPhysicsScene = MakeUnique<FPhysScene_Chaos>(PhysicsFactory, SimSolver.Get());

    //// 씬 초기화
    //SimPhysicsScene->Init();

    //// Solver에 씬 등록
    //SimSolver->AddScene(SimPhysicsScene.Get());
}

// 별도 물리 시뮬레이션 Step 함수
void ABallSimulatorActor::StepSimPhysicsScene(float DeltaTime)
{
    //if (!SimPhysicsScene.IsValid())
    //    return;

    //// 물리 시뮬레이션 수행
    //SimPhysicsScene->Simulate(DeltaTime);

    // 필요 시 SimPhysicsScene 내 액터, 컴포넌트의 위치/회전값 읽기
    // 예: SimPhysicsScene->GetRigidBodies() 등
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

    //if (!SimWorld)
    //{
    //    if (GEngine)
    //    {
    //        GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("Error: SimWorld is null! This function is available only after BeginPlay() has called."));
    //    }
    //    return;
    //}

/*    if(StepInterval <= 0.0f)
        return;

    int TotalStepCount = SimulationTime / StepInterval;
    OutSnapshots.Empty();
    OutSnapshots.Reserve(TotalStepCount);

    SimSphereComp->SetWorldLocation(InitialPosition);
    SimSphereComp->SetPhysicsLinearVelocity(InitialVelocity);
    SimSphereComp->SetPhysicsAngularVelocityInRadians(InitialSpin);
    SimSphereComp->SetSphereRadius(SphereCollisionRadius);

    float TimeElapsed = 0.0f;

    while (TimeElapsed < SimulationTime)
    {
        SimWorld->Tick(ELevelTick::LEVELTICK_All, StepInterval);

        FBallSnapshot Snapshot;
        Snapshot.Position = SimSphereComp->GetComponentLocation();
        Snapshot.Rotation = SimSphereComp->GetComponentQuat();
        Snapshot.Direction = SimSphereComp->GetPhysicsLinearVelocity();
        Snapshot.Spin = SimSphereComp->GetPhysicsAngularVelocityInRadians();
        Snapshot.Speed = Snapshot.Direction.Size();

        OutSnapshots.Add(Snapshot);

        if (Snapshot.Speed < 10.f && Snapshot.Spin.Size() < 1.f)
            break;

        TimeElapsed += StepInterval;
    }  */  
}
