// Fill out your copyright notice in the Description page of Project Settings.


#include "PCG/GenerationManager.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/Transform.h"
#include "TimerManager.h"
#include "UObject/ConstructorHelpers.h"
#include "UObject/UnrealType.h"

namespace
{
TArray<FTransform>* FindTransformArrayProperty(UObject* Object, const FName PropertyName)
{
	if (!Object) {
		return nullptr;
	}

	if (FArrayProperty* ArrayProperty = FindFProperty<FArrayProperty>(Object->GetClass(), PropertyName)) {
		if (FStructProperty* InnerStruct = CastField<FStructProperty>(ArrayProperty->Inner)) {
			if (InnerStruct->Struct == TBaseStructure<FTransform>::Get()) {
				return ArrayProperty->ContainerPtrToValuePtr<TArray<FTransform>>(Object);
			}
		}
	}

	return nullptr;
}

TArray<AActor*>* FindActorArrayProperty(UObject* Object, const FName PropertyName)
{
	if (!Object) {
		return nullptr;
	}

	if (FArrayProperty* ArrayProperty = FindFProperty<FArrayProperty>(Object->GetClass(), PropertyName)) {
		if (FObjectPropertyBase* InnerObject = CastField<FObjectPropertyBase>(ArrayProperty->Inner)) {
			if (InnerObject->PropertyClass && InnerObject->PropertyClass->IsChildOf(AActor::StaticClass())) {
				return ArrayProperty->ContainerPtrToValuePtr<TArray<AActor*>>(Object);
			}
		}
	}

	return nullptr;
}

FRandomStream* FindRandomStreamProperty(UObject* Object, const FName PropertyName)
{
	if (!Object) {
		return nullptr;
	}

	if (FStructProperty* StructProperty = FindFProperty<FStructProperty>(Object->GetClass(), PropertyName)) {
		if (StructProperty->Struct == TBaseStructure<FRandomStream>::Get()) {
			return StructProperty->ContainerPtrToValuePtr<FRandomStream>(Object);
		}
	}

	return nullptr;
}

void SetBoolProperty(UObject* Object, const FName PropertyName, const bool bValue)
{
	if (!Object) {
		return;
	}

	if (FBoolProperty* BoolProperty = FindFProperty<FBoolProperty>(Object->GetClass(), PropertyName)) {
		BoolProperty->SetPropertyValue_InContainer(Object, bValue);
	}
}

void SetRealProperty(UObject* Object, const FName PropertyName, const double Value)
{
	if (!Object) {
		return;
	}

	if (FDoubleProperty* DoubleProperty = FindFProperty<FDoubleProperty>(Object->GetClass(), PropertyName)) {
		DoubleProperty->SetPropertyValue_InContainer(Object, Value);
		return;
	}

	if (FFloatProperty* FloatProperty = FindFProperty<FFloatProperty>(Object->GetClass(), PropertyName)) {
		FloatProperty->SetPropertyValue_InContainer(Object, static_cast<float>(Value));
	}
}
} // namespace

// Sets default values
AGenerationManager::AGenerationManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	static ConstructorHelpers::FClassFinder<AActor> DefaultObstacleClass(TEXT("/AirSim/Blueprints/BP_NPCSpawn"));
	if (DefaultObstacleClass.Succeeded()) {
		ObstacleClass = DefaultObstacleClass.Class;
	}
}

// Called when the game starts or when spawned
void AGenerationManager::BeginPlay()
{
	Super::BeginPlay();
	
	aseed = 10;
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d"), aseed);
}

bool AGenerationManager::HavenStep(TArray<FTransform>& haven, float mina, float maxa, float mind, float maxd)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 2"), aseed);
	return true;
}
void AGenerationManager::WallStep(TArray<FTransform>& haven, TArray<FTransform>& wall, float min, float max)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 3"), aseed);
}
void AGenerationManager::GenerateWalls(TArray<FTransform>& points, bool inlets, float min, float max)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 4"), aseed);
}
void AGenerationManager::InletFunction(FTransform begin, FTransform end, FRotator rotation, float min, float max, float length)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 5"), aseed);
}
void AGenerationManager::InletGeneration(TArray<FTransform>& points)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 6"), aseed);
}
void AGenerationManager::Clear()
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 6"), aseed);
}
void AGenerationManager::SetSeed(int32 seed)
{
	aseed = seed;
	stream.Initialize(aseed);
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 7"), aseed);
}
void AGenerationManager::generateTerrain(FString type, float mina, float maxa, float mind, float maxd, int length)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 8"), aseed);
}
void AGenerationManager::getGoal(int32 distance, FVector& location, FVector& left, FVector& right)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 9"), aseed);
}

void AGenerationManager::SpawnObstacles()
{
	UWorld* World = GetWorld();
	if (!World) {
		UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning skipped: world is null."));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning scheduled for next tick."));
	World->GetTimerManager().SetTimerForNextTick(FTimerDelegate::CreateWeakLambda(this, [this]()
	{
		if (IsValid(this)) {
			SpawnObstaclesImmediate();
		}
	}));
}

void AGenerationManager::SpawnObstaclesImmediate()
{
	if (!bSpawnObstacles) {
		UE_LOG(LogTemp, Verbose, TEXT("GenerationManager obstacle spawning disabled."));
		return;
	}

	UWorld* World = GetWorld();
	if (!World) {
		UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning skipped: world is null."));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning started."));

	TArray<FTransform>* RoadPoints = FindTransformArrayProperty(this, TEXT("road"));
	if (!RoadPoints) {
		RoadPoints = &road;
	}

	TArray<FTransform>* LeftPoints = FindTransformArrayProperty(this, TEXT("left"));
	if (!LeftPoints) {
		LeftPoints = FindTransformArrayProperty(this, TEXT("aleft"));
	}

	TArray<FTransform>* RightPoints = FindTransformArrayProperty(this, TEXT("right"));
	if (!RightPoints) {
		RightPoints = FindTransformArrayProperty(this, TEXT("aright"));
	}

	TArray<AActor*>* GeneratedActors = FindActorArrayProperty(this, TEXT("Generated"));
	if (!GeneratedActors) {
		GeneratedActors = FindActorArrayProperty(this, TEXT("generated"));
	}
	if (!GeneratedActors) {
		GeneratedActors = &generated;
	}

	FRandomStream* StreamProperty = FindRandomStreamProperty(this, TEXT("stream"));
	FRandomStream FallbackStream(aseed);
	FRandomStream& ObstacleStream = StreamProperty ? *StreamProperty : FallbackStream;

	if (!ObstacleClass) {
		UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning skipped: ObstacleClass is null."));
		return;
	}

	const int32 PointCount = RoadPoints ? RoadPoints->Num() : 0;
	if (PointCount <= 0) {
		UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning skipped: road array is empty."));
		return;
	}

	const int32 BoundaryCount = (LeftPoints && RightPoints)
		? FMath::Min(LeftPoints->Num(), RightPoints->Num())
		: PointCount;
	const int32 UsableCount = FMath::Min(PointCount, BoundaryCount);
	if (UsableCount <= ObstacleStartBuffer + ObstacleEndBuffer) {
		UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning skipped: not enough generated points (%d)."), UsableCount);
		return;
	}

	const int32 StartIndex = FMath::Clamp(ObstacleStartBuffer, 0, UsableCount - 1);
	const int32 LastIndex = FMath::Clamp(UsableCount - 1 - FMath::Max(ObstacleEndBuffer, 0), 0, UsableCount - 1);
	if (StartIndex > LastIndex) {
		UE_LOG(LogTemp, Warning, TEXT("GenerationManager obstacle spawning skipped: candidate range is empty."));
		return;
	}

	const int32 StepMin = FMath::Max(1, FMath::Min(ObstacleMinIndexStep, ObstacleMaxIndexStep));
	const int32 StepMax = FMath::Max(StepMin, FMath::Max(ObstacleMinIndexStep, ObstacleMaxIndexStep));
	const int32 MaxSpawnCount = FMath::Max(0, ObstacleMaxCount);

	int32 CurrentIndex = StartIndex + ObstacleStream.RandRange(0, StepMax - 1);
	int32 SpawnedCount = 0;
	int32 SafetyCounter = 0;

	while (CurrentIndex <= LastIndex && SpawnedCount < MaxSpawnCount && SafetyCounter < UsableCount) {
		const FTransform& RoadTransform = (*RoadPoints)[CurrentIndex];
		const FVector RoadLocation = RoadTransform.GetLocation();
		const FRotator RoadRotation = RoadTransform.Rotator();
		const FVector ForwardVector = RoadRotation.Vector();
		const FVector RightVector = RoadRotation.RotateVector(FVector::RightVector);

		float HalfUsableWidth = 0.0f;
		if (LeftPoints && RightPoints && LeftPoints->IsValidIndex(CurrentIndex) && RightPoints->IsValidIndex(CurrentIndex)) {
			const float ChannelWidth = FVector::Dist2D((*LeftPoints)[CurrentIndex].GetLocation(), (*RightPoints)[CurrentIndex].GetLocation());
			if (ChannelWidth < ObstacleMinChannelWidth) {
				CurrentIndex += ObstacleStream.RandRange(StepMin, StepMax);
				++SafetyCounter;
				continue;
			}

			HalfUsableWidth = FMath::Max((ChannelWidth * 0.5f) - ObstacleBoundaryPadding, 0.0f);
			HalfUsableWidth *= FMath::Clamp(ObstacleLateralRatio, 0.0f, 1.0f);
		}

		const float LateralOffset = (HalfUsableWidth > KINDA_SMALL_NUMBER)
			? ObstacleStream.FRandRange(-HalfUsableWidth, HalfUsableWidth)
			: 0.0f;
		const float ForwardOffset = (ObstacleForwardJitter > 0.0f)
			? ObstacleStream.FRandRange(-ObstacleForwardJitter, ObstacleForwardJitter)
			: 0.0f;
		const float YawOffset = (ObstacleYawJitter > 0.0f)
			? ObstacleStream.FRandRange(-ObstacleYawJitter, ObstacleYawJitter)
			: 0.0f;

		FVector SpawnLocation = RoadLocation + (RightVector * LateralOffset) + (ForwardVector * ForwardOffset);
		SpawnLocation.Z += ObstacleZOffset;

		FRotator SpawnRotation = RoadRotation;
		SpawnRotation.Pitch = 0.0f;
		SpawnRotation.Roll = 0.0f;
		SpawnRotation.Yaw += YawOffset;

		const FTransform SpawnTransform(SpawnRotation, SpawnLocation);
		AActor* SpawnedObstacle = World->SpawnActorDeferred<AActor>(
			ObstacleClass,
			SpawnTransform,
			this,
			nullptr,
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn);

		if (SpawnedObstacle) {
			const float SpeedMin = FMath::Min(ObstacleSpeedMin, ObstacleSpeedMax);
			const float SpeedMax = FMath::Max(ObstacleSpeedMin, ObstacleSpeedMax);
			const double ObstacleSpeed = (SpeedMax > SpeedMin)
				? ObstacleStream.FRandRange(SpeedMin, SpeedMax)
				: SpeedMin;

			SetBoolProperty(SpawnedObstacle, TEXT("CustomVelocity"), true);
			SetBoolProperty(SpawnedObstacle, TEXT("ApplyVelocityOnBeginPlay"), false);
			SetRealProperty(SpawnedObstacle, TEXT("Speed"), ObstacleSpeed);

			UGameplayStatics::FinishSpawningActor(SpawnedObstacle, SpawnTransform);
			GeneratedActors->Add(SpawnedObstacle);
			++SpawnedCount;
		}

		CurrentIndex += ObstacleStream.RandRange(StepMin, StepMax);
		++SafetyCounter;
	}

	UE_LOG(LogTemp, Warning, TEXT("GenerationManager spawned %d obstacle actors."), SpawnedCount);
}

// Called every frame
void AGenerationManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

