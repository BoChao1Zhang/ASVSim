// Fill out your copyright notice in the Description page of Project Settings.


#include "PCG/GenerationManager.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/Transform.h"
#include "Math/UnrealMathUtility.h"
#include "TimerManager.h"
#include "UObject/ConstructorHelpers.h"
#include "UObject/UnrealType.h"
#include <queue>

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

TArray<AActor*>* FindGeneratedActorArrayProperty(UObject* Object)
{
	if (TArray<AActor*>* GeneratedActors = FindActorArrayProperty(Object, TEXT("Generated"))) {
		return GeneratedActors;
	}
	return FindActorArrayProperty(Object, TEXT("generated"));
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

FIntProperty* FindIntProperty(UObject* Object, const FName PropertyName)
{
	if (!Object) {
		return nullptr;
	}

	return FindFProperty<FIntProperty>(Object->GetClass(), PropertyName);
}

bool GetBoolPropertyValue(const UObject* Object, const FName PropertyName, const bool DefaultValue)
{
	if (!Object) {
		return DefaultValue;
	}

	if (const FBoolProperty* BoolProperty = FindFProperty<FBoolProperty>(Object->GetClass(), PropertyName)) {
		return BoolProperty->GetPropertyValue_InContainer(Object);
	}

	return DefaultValue;
}

int32 GetIntPropertyValue(const UObject* Object, const FName PropertyName, const int32 DefaultValue)
{
	if (!Object) {
		return DefaultValue;
	}

	if (const FIntProperty* IntProperty = FindFProperty<FIntProperty>(Object->GetClass(), PropertyName)) {
		return IntProperty->GetPropertyValue_InContainer(Object);
	}

	return DefaultValue;
}

float GetFloatPropertyValue(const UObject* Object, const FName PropertyName, const float DefaultValue)
{
	if (!Object) {
		return DefaultValue;
	}

	if (const FFloatProperty* FloatProperty = FindFProperty<FFloatProperty>(Object->GetClass(), PropertyName)) {
		return FloatProperty->GetPropertyValue_InContainer(Object);
	}

	if (const FDoubleProperty* DoubleProperty = FindFProperty<FDoubleProperty>(Object->GetClass(), PropertyName)) {
		return static_cast<float>(DoubleProperty->GetPropertyValue_InContainer(Object));
	}

	return DefaultValue;
}

TArray<FPCGObstacleSpec>* FindObstacleSpecArrayProperty(UObject* Object, const FName PropertyName)
{
	if (!Object) {
		return nullptr;
	}

	if (FArrayProperty* ArrayProperty = FindFProperty<FArrayProperty>(Object->GetClass(), PropertyName)) {
		if (FStructProperty* InnerStruct = CastField<FStructProperty>(ArrayProperty->Inner)) {
			if (InnerStruct->Struct == FPCGObstacleSpec::StaticStruct()) {
				return ArrayProperty->ContainerPtrToValuePtr<TArray<FPCGObstacleSpec>>(Object);
			}
		}
	}

	return nullptr;
}

bool IsShipObstacleClassPath(const FString& ClassPath)
{
	return ClassPath.Contains(TEXT("Boat_Blueprint"))
		|| ClassPath.Contains(TEXT("Barge_Blueprint"))
		|| ClassPath.Contains(TEXT("BP_CargoPawn"))
		|| ClassPath.Contains(TEXT("BP_NPCSpawn"));
}

bool IsShipObstacleClass(const TSoftClassPtr<AActor>& ObstacleClass)
{
	return IsShipObstacleClassPath(ObstacleClass.ToSoftObjectPath().ToString());
}

bool IsShipObstacleClass(const UClass* ObstacleClass)
{
	return ObstacleClass && IsShipObstacleClassPath(ObstacleClass->GetPathName());
}

void DisableDynamicShipObstacleSpeeds(TArray<FPCGObstacleSpec>& Catalog)
{
	for (FPCGObstacleSpec& Spec : Catalog) {
		if (!IsShipObstacleClass(Spec.ObstacleClass)) {
			continue;
		}
		Spec.MinSpeed = 0.0f;
		Spec.MaxSpeed = 0.0f;
	}
}

void ResetTransformArrayIfPresent(UObject* Object, const FName PropertyName)
{
	if (TArray<FTransform>* TransformArray = FindTransformArrayProperty(Object, PropertyName)) {
		TransformArray->Reset();
	}
}

void ResetRuntimePathArrays(UObject* Object)
{
	ResetTransformArrayIfPresent(Object, TEXT("road"));
	ResetTransformArrayIfPresent(Object, TEXT("left"));
	ResetTransformArrayIfPresent(Object, TEXT("right"));
	ResetTransformArrayIfPresent(Object, TEXT("temp"));
	ResetTransformArrayIfPresent(Object, TEXT("aleft"));
	ResetTransformArrayIfPresent(Object, TEXT("aright"));
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

// -----------------------------------------------------------------------------
// Advanced PCG helpers: navigation grid + pattern placement.
// -----------------------------------------------------------------------------

constexpr uint8 kCellOutside = 0;
constexpr uint8 kCellFree = 1;
constexpr uint8 kCellBlocked = 2;

struct FNavGrid2D
{
	int32 Width = 0;
	int32 Height = 0;
	float CellSize = 100.0f;
	FVector2D Origin = FVector2D::ZeroVector; // world XY of cell (0,0) center
	TArray<uint8> Cells;

	FORCEINLINE int32 Index(int32 X, int32 Y) const { return Y * Width + X; }
	FORCEINLINE bool InBounds(int32 X, int32 Y) const { return X >= 0 && Y >= 0 && X < Width && Y < Height; }
	FORCEINLINE uint8 Get(int32 X, int32 Y) const { return Cells[Index(X, Y)]; }
	FORCEINLINE void Set(int32 X, int32 Y, uint8 V) { Cells[Index(X, Y)] = V; }
	FORCEINLINE FVector2D CellCenter(int32 X, int32 Y) const { return Origin + FVector2D(X * CellSize, Y * CellSize); }
	FORCEINLINE FIntPoint WorldToCell(const FVector2D& P) const
	{
		return FIntPoint(
			FMath::FloorToInt((P.X - Origin.X) / CellSize + 0.5f),
			FMath::FloorToInt((P.Y - Origin.Y) / CellSize + 0.5f));
	}

	void Initialize(const FVector2D& InOrigin, int32 InWidth, int32 InHeight, float InCellSize)
	{
		Width = FMath::Max(1, InWidth);
		Height = FMath::Max(1, InHeight);
		CellSize = FMath::Max(1.0f, InCellSize);
		Origin = InOrigin;
		Cells.SetNumZeroed(Width * Height);
	}

	// Rasterize navigable channel polygon defined by left[]/right[]/road[].
	void RasterizeChannel(const TArray<FTransform>& Road, const TArray<FTransform>& Left, const TArray<FTransform>& Right)
	{
		const int32 N = FMath::Min(Road.Num(), FMath::Min(Left.Num(), Right.Num()));
		for (int32 i = 0; i + 1 < N; ++i) {
			const FVector2D A2D(Road[i].GetLocation().X, Road[i].GetLocation().Y);
			const FVector2D B2D(Road[i + 1].GetLocation().X, Road[i + 1].GetLocation().Y);
			const float HalfW0 = FVector2D::Distance(
				FVector2D(Left[i].GetLocation().X, Left[i].GetLocation().Y),
				FVector2D(Right[i].GetLocation().X, Right[i].GetLocation().Y)) * 0.5f;
			const float HalfW1 = FVector2D::Distance(
				FVector2D(Left[i + 1].GetLocation().X, Left[i + 1].GetLocation().Y),
				FVector2D(Right[i + 1].GetLocation().X, Right[i + 1].GetLocation().Y)) * 0.5f;
			const float MaxHalf = FMath::Max(HalfW0, HalfW1) + CellSize;

			const float MinX = FMath::Min(A2D.X, B2D.X) - MaxHalf;
			const float MaxX = FMath::Max(A2D.X, B2D.X) + MaxHalf;
			const float MinY = FMath::Min(A2D.Y, B2D.Y) - MaxHalf;
			const float MaxY = FMath::Max(A2D.Y, B2D.Y) + MaxHalf;

			const FIntPoint CellMin = WorldToCell(FVector2D(MinX, MinY));
			const FIntPoint CellMax = WorldToCell(FVector2D(MaxX, MaxY));

			const FVector2D AB = B2D - A2D;
			const float ABLenSq = FMath::Max(AB.SizeSquared(), KINDA_SMALL_NUMBER);

			for (int32 Y = FMath::Max(0, CellMin.Y); Y <= FMath::Min(Height - 1, CellMax.Y); ++Y) {
				for (int32 X = FMath::Max(0, CellMin.X); X <= FMath::Min(Width - 1, CellMax.X); ++X) {
					const FVector2D P = CellCenter(X, Y);
					const float T = FMath::Clamp(FVector2D::DotProduct(P - A2D, AB) / ABLenSq, 0.f, 1.f);
					const FVector2D Q = A2D + T * AB;
					const float HalfW = FMath::Lerp(HalfW0, HalfW1, T);
					if (FVector2D::DistSquared(P, Q) <= HalfW * HalfW) {
						Set(X, Y, kCellFree);
					}
				}
			}
		}
	}

	struct FCellPatch
	{
		int32 X;
		int32 Y;
		uint8 PrevValue;
	};

	// Mark a disk of given radius (plus padding) as blocked. Records overwritten cells so we can roll back.
	int32 MarkDiskBlocked(const FVector2D& Center, float RadiusCm, TArray<FCellPatch>& OutPatches, uint8 Value = kCellBlocked)
	{
		const FIntPoint C = WorldToCell(Center);
		const int32 R = FMath::Max(0, FMath::CeilToInt(RadiusCm / CellSize));
		const float R2 = RadiusCm * RadiusCm;
		int32 Marked = 0;
		for (int32 Dy = -R; Dy <= R; ++Dy) {
			for (int32 Dx = -R; Dx <= R; ++Dx) {
				const int32 X = C.X + Dx;
				const int32 Y = C.Y + Dy;
				if (!InBounds(X, Y)) continue;
				const FVector2D P = CellCenter(X, Y);
				if (FVector2D::DistSquared(P, Center) <= R2) {
					const uint8 Prev = Get(X, Y);
					if (Prev != Value) {
						OutPatches.Add(FCellPatch{X, Y, Prev});
						Set(X, Y, Value);
					}
					++Marked;
				}
			}
		}
		return Marked;
	}

	void RollbackPatches(const TArray<FCellPatch>& Patches)
	{
		for (const FCellPatch& P : Patches) {
			Set(P.X, P.Y, P.PrevValue);
		}
	}

	// BFS with clearance window: a cell is traversable only if every cell in
	// a (2*r+1) window around it is kCellFree. Cheap correctness > clever DT.
	bool HasClearPath(const FVector2D& StartWorld, const FVector2D& GoalWorld, int32 ClearanceCells) const
	{
		const FIntPoint Start = WorldToCell(StartWorld);
		const FIntPoint Goal = WorldToCell(GoalWorld);
		if (!InBounds(Start.X, Start.Y) || !InBounds(Goal.X, Goal.Y)) {
			return false;
		}

		const int32 R = FMath::Max(0, ClearanceCells);
		auto IsWindowFree = [&](int32 X, int32 Y) -> bool {
			for (int32 Dy = -R; Dy <= R; ++Dy) {
				for (int32 Dx = -R; Dx <= R; ++Dx) {
					const int32 Nx = X + Dx;
					const int32 Ny = Y + Dy;
					if (!InBounds(Nx, Ny) || Get(Nx, Ny) != kCellFree) return false;
				}
			}
			return true;
		};

		// Soft-start: allow start/goal cells to traverse even if their window clips walls.
		auto IsWindowTraversable = [&](int32 X, int32 Y, bool bSoft) -> bool {
			if (bSoft) {
				// Require center free + direct 4-neighbors not blocked (obstacles), walls are tolerated near endpoints.
				if (!InBounds(X, Y)) return false;
				if (Get(X, Y) == kCellBlocked) return false;
				return true;
			}
			return IsWindowFree(X, Y);
		};

		TArray<uint8> Visited;
		Visited.SetNumZeroed(Width * Height);
		auto VisIdx = [&](int32 X, int32 Y) { return Y * Width + X; };

		std::queue<FIntPoint> Queue;
		Queue.push(Start);
		Visited[VisIdx(Start.X, Start.Y)] = 1;

		static const int32 DX8[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
		static const int32 DY8[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

		const int32 GoalChebyshev = FMath::Max(FMath::Abs(Start.X - Goal.X), FMath::Abs(Start.Y - Goal.Y));

		while (!Queue.empty()) {
			const FIntPoint Cur = Queue.front();
			Queue.pop();
			if (Cur.X == Goal.X && Cur.Y == Goal.Y) return true;

			for (int32 K = 0; K < 8; ++K) {
				const int32 Nx = Cur.X + DX8[K];
				const int32 Ny = Cur.Y + DY8[K];
				if (!InBounds(Nx, Ny) || Visited[VisIdx(Nx, Ny)]) continue;
				const int32 DistFromStart = FMath::Max(FMath::Abs(Nx - Start.X), FMath::Abs(Ny - Start.Y));
				const int32 DistFromGoal = FMath::Max(FMath::Abs(Nx - Goal.X), FMath::Abs(Ny - Goal.Y));
				// Within R cells of either endpoint, soften the clearance window.
				const bool bSoft = (DistFromStart <= R + 1) || (DistFromGoal <= R + 1);
				if (!IsWindowTraversable(Nx, Ny, bSoft)) continue;
				Visited[VisIdx(Nx, Ny)] = 1;
				Queue.push(FIntPoint(Nx, Ny));
			}
			(void)GoalChebyshev;
		}
		return false;
	}
};

struct FObstacleCandidate
{
	FVector Location = FVector::ZeroVector;
	FRotator Rotation = FRotator::ZeroRotator;
	float FootprintRadius = 0.0f;
	int32 CatalogIndex = INDEX_NONE;
};

EPCGObstaclePattern PickPattern(FRandomStream& Stream, float WSingle, float WGate, float WSlalom, float WCluster)
{
	const float Total = FMath::Max(WSingle + WGate + WSlalom + WCluster, KINDA_SMALL_NUMBER);
	float Roll = Stream.FRandRange(0.0f, Total);
	if ((Roll -= WSingle) <= 0.0f) return EPCGObstaclePattern::Single;
	if ((Roll -= WGate) <= 0.0f) return EPCGObstaclePattern::Gate;
	if ((Roll -= WSlalom) <= 0.0f) return EPCGObstaclePattern::Slalom;
	return EPCGObstaclePattern::Cluster;
}

int32 PickCatalogIndex(const TArray<FPCGObstacleSpec>& Catalog, FRandomStream& Stream, float MaxRadius = -1.0f)
{
	float TotalWeight = 0.0f;
	for (int32 i = 0; i < Catalog.Num(); ++i) {
		if (MaxRadius > 0.0f && Catalog[i].FootprintRadius > MaxRadius) continue;
		TotalWeight += FMath::Max(0.0f, Catalog[i].Weight);
	}
	if (TotalWeight <= KINDA_SMALL_NUMBER) {
		// Fall back to smallest item when MaxRadius filters out everything.
		int32 SmallestIdx = INDEX_NONE;
		float SmallestR = TNumericLimits<float>::Max();
		for (int32 i = 0; i < Catalog.Num(); ++i) {
			if (Catalog[i].FootprintRadius < SmallestR) {
				SmallestR = Catalog[i].FootprintRadius;
				SmallestIdx = i;
			}
		}
		return SmallestIdx;
	}
	float Roll = Stream.FRandRange(0.0f, TotalWeight);
	for (int32 i = 0; i < Catalog.Num(); ++i) {
		if (MaxRadius > 0.0f && Catalog[i].FootprintRadius > MaxRadius) continue;
		const float W = FMath::Max(0.0f, Catalog[i].Weight);
		if ((Roll -= W) <= 0.0f) return i;
	}
	return Catalog.Num() - 1;
}

float ChannelHalfWidthAt(const TArray<FTransform>& Left, const TArray<FTransform>& Right, int32 Index)
{
	if (!Left.IsValidIndex(Index) || !Right.IsValidIndex(Index)) return 0.0f;
	return 0.5f * FVector2D::Distance(
		FVector2D(Left[Index].GetLocation().X, Left[Index].GetLocation().Y),
		FVector2D(Right[Index].GetLocation().X, Right[Index].GetLocation().Y));
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

	InitializeDefaultObstacleCatalog();
}

void AGenerationManager::InitializeDefaultObstacleCatalog()
{
	if (ObstacleCatalog.Num() > 0) {
		return;
	}

	auto AppendSpec = [&](const TCHAR* Path, float Radius, float Weight, float MinSpeed, float MaxSpeed, bool bVelocity) {
		FPCGObstacleSpec Spec;
		Spec.ObstacleClass = TSoftClassPtr<AActor>(FSoftObjectPath(FString(Path) + TEXT("_C")));
		Spec.FootprintRadius = Radius;
		Spec.Weight = Weight;
		Spec.MinSpeed = MinSpeed;
		Spec.MaxSpeed = MaxSpeed;
		Spec.bSupportsVelocityProperties = bVelocity;
		ObstacleCatalog.Add(Spec);
	};

	AppendSpec(TEXT("/AirSim/Blueprints/BP_BuoySpawn.BP_BuoySpawn"), 80.0f, 0.40f, 0.0f, 0.0f, false);
	AppendSpec(TEXT("/AirSim/Blueprints/Boat_Blueprint.Boat_Blueprint"), 300.0f, 0.30f, 0.0f, 0.0f, true);
	AppendSpec(TEXT("/AirSim/Blueprints/ShippingSim/CargoVessel/Barge_Blueprint.Barge_Blueprint"), 700.0f, 0.20f, 0.0f, 0.0f, true);
	AppendSpec(TEXT("/AirSim/Blueprints/ShippingSim/CargoVessel/BP_CargoPawn.BP_CargoPawn"), 900.0f, 0.10f, 0.0f, 0.0f, true);
	DisableDynamicShipObstacleSpeeds(ObstacleCatalog);
}

// Called when the game starts or when spawned
void AGenerationManager::BeginPlay()
{
	Super::BeginPlay();
}

bool AGenerationManager::HavenStep(TArray<FTransform>& haven, float mina, float maxa, float mind, float maxd)
{
	return true;
}
void AGenerationManager::WallStep(TArray<FTransform>& haven, TArray<FTransform>& wall, float min, float max)
{
}
void AGenerationManager::GenerateWalls(TArray<FTransform>& points, bool inlets, float min, float max)
{
}
void AGenerationManager::InletFunction(FTransform begin, FTransform end, FRotator rotation, float min, float max, float length)
{
}
void AGenerationManager::InletGeneration(TArray<FTransform>& points)
{
}
void AGenerationManager::Clear()
{
}

int32 AGenerationManager::CleanupGeneratedActorsNative(AActor* GenerationManagerActor, bool bResetPathData)
{
	if (!GenerationManagerActor) {
		return 0;
	}

	int32 DestroyedCount = 0;
	if (TArray<AActor*>* GeneratedActors = FindGeneratedActorArrayProperty(GenerationManagerActor)) {
		for (AActor*& SpawnedActor : *GeneratedActors) {
			if (IsValid(SpawnedActor) && !SpawnedActor->IsActorBeingDestroyed()) {
				SpawnedActor->SetActorEnableCollision(false);
				SpawnedActor->SetActorHiddenInGame(true);
				SpawnedActor->Destroy();
				++DestroyedCount;
			}
			SpawnedActor = nullptr;
		}
		GeneratedActors->Reset();
	}

	if (bResetPathData) {
		ResetRuntimePathArrays(GenerationManagerActor);
	}

	return DestroyedCount;
}

int32 AGenerationManager::GetGeneratedActorCountNative(const AActor* GenerationManagerActor)
{
	if (!GenerationManagerActor) {
		return 0;
	}

	const TArray<AActor*>* GeneratedActors = FindGeneratedActorArrayProperty(const_cast<AActor*>(GenerationManagerActor));
	if (!GeneratedActors) {
		return 0;
	}

	int32 LiveCount = 0;
	for (AActor* SpawnedActor : *GeneratedActors) {
		if (IsValid(SpawnedActor) && !SpawnedActor->IsActorBeingDestroyed()) {
			++LiveCount;
		}
	}
	return LiveCount;
}

void AGenerationManager::SetSeed(int32 seed)
{
	CachedSeed = seed;

	if (FIntProperty* SeedProperty = FindIntProperty(this, TEXT("Seed"))) {
		SeedProperty->SetPropertyValue_InContainer(this, seed);
	}
	else if (FIntProperty* LegacySeedProperty = FindIntProperty(this, TEXT("aseed"))) {
		LegacySeedProperty->SetPropertyValue_InContainer(this, seed);
	}

	if (FRandomStream* StreamProperty = FindRandomStreamProperty(this, TEXT("stream"))) {
		StreamProperty->Initialize(seed);
	}

	UE_LOG(LogTemp, Warning, TEXT("GenerationManager seed set to %d."), seed);
}
void AGenerationManager::generateTerrain(FString type, float mina, float maxa, float mind, float maxd, int length)
{
	UE_LOG(LogTemp, Warning, TEXT("GenerationManager base generateTerrain called with type=%s length=%d."), *type, length);
}
void AGenerationManager::getGoal(int32 distance, FVector& location, FVector& left, FVector& right)
{
	UE_LOG(LogTemp, Warning, TEXT("GenerationManager base getGoal called for distance=%d."), distance);
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

	FRandomStream* StreamProperty = FindRandomStreamProperty(this, TEXT("stream"));
	FRandomStream FallbackStream(CachedSeed);
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
			double ObstacleSpeed = (SpeedMax > SpeedMin)
				? ObstacleStream.FRandRange(SpeedMin, SpeedMax)
				: SpeedMin;
			if (IsShipObstacleClass(ObstacleClass)) {
				ObstacleSpeed = 0.0;
			}

			SetBoolProperty(SpawnedObstacle, TEXT("CustomVelocity"), true);
			SetBoolProperty(SpawnedObstacle, TEXT("ApplyVelocityOnBeginPlay"), false);
			SetRealProperty(SpawnedObstacle, TEXT("Speed"), ObstacleSpeed);

			UGameplayStatics::FinishSpawningActor(SpawnedObstacle, SpawnTransform);
			if (GeneratedActors) {
				GeneratedActors->Add(SpawnedObstacle);
			}
			++SpawnedCount;
		}

		CurrentIndex += ObstacleStream.RandRange(StepMin, StepMax);
		++SafetyCounter;
	}

	UE_LOG(LogTemp, Warning, TEXT("GenerationManager spawned %d obstacle actors."), SpawnedCount);
}

// -----------------------------------------------------------------------------
// RefineChannelGeometry: Chaikin-smooth the spine, arc-length-resample road/left
// /right at a common count, then optionally apply a bounded coherent chicane.
// The original BP-generated walls stay valid because left/right are resampled
// from the original arrays (never extended outwards).
// -----------------------------------------------------------------------------
int32 AGenerationManager::RefineChannelGeometry()
{
	TArray<FTransform>* RoadPoints = FindTransformArrayProperty(this, TEXT("road"));
	TArray<FTransform>* LeftPoints = FindTransformArrayProperty(this, TEXT("left"));
	if (!LeftPoints) LeftPoints = FindTransformArrayProperty(this, TEXT("aleft"));
	TArray<FTransform>* RightPoints = FindTransformArrayProperty(this, TEXT("right"));
	if (!RightPoints) RightPoints = FindTransformArrayProperty(this, TEXT("aright"));

	if (!RoadPoints || !LeftPoints || !RightPoints) {
		UE_LOG(LogTemp, Warning, TEXT("RefineChannelGeometry: road/left/right properties missing."));
		return 0;
	}
	const int32 OrigN = FMath::Min(RoadPoints->Num(), FMath::Min(LeftPoints->Num(), RightPoints->Num()));
	if (OrigN < 2) {
		UE_LOG(LogTemp, Warning, TEXT("RefineChannelGeometry: need >= 2 aligned points, have %d."), OrigN);
		return 0;
	}

	FRandomStream* StreamProperty = FindRandomStreamProperty(this, TEXT("stream"));
	FRandomStream FallbackStream(CachedSeed);
	FRandomStream& Stream = StreamProperty ? *StreamProperty : FallbackStream;

	// --- Step 1. Extract XY + Z for each aligned triple.
	TArray<FVector2D> Road2D, Left2D, Right2D;
	TArray<float> RoadZ;
	Road2D.Reserve(OrigN);
	Left2D.Reserve(OrigN);
	Right2D.Reserve(OrigN);
	RoadZ.Reserve(OrigN);
	for (int32 i = 0; i < OrigN; ++i) {
		const FVector R = (*RoadPoints)[i].GetLocation();
		const FVector L = (*LeftPoints)[i].GetLocation();
		const FVector Rt = (*RightPoints)[i].GetLocation();
		Road2D.Add(FVector2D(R.X, R.Y));
		Left2D.Add(FVector2D(L.X, L.Y));
		Right2D.Add(FVector2D(Rt.X, Rt.Y));
		RoadZ.Add(R.Z);
	}

	auto CumulativeLen = [](const TArray<FVector2D>& Ps) -> TArray<float> {
		TArray<float> C;
		C.SetNumZeroed(Ps.Num());
		for (int32 i = 1; i < Ps.Num(); ++i) {
			C[i] = C[i - 1] + FVector2D::Distance(Ps[i - 1], Ps[i]);
		}
		return C;
	};

	auto ResampleAtU = [](const TArray<FVector2D>& Ps, const TArray<float>& Cum, float U) -> FVector2D {
		const float Total = FMath::Max(Cum.Last(), KINDA_SMALL_NUMBER);
		const float Target = FMath::Clamp(U, 0.0f, 1.0f) * Total;
		int32 Seg = 0;
		while (Seg + 1 < Ps.Num() - 1 && Cum[Seg + 1] < Target) ++Seg;
		const int32 Next = FMath::Min(Seg + 1, Ps.Num() - 1);
		const float SegLen = FMath::Max(Cum[Next] - Cum[Seg], KINDA_SMALL_NUMBER);
		const float T = FMath::Clamp((Target - Cum[Seg]) / SegLen, 0.0f, 1.0f);
		return FMath::Lerp(Ps[Seg], Ps[Next], T);
	};

	// --- Step 2. Chaikin-smooth road only, keeping endpoints.
	TArray<FVector2D> SmoothRoad = Road2D;
	for (int32 Iter = 0; Iter < SpineChaikinIter; ++Iter) {
		if (SmoothRoad.Num() < 3) break;
		TArray<FVector2D> Next;
		Next.Reserve(SmoothRoad.Num() * 2);
		Next.Add(SmoothRoad[0]);
		for (int32 i = 0; i + 1 < SmoothRoad.Num(); ++i) {
			const FVector2D Q = 0.75f * SmoothRoad[i] + 0.25f * SmoothRoad[i + 1];
			const FVector2D R = 0.25f * SmoothRoad[i] + 0.75f * SmoothRoad[i + 1];
			Next.Add(Q);
			Next.Add(R);
		}
		Next.Add(SmoothRoad.Last());
		SmoothRoad = MoveTemp(Next);
	}

	const TArray<float> RoadCum = CumulativeLen(SmoothRoad);
	const TArray<float> LeftCum = CumulativeLen(Left2D);
	const TArray<float> RightCum = CumulativeLen(Right2D);
	const float RoadTotal = RoadCum.Last();
	if (RoadTotal <= SpineResampleCm) {
		UE_LOG(LogTemp, Warning, TEXT("RefineChannelGeometry: length %.1f cm too short; skipping."), RoadTotal);
		return 0;
	}

	// --- Step 3. PRESERVE array length = OrigN.
	// The Blueprint's getGoal(distance) is a direct index into road/left/right,
	// so `distance=k` must still refer to the same "k-th segment" after refinement.
	// Chaikin smoothing + arc-length resample still improve curvature/uniformity.
	const int32 NumSamples = OrigN;

	// --- Step 4. Resample road (Chaikin-smoothed), left, right at matching U in [0,1].
	TArray<FVector2D> UR; UR.SetNumUninitialized(NumSamples);
	TArray<FVector2D> UL; UL.SetNumUninitialized(NumSamples);
	TArray<FVector2D> URt; URt.SetNumUninitialized(NumSamples);
	for (int32 i = 0; i < NumSamples; ++i) {
		const float U = static_cast<float>(i) / static_cast<float>(NumSamples - 1);
		UR[i] = ResampleAtU(SmoothRoad, RoadCum, U);
		UL[i] = ResampleAtU(Left2D, LeftCum, U);
		URt[i] = ResampleAtU(Right2D, RightCum, U);
	}

	// --- Step 5. Compute rotators from neighboring road samples.
	auto ComputeRotators = [&](const TArray<FVector2D>& Ps, TArray<FRotator>& OutRot) {
		OutRot.SetNumUninitialized(Ps.Num());
		for (int32 i = 0; i < Ps.Num(); ++i) {
			FVector2D Dir;
			if (i == 0) Dir = Ps[1] - Ps[0];
			else if (i == Ps.Num() - 1) Dir = Ps.Last() - Ps[Ps.Num() - 2];
			else Dir = Ps[i + 1] - Ps[i - 1];
			if (Dir.IsNearlyZero()) Dir = FVector2D(1.0f, 0.0f);
			Dir.Normalize();
			OutRot[i] = FRotator(0.0f, FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X)), 0.0f);
		}
	};
	TArray<FRotator> Rotators;
	ComputeRotators(UR, Rotators);

	// --- Step 6. Optional coherent chicane, amplitude bounded so road stays
	// strictly inside the local corridor (between UL[i] and URt[i]).
	const float ChicaneRoll = Stream.FRand();
	const float ChicaneThresh = ChicaneProbability * (0.5f + 0.5f * DifficultyLevel);
	const bool bChicane = (ChicaneRoll < ChicaneThresh && NumSamples > 6);
	if (bChicane) {
		const float K = Stream.FRandRange(2.0f, 4.0f);
		const float ChicPhase = Stream.FRandRange(0.0f, 2.0f * PI);
		const int32 StartIdx = 2;
		const int32 EndIdx = NumSamples - 3;
		for (int32 i = StartIdx; i <= EndIdx; ++i) {
			const float U = static_cast<float>(i - StartIdx) / FMath::Max(1, EndIdx - StartIdx);
			const float Taper = FMath::Sin(PI * U);
			const FVector RightVec = Rotators[i].RotateVector(FVector::RightVector);
			const FVector2D RightXY(RightVec.X, RightVec.Y);
			const float DistLeft = FVector2D::Distance(UR[i], UL[i]);
			const float DistRight = FVector2D::Distance(UR[i], URt[i]);
			const float LocalBudget = FMath::Max(0.0f, FMath::Min(DistLeft, DistRight) - VesselBeamCm * ClearanceMultiple * 0.5f);
			const float Amp = FMath::Min(ChicaneLateralRatio * LocalBudget, LocalBudget) * Taper;
			const float Offset = Amp * FMath::Sin(2.0f * PI * K * U + ChicPhase);
			UR[i] += RightXY * Offset;
		}
		ComputeRotators(UR, Rotators);
	}

	// --- Step 7. Optionally tighten width toward ChannelWidthMinCm if the
	// original corridor is considerably wider. Never widen.
	const float MinHalf = FMath::Max(100.0f, ChannelWidthMinCm * 0.5f);
	const float MaxHalf = FMath::Max(MinHalf + 100.0f, ChannelWidthMaxCm * 0.5f);
	const float Phase = Stream.FRandRange(0.0f, 2.0f * PI);
	const float FreqPerIndex = ChannelWidthFreq * (1.0f + 0.5f * DifficultyLevel);
	for (int32 i = 0; i < NumSamples; ++i) {
		const FVector RightVec = Rotators[i].RotateVector(FVector::RightVector);
		const FVector2D RightXY(RightVec.X, RightVec.Y);
		const float U = static_cast<float>(i) / static_cast<float>(NumSamples - 1);
		const float S = 0.5f * (1.0f + FMath::Sin(2.0f * PI * FreqPerIndex * U + Phase));
		const float TargetHalf = FMath::Lerp(MinHalf, MaxHalf, S);
		const float CurrentLeft = FVector2D::Distance(UR[i], UL[i]);
		const float CurrentRight = FVector2D::Distance(UR[i], URt[i]);
		// Pull each side inward only when it is wider than TargetHalf.
		if (CurrentLeft > TargetHalf) {
			UL[i] = UR[i] - RightXY * TargetHalf;
		}
		if (CurrentRight > TargetHalf) {
			URt[i] = UR[i] + RightXY * TargetHalf;
		}
	}

	// --- Step 8. Write all three arrays back.
	RoadPoints->Reset();
	LeftPoints->Reset();
	RightPoints->Reset();
	RoadPoints->Reserve(NumSamples);
	LeftPoints->Reserve(NumSamples);
	RightPoints->Reserve(NumSamples);
	const float ZStart = RoadZ.Num() > 0 ? RoadZ[0] : 0.0f;
	const float ZEnd = RoadZ.Num() > 0 ? RoadZ.Last() : 0.0f;
	for (int32 i = 0; i < NumSamples; ++i) {
		const float U = static_cast<float>(i) / static_cast<float>(NumSamples - 1);
		const float Z = FMath::Lerp(ZStart, ZEnd, U);
		RoadPoints->Add(FTransform(Rotators[i], FVector(UR[i].X, UR[i].Y, Z)));
		LeftPoints->Add(FTransform(Rotators[i], FVector(UL[i].X, UL[i].Y, Z)));
		RightPoints->Add(FTransform(Rotators[i], FVector(URt[i].X, URt[i].Y, Z)));
	}

	UE_LOG(LogTemp, Warning, TEXT("RefineChannelGeometry: in=%d out=%d length=%.1fcm chicane=%s."),
		OrigN, NumSamples, RoadTotal, bChicane ? TEXT("true") : TEXT("false"));
	return NumSamples;
}

// -----------------------------------------------------------------------------
// SpawnAdvancedObstacles: structured pattern placement guarded by occupancy
// grid BFS with clearance >= VesselBeamCm * ClearanceMultiple.
// -----------------------------------------------------------------------------
int32 AGenerationManager::SpawnAdvancedObstacles()
{
	if (!bSpawnObstacles) {
		UE_LOG(LogTemp, Verbose, TEXT("SpawnAdvancedObstacles: disabled."));
		return 0;
	}

	UWorld* World = GetWorld();
	if (!World) return 0;

	TArray<FTransform>* RoadPoints = FindTransformArrayProperty(this, TEXT("road"));
	TArray<FTransform>* LeftPoints = FindTransformArrayProperty(this, TEXT("left"));
	if (!LeftPoints) LeftPoints = FindTransformArrayProperty(this, TEXT("aleft"));
	TArray<FTransform>* RightPoints = FindTransformArrayProperty(this, TEXT("right"));
	if (!RightPoints) RightPoints = FindTransformArrayProperty(this, TEXT("aright"));
	TArray<AActor*>* GeneratedActors = FindActorArrayProperty(this, TEXT("Generated"));
	if (!GeneratedActors) GeneratedActors = FindActorArrayProperty(this, TEXT("generated"));

	if (!RoadPoints || !LeftPoints || !RightPoints) {
		UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstacles: road/left/right missing."));
		return 0;
	}

	const int32 N = FMath::Min(RoadPoints->Num(), FMath::Min(LeftPoints->Num(), RightPoints->Num()));
	if (N < 4) {
		UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstacles: path too short (%d)."), N);
		return 0;
	}

	if (ObstacleCatalog.Num() == 0) {
		InitializeDefaultObstacleCatalog();
	}
	DisableDynamicShipObstacleSpeeds(ObstacleCatalog);

	FRandomStream* StreamProperty = FindRandomStreamProperty(this, TEXT("stream"));
	FRandomStream FallbackStream(CachedSeed);
	FRandomStream& Stream = StreamProperty ? *StreamProperty : FallbackStream;

	// --- Compute channel AABB + build nav grid.
	FVector2D AABBMin(TNumericLimits<float>::Max(), TNumericLimits<float>::Max());
	FVector2D AABBMax(-TNumericLimits<float>::Max(), -TNumericLimits<float>::Max());
	float TotalLenCm = 0.0f;
	for (int32 i = 0; i < N; ++i) {
		const FVector2D L2D((*LeftPoints)[i].GetLocation().X, (*LeftPoints)[i].GetLocation().Y);
		const FVector2D R2D((*RightPoints)[i].GetLocation().X, (*RightPoints)[i].GetLocation().Y);
		AABBMin = FVector2D(FMath::Min(AABBMin.X, FMath::Min(L2D.X, R2D.X)), FMath::Min(AABBMin.Y, FMath::Min(L2D.Y, R2D.Y)));
		AABBMax = FVector2D(FMath::Max(AABBMax.X, FMath::Max(L2D.X, R2D.X)), FMath::Max(AABBMax.Y, FMath::Max(L2D.Y, R2D.Y)));
		if (i + 1 < N) {
			TotalLenCm += FVector2D::Distance(
				FVector2D((*RoadPoints)[i].GetLocation().X, (*RoadPoints)[i].GetLocation().Y),
				FVector2D((*RoadPoints)[i + 1].GetLocation().X, (*RoadPoints)[i + 1].GetLocation().Y));
		}
	}

	const float Padding = FMath::Max(GridCellSizeCm * 2.0f, 200.0f);
	AABBMin -= FVector2D(Padding, Padding);
	AABBMax += FVector2D(Padding, Padding);

	FNavGrid2D Grid;
	const int32 GW = FMath::CeilToInt((AABBMax.X - AABBMin.X) / GridCellSizeCm) + 1;
	const int32 GH = FMath::CeilToInt((AABBMax.Y - AABBMin.Y) / GridCellSizeCm) + 1;
	if (GW <= 0 || GH <= 0 || GW > 4096 || GH > 4096) {
		UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstacles: invalid grid %dx%d, skipping."), GW, GH);
		return 0;
	}
	Grid.Initialize(AABBMin, GW, GH, GridCellSizeCm);
	Grid.RasterizeChannel(*RoadPoints, *LeftPoints, *RightPoints);

	const FVector2D StartXY((*RoadPoints)[0].GetLocation().X, (*RoadPoints)[0].GetLocation().Y);
	const FVector2D GoalXY((*RoadPoints)[N - 1].GetLocation().X, (*RoadPoints)[N - 1].GetLocation().Y);
	const int32 ClearanceCells = FMath::Max(1, FMath::CeilToInt((VesselBeamCm * ClearanceMultiple) / (2.0f * GridCellSizeCm)));

	if (!Grid.HasClearPath(StartXY, GoalXY, ClearanceCells)) {
		UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstacles: base channel itself fails clearance BFS (cells=%d); aborting obstacle placement."), ClearanceCells);
		return 0;
	}

	// --- Target count.
	const float ChannelLenM = TotalLenCm / 100.0f;
	const float DifficultyFactor = 0.4f + 1.2f * DifficultyLevel;
	int32 TargetCount = FMath::Clamp(
		FMath::RoundToInt(ObstaclesPerKm * (ChannelLenM / 1000.0f) * DifficultyFactor),
		0, FMath::Max(0, ObstacleHardCap));

	// --- Pre-load catalog classes once.
	TArray<UClass*> CatalogClasses;
	CatalogClasses.SetNum(ObstacleCatalog.Num());
	for (int32 i = 0; i < ObstacleCatalog.Num(); ++i) {
		CatalogClasses[i] = ObstacleCatalog[i].ObstacleClass.IsValid()
			? ObstacleCatalog[i].ObstacleClass.Get()
			: ObstacleCatalog[i].ObstacleClass.LoadSynchronous();
	}

	// --- Pattern candidate generators.
	auto MakeSingle = [&](int32 Index, int32 CatIdx, float HalfW) -> FObstacleCandidate {
		FObstacleCandidate C;
		const FTransform& Road = (*RoadPoints)[Index];
		const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
		const float Margin = FMath::Max(0.0f, HalfW - ObstacleBoundaryPadding - ObstacleCatalog[CatIdx].FootprintRadius);
		const float LateralOffset = Stream.FRandRange(-Margin, Margin);
		C.Location = Road.GetLocation() + RightVec * LateralOffset;
		C.Location.Z += ObstacleZOffset;
		C.Rotation = Road.Rotator();
		C.Rotation.Yaw += Stream.FRandRange(-ObstacleYawJitter, ObstacleYawJitter);
		C.FootprintRadius = ObstacleCatalog[CatIdx].FootprintRadius;
		C.CatalogIndex = CatIdx;
		return C;
	};

	auto TryCommitCandidates = [&](TArray<FObstacleCandidate>& Candidates) -> bool {
		TArray<FNavGrid2D::FCellPatch> Patches;
		Patches.Reserve(Candidates.Num() * 64);
		for (const FObstacleCandidate& C : Candidates) {
			const float EffectiveR = C.FootprintRadius + ClearanceCells * GridCellSizeCm * 0.25f;
			Grid.MarkDiskBlocked(FVector2D(C.Location.X, C.Location.Y), EffectiveR, Patches);
		}
		if (Grid.HasClearPath(StartXY, GoalXY, ClearanceCells)) {
			return true;
		}
		Grid.RollbackPatches(Patches);
		return false;
	};

	auto SpawnOne = [&](const FObstacleCandidate& C) -> AActor* {
		UClass* SpawnClass = (ObstacleCatalog.IsValidIndex(C.CatalogIndex)) ? CatalogClasses[C.CatalogIndex] : nullptr;
		if (!SpawnClass) return nullptr;
		const FTransform SpawnT(C.Rotation, C.Location);
		AActor* Actor = World->SpawnActorDeferred<AActor>(
			SpawnClass, SpawnT, this, nullptr,
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn);
		if (!Actor) return nullptr;

		const FPCGObstacleSpec& Spec = ObstacleCatalog[C.CatalogIndex];
		if (Spec.bSupportsVelocityProperties) {
			const float Lo = FMath::Min(Spec.MinSpeed, Spec.MaxSpeed);
			const float Hi = FMath::Max(Spec.MinSpeed, Spec.MaxSpeed);
			const double Speed = (Hi > Lo) ? Stream.FRandRange(Lo, Hi) : Lo;
			SetBoolProperty(Actor, TEXT("CustomVelocity"), true);
			SetBoolProperty(Actor, TEXT("ApplyVelocityOnBeginPlay"), false);
			SetRealProperty(Actor, TEXT("Speed"), Speed);
		}
		UGameplayStatics::FinishSpawningActor(Actor, SpawnT);
		if (GeneratedActors) GeneratedActors->Add(Actor);
		return Actor;
	};

	// --- Iterate spine, pick patterns, guard with BFS.
	const int32 StartIdxPath = FMath::Clamp(ObstacleStartBuffer, 0, N - 1);
	const int32 EndIdxPath = FMath::Clamp(N - 1 - ObstacleEndBuffer, 0, N - 1);

	int32 Spawned = 0;
	int32 ConsecFails = 0;
	int32 Cursor = StartIdxPath + Stream.RandRange(0, FMath::Max(0, ObstacleMaxIndexStep - 1));

	while (Spawned < TargetCount && Cursor <= EndIdxPath && ConsecFails < MaxPlacementRetries) {
		const EPCGObstaclePattern Pattern = PickPattern(Stream,
			PatternWeightSingle, PatternWeightGate, PatternWeightSlalom, PatternWeightCluster);
		const float HalfW = ChannelHalfWidthAt(*LeftPoints, *RightPoints, Cursor);

		// Enforce the original min width invariant; skip if this section is too narrow.
		if (HalfW * 2.0f < ObstacleMinChannelWidth) {
			Cursor += Stream.RandRange(FMath::Max(1, ObstacleMinIndexStep), FMath::Max(1, ObstacleMaxIndexStep));
			continue;
		}

		TArray<FObstacleCandidate> Candidates;

		// Cap each obstacle radius so that after placement the residual half-corridor
		// is at least VesselBeam * ClearanceMultiple / 2 on each side.
		const float MaxAllowedRadius = FMath::Max(50.0f, HalfW - VesselBeamCm * ClearanceMultiple * 0.5f - ObstacleBoundaryPadding);

		switch (Pattern) {
			case EPCGObstaclePattern::Single: {
				const int32 CatIdx = PickCatalogIndex(ObstacleCatalog, Stream, MaxAllowedRadius);
				if (CatIdx != INDEX_NONE) Candidates.Add(MakeSingle(Cursor, CatIdx, HalfW));
				break;
			}
			case EPCGObstaclePattern::Gate: {
				// Two obstacles on opposite sides; leave gap >= 2*VesselBeam*ClearanceMultiple.
				const float Gap = VesselBeamCm * ClearanceMultiple * 1.2f;
				const float SideBudget = FMath::Max(50.0f, HalfW - Gap * 0.5f - ObstacleBoundaryPadding);
				const int32 LeftCat = PickCatalogIndex(ObstacleCatalog, Stream, SideBudget);
				const int32 RightCat = PickCatalogIndex(ObstacleCatalog, Stream, SideBudget);
				if (LeftCat != INDEX_NONE && RightCat != INDEX_NONE) {
					const FTransform& Road = (*RoadPoints)[Cursor];
					const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
					const float OffsetL = -(Gap * 0.5f + ObstacleCatalog[LeftCat].FootprintRadius);
					const float OffsetR = +(Gap * 0.5f + ObstacleCatalog[RightCat].FootprintRadius);

					FObstacleCandidate L; L.Location = Road.GetLocation() + RightVec * OffsetL;
					L.Location.Z += ObstacleZOffset; L.Rotation = Road.Rotator();
					L.FootprintRadius = ObstacleCatalog[LeftCat].FootprintRadius; L.CatalogIndex = LeftCat;
					FObstacleCandidate R; R.Location = Road.GetLocation() + RightVec * OffsetR;
					R.Location.Z += ObstacleZOffset; R.Rotation = Road.Rotator();
					R.FootprintRadius = ObstacleCatalog[RightCat].FootprintRadius; R.CatalogIndex = RightCat;
					Candidates.Add(L); Candidates.Add(R);
				}
				break;
			}
			case EPCGObstaclePattern::Slalom: {
				const int32 CountS = Stream.RandRange(3, 5);
				int32 Idx = Cursor;
				int32 Sign = (Stream.FRand() < 0.5f) ? -1 : 1;
				for (int32 s = 0; s < CountS && Idx <= EndIdxPath; ++s) {
					const float IdxHalfW = ChannelHalfWidthAt(*LeftPoints, *RightPoints, Idx);
					const float SideBudget = FMath::Max(50.0f, IdxHalfW - VesselBeamCm * ClearanceMultiple * 0.5f - ObstacleBoundaryPadding);
					const int32 CatIdx = PickCatalogIndex(ObstacleCatalog, Stream, SideBudget);
					if (CatIdx == INDEX_NONE) break;
					const FTransform& Road = (*RoadPoints)[Idx];
					const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
					const float Offset = Sign * (SideBudget * 0.6f + ObstacleCatalog[CatIdx].FootprintRadius);
					FObstacleCandidate C; C.Location = Road.GetLocation() + RightVec * Offset;
					C.Location.Z += ObstacleZOffset; C.Rotation = Road.Rotator();
					C.FootprintRadius = ObstacleCatalog[CatIdx].FootprintRadius; C.CatalogIndex = CatIdx;
					Candidates.Add(C);
					Sign = -Sign;
					Idx += FMath::Max(1, ObstacleMinIndexStep + Stream.RandRange(0, 1));
				}
				Cursor = FMath::Max(Cursor, Idx - 1);
				break;
			}
			case EPCGObstaclePattern::Cluster: {
				const int32 CountC = Stream.RandRange(2, 3);
				for (int32 s = 0; s < CountC; ++s) {
					// Cluster biases to small obstacles (buoys) for realism.
					const int32 CatIdx = PickCatalogIndex(ObstacleCatalog, Stream, FMath::Min(MaxAllowedRadius, 200.0f));
					if (CatIdx == INDEX_NONE) break;
					const FTransform& Road = (*RoadPoints)[Cursor];
					const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
					const FVector FwdVec = Road.Rotator().Vector();
					const float Lat = Stream.FRandRange(-HalfW * 0.6f, HalfW * 0.6f);
					const float Fwd = Stream.FRandRange(-ObstacleForwardJitter * 2.0f, ObstacleForwardJitter * 2.0f);
					FObstacleCandidate C; C.Location = Road.GetLocation() + RightVec * Lat + FwdVec * Fwd;
					C.Location.Z += ObstacleZOffset; C.Rotation = Road.Rotator();
					C.Rotation.Yaw += Stream.FRandRange(-ObstacleYawJitter, ObstacleYawJitter);
					C.FootprintRadius = ObstacleCatalog[CatIdx].FootprintRadius; C.CatalogIndex = CatIdx;
					Candidates.Add(C);
				}
				break;
			}
		}

		if (Candidates.Num() == 0) {
			Cursor += Stream.RandRange(FMath::Max(1, ObstacleMinIndexStep), FMath::Max(1, ObstacleMaxIndexStep));
			++ConsecFails;
			continue;
		}

		// Enforce hard cap strictly: if this pattern would overshoot the remaining
		// budget (either TargetCount or ObstacleHardCap), truncate the batch.
		const int32 RemainingBudget = FMath::Max(0, TargetCount - Spawned);
		if (RemainingBudget <= 0) {
			break;
		}
		if (Candidates.Num() > RemainingBudget) {
			Candidates.SetNum(RemainingBudget, EAllowShrinking::No);
		}

		// Try: if BFS fails, progressively shrink candidate radii by downgrading class.
		bool bCommitted = TryCommitCandidates(Candidates);
		if (!bCommitted) {
			// Downgrade: replace each candidate with smallest feasible catalog item.
			int32 SmallestIdx = INDEX_NONE;
			float SmallestR = TNumericLimits<float>::Max();
			for (int32 i = 0; i < ObstacleCatalog.Num(); ++i) {
				if (ObstacleCatalog[i].FootprintRadius < SmallestR) {
					SmallestR = ObstacleCatalog[i].FootprintRadius;
					SmallestIdx = i;
				}
			}
			if (SmallestIdx != INDEX_NONE) {
				for (FObstacleCandidate& C : Candidates) {
					C.CatalogIndex = SmallestIdx;
					C.FootprintRadius = ObstacleCatalog[SmallestIdx].FootprintRadius;
				}
				bCommitted = TryCommitCandidates(Candidates);
			}
		}

		if (bCommitted) {
			for (const FObstacleCandidate& C : Candidates) {
				if (SpawnOne(C)) ++Spawned;
			}
			ConsecFails = 0;
		} else {
			++ConsecFails;
		}

		Cursor += Stream.RandRange(FMath::Max(1, ObstacleMinIndexStep), FMath::Max(2, ObstacleMaxIndexStep));
	}

	UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstacles: spawned=%d target=%d channelLen=%.1fm difficulty=%.2f clearanceCells=%d."),
		Spawned, TargetCount, ChannelLenM, DifficultyLevel, ClearanceCells);
	return Spawned;
}

// -----------------------------------------------------------------------------
// ApplyAdvancedPCGNative: operates on ANY actor that exposes road/left/right
// /Generated/stream via reflection. Uses hardcoded defaults so it works even
// when the actor is a pure-BP subclass of AActor (not AGenerationManager).
// -----------------------------------------------------------------------------
namespace
{
struct FPCGNativeConfig
{
	bool bSpawnObstacles = true;
	// Density / difficulty
	float DifficultyLevel = 0.5f;
	float ObstaclesPerKm = 12.0f;
	int32 ObstacleHardCap = 40;
	// Navigability — ClearanceMultiple=1.5 leaves ~4.5m residual corridor with
	// VesselBeamCm=300 (MilliAmpere-class ASV), which is a realistic pilotable
	// width while allowing many obstacles to still pass BFS.
	float VesselBeamCm = 300.0f;
	float ClearanceMultiple = 1.5f;
	float GridCellSizeCm = 100.0f;
	int32 MaxPlacementRetries = 40;
	// Channel width profile (only narrows, never widens beyond original)
	float ChannelWidthMinCm = 1400.0f;
	float ChannelWidthMaxCm = 3200.0f;
	float ChannelWidthFreq = 0.8f;
	int32 SpineChaikinIter = 2;
	float ChicaneProbability = 0.25f;
	float ChicaneLateralRatio = 0.4f;
	// Pattern weights
	float PatternWeightSingle = 0.35f;
	float PatternWeightGate = 0.30f;
	float PatternWeightSlalom = 0.25f;
	float PatternWeightCluster = 0.10f;
	// Placement misc
	float ObstacleBoundaryPadding = 200.0f;
	float ObstacleForwardJitter = 250.0f;
	float ObstacleYawJitter = 20.0f;
	float ObstacleZOffset = 30.0f;
	float ObstacleMinChannelWidth = 800.0f;
	int32 ObstacleStartBuffer = 2;
	int32 ObstacleEndBuffer = 2;
	int32 ObstacleMinIndexStep = 1;
	int32 ObstacleMaxIndexStep = 2;
};

TArray<FPCGObstacleSpec> LoadDefaultNativeCatalog()
{
	TArray<FPCGObstacleSpec> Catalog;
	auto AddSpec = [&](const TCHAR* Path, float Radius, float Weight, float SMin, float SMax, bool bVel) {
		FPCGObstacleSpec Spec;
		Spec.ObstacleClass = TSoftClassPtr<AActor>(FSoftObjectPath(FString(Path) + TEXT("_C")));
		Spec.FootprintRadius = Radius;
		Spec.Weight = Weight;
		Spec.MinSpeed = SMin;
		Spec.MaxSpeed = SMax;
		Spec.bSupportsVelocityProperties = bVel;
		Catalog.Add(Spec);
	};
	AddSpec(TEXT("/AirSim/Blueprints/BP_BuoySpawn.BP_BuoySpawn"), 80.0f, 0.40f, 0.0f, 0.0f, false);
	AddSpec(TEXT("/AirSim/Blueprints/Boat_Blueprint.Boat_Blueprint"), 300.0f, 0.30f, 0.0f, 0.0f, true);
	AddSpec(TEXT("/AirSim/Blueprints/ShippingSim/CargoVessel/Barge_Blueprint.Barge_Blueprint"), 700.0f, 0.20f, 0.0f, 0.0f, true);
	AddSpec(TEXT("/AirSim/Blueprints/ShippingSim/CargoVessel/BP_CargoPawn.BP_CargoPawn"), 900.0f, 0.10f, 0.0f, 0.0f, true);
	DisableDynamicShipObstacleSpeeds(Catalog);
	return Catalog;
}

void LoadNativeConfigFromActor(const AActor* Actor, FPCGNativeConfig& Cfg)
{
	if (const AGenerationManager* Defaults = GetDefault<AGenerationManager>()) {
		Cfg.bSpawnObstacles = Defaults->bSpawnObstacles;
		Cfg.DifficultyLevel = Defaults->DifficultyLevel;
		Cfg.ObstaclesPerKm = Defaults->ObstaclesPerKm;
		Cfg.ObstacleHardCap = Defaults->ObstacleHardCap;
		Cfg.VesselBeamCm = Defaults->VesselBeamCm;
		Cfg.ClearanceMultiple = Defaults->ClearanceMultiple;
		Cfg.GridCellSizeCm = Defaults->GridCellSizeCm;
		Cfg.MaxPlacementRetries = Defaults->MaxPlacementRetries;
		Cfg.ChannelWidthMinCm = Defaults->ChannelWidthMinCm;
		Cfg.ChannelWidthMaxCm = Defaults->ChannelWidthMaxCm;
		Cfg.ChannelWidthFreq = Defaults->ChannelWidthFreq;
		Cfg.SpineChaikinIter = Defaults->SpineChaikinIter;
		Cfg.ChicaneProbability = Defaults->ChicaneProbability;
		Cfg.ChicaneLateralRatio = Defaults->ChicaneLateralRatio;
		Cfg.PatternWeightSingle = Defaults->PatternWeightSingle;
		Cfg.PatternWeightGate = Defaults->PatternWeightGate;
		Cfg.PatternWeightSlalom = Defaults->PatternWeightSlalom;
		Cfg.PatternWeightCluster = Defaults->PatternWeightCluster;
		Cfg.ObstacleBoundaryPadding = Defaults->ObstacleBoundaryPadding;
		Cfg.ObstacleForwardJitter = Defaults->ObstacleForwardJitter;
		Cfg.ObstacleYawJitter = Defaults->ObstacleYawJitter;
		Cfg.ObstacleZOffset = Defaults->ObstacleZOffset;
		Cfg.ObstacleMinChannelWidth = Defaults->ObstacleMinChannelWidth;
		Cfg.ObstacleStartBuffer = Defaults->ObstacleStartBuffer;
		Cfg.ObstacleEndBuffer = Defaults->ObstacleEndBuffer;
		Cfg.ObstacleMinIndexStep = Defaults->ObstacleMinIndexStep;
		Cfg.ObstacleMaxIndexStep = Defaults->ObstacleMaxIndexStep;
	}

	if (!Actor) {
		return;
	}

	Cfg.bSpawnObstacles = GetBoolPropertyValue(Actor, TEXT("bSpawnObstacles"), Cfg.bSpawnObstacles);
	Cfg.DifficultyLevel = GetFloatPropertyValue(Actor, TEXT("DifficultyLevel"), Cfg.DifficultyLevel);
	Cfg.ObstaclesPerKm = GetFloatPropertyValue(Actor, TEXT("ObstaclesPerKm"), Cfg.ObstaclesPerKm);
	Cfg.ObstacleHardCap = GetIntPropertyValue(Actor, TEXT("ObstacleHardCap"), Cfg.ObstacleHardCap);
	Cfg.VesselBeamCm = GetFloatPropertyValue(Actor, TEXT("VesselBeamCm"), Cfg.VesselBeamCm);
	Cfg.ClearanceMultiple = GetFloatPropertyValue(Actor, TEXT("ClearanceMultiple"), Cfg.ClearanceMultiple);
	Cfg.GridCellSizeCm = GetFloatPropertyValue(Actor, TEXT("GridCellSizeCm"), Cfg.GridCellSizeCm);
	Cfg.MaxPlacementRetries = GetIntPropertyValue(Actor, TEXT("MaxPlacementRetries"), Cfg.MaxPlacementRetries);
	Cfg.ChannelWidthMinCm = GetFloatPropertyValue(Actor, TEXT("ChannelWidthMinCm"), Cfg.ChannelWidthMinCm);
	Cfg.ChannelWidthMaxCm = GetFloatPropertyValue(Actor, TEXT("ChannelWidthMaxCm"), Cfg.ChannelWidthMaxCm);
	Cfg.ChannelWidthFreq = GetFloatPropertyValue(Actor, TEXT("ChannelWidthFreq"), Cfg.ChannelWidthFreq);
	Cfg.SpineChaikinIter = GetIntPropertyValue(Actor, TEXT("SpineChaikinIter"), Cfg.SpineChaikinIter);
	Cfg.ChicaneProbability = GetFloatPropertyValue(Actor, TEXT("ChicaneProbability"), Cfg.ChicaneProbability);
	Cfg.ChicaneLateralRatio = GetFloatPropertyValue(Actor, TEXT("ChicaneLateralRatio"), Cfg.ChicaneLateralRatio);
	Cfg.PatternWeightSingle = GetFloatPropertyValue(Actor, TEXT("PatternWeightSingle"), Cfg.PatternWeightSingle);
	Cfg.PatternWeightGate = GetFloatPropertyValue(Actor, TEXT("PatternWeightGate"), Cfg.PatternWeightGate);
	Cfg.PatternWeightSlalom = GetFloatPropertyValue(Actor, TEXT("PatternWeightSlalom"), Cfg.PatternWeightSlalom);
	Cfg.PatternWeightCluster = GetFloatPropertyValue(Actor, TEXT("PatternWeightCluster"), Cfg.PatternWeightCluster);
	Cfg.ObstacleBoundaryPadding = GetFloatPropertyValue(Actor, TEXT("ObstacleBoundaryPadding"), Cfg.ObstacleBoundaryPadding);
	Cfg.ObstacleForwardJitter = GetFloatPropertyValue(Actor, TEXT("ObstacleForwardJitter"), Cfg.ObstacleForwardJitter);
	Cfg.ObstacleYawJitter = GetFloatPropertyValue(Actor, TEXT("ObstacleYawJitter"), Cfg.ObstacleYawJitter);
	Cfg.ObstacleZOffset = GetFloatPropertyValue(Actor, TEXT("ObstacleZOffset"), Cfg.ObstacleZOffset);
	Cfg.ObstacleMinChannelWidth = GetFloatPropertyValue(Actor, TEXT("ObstacleMinChannelWidth"), Cfg.ObstacleMinChannelWidth);
	Cfg.ObstacleStartBuffer = GetIntPropertyValue(Actor, TEXT("ObstacleStartBuffer"), Cfg.ObstacleStartBuffer);
	Cfg.ObstacleEndBuffer = GetIntPropertyValue(Actor, TEXT("ObstacleEndBuffer"), Cfg.ObstacleEndBuffer);
	Cfg.ObstacleMinIndexStep = GetIntPropertyValue(Actor, TEXT("ObstacleMinIndexStep"), Cfg.ObstacleMinIndexStep);
	Cfg.ObstacleMaxIndexStep = GetIntPropertyValue(Actor, TEXT("ObstacleMaxIndexStep"), Cfg.ObstacleMaxIndexStep);
}

TArray<FPCGObstacleSpec> LoadCatalogFromActor(const AActor* Actor)
{
	TArray<FPCGObstacleSpec> Catalog;
	if (const AGenerationManager* Defaults = GetDefault<AGenerationManager>()) {
		Catalog = Defaults->ObstacleCatalog;
	}
	if (Catalog.Num() == 0) {
		Catalog = LoadDefaultNativeCatalog();
	}

	if (TArray<FPCGObstacleSpec>* ActorCatalog = FindObstacleSpecArrayProperty(const_cast<AActor*>(Actor), TEXT("ObstacleCatalog"))) {
		if (ActorCatalog->Num() > 0) {
			Catalog = *ActorCatalog;
		}
	}

	DisableDynamicShipObstacleSpeeds(Catalog);
	return Catalog;
}

int32 RefineChannelGeometryNative(AActor* Actor, const FPCGNativeConfig& Cfg)
{
	if (!Actor) return 0;

	TArray<FTransform>* RoadPoints = FindTransformArrayProperty(Actor, TEXT("road"));
	TArray<FTransform>* LeftPoints = FindTransformArrayProperty(Actor, TEXT("left"));
	if (!LeftPoints) LeftPoints = FindTransformArrayProperty(Actor, TEXT("aleft"));
	TArray<FTransform>* RightPoints = FindTransformArrayProperty(Actor, TEXT("right"));
	if (!RightPoints) RightPoints = FindTransformArrayProperty(Actor, TEXT("aright"));
	if (!RoadPoints || !LeftPoints || !RightPoints) return 0;
	const int32 OrigN = FMath::Min(RoadPoints->Num(), FMath::Min(LeftPoints->Num(), RightPoints->Num()));
	if (OrigN < 2) return 0;

	// Diagnostic: sample first and mid road points before refine.
	if (OrigN >= 2) {
		const FVector R0 = (*RoadPoints)[0].GetLocation();
		const FVector Rm = (*RoadPoints)[OrigN / 2].GetLocation();
		const FVector Rn = (*RoadPoints)[OrigN - 1].GetLocation();
		UE_LOG(LogTemp, Warning, TEXT("Refine[IN]: road[0]=(%.0f,%.0f) road[%d]=(%.0f,%.0f) road[%d]=(%.0f,%.0f) (cm)"),
			R0.X, R0.Y, OrigN / 2, Rm.X, Rm.Y, OrigN - 1, Rn.X, Rn.Y);
	}

	FRandomStream* StreamProperty = FindRandomStreamProperty(Actor, TEXT("stream"));
	FRandomStream FallbackStream(10);
	FRandomStream& Stream = StreamProperty ? *StreamProperty : FallbackStream;

	TArray<FVector2D> Road2D, Left2D, Right2D;
	TArray<float> RoadZ;
	Road2D.Reserve(OrigN); Left2D.Reserve(OrigN); Right2D.Reserve(OrigN); RoadZ.Reserve(OrigN);
	for (int32 i = 0; i < OrigN; ++i) {
		const FVector R = (*RoadPoints)[i].GetLocation();
		const FVector L = (*LeftPoints)[i].GetLocation();
		const FVector Rt = (*RightPoints)[i].GetLocation();
		Road2D.Add(FVector2D(R.X, R.Y));
		Left2D.Add(FVector2D(L.X, L.Y));
		Right2D.Add(FVector2D(Rt.X, Rt.Y));
		RoadZ.Add(R.Z);
	}

	auto CumulativeLen = [](const TArray<FVector2D>& Ps) -> TArray<float> {
		TArray<float> C; C.SetNumZeroed(Ps.Num());
		for (int32 i = 1; i < Ps.Num(); ++i) C[i] = C[i - 1] + FVector2D::Distance(Ps[i - 1], Ps[i]);
		return C;
	};
	auto ResampleAtU = [](const TArray<FVector2D>& Ps, const TArray<float>& Cum, float U) -> FVector2D {
		const float Total = FMath::Max(Cum.Last(), KINDA_SMALL_NUMBER);
		const float Target = FMath::Clamp(U, 0.0f, 1.0f) * Total;
		int32 Seg = 0;
		while (Seg + 1 < Ps.Num() - 1 && Cum[Seg + 1] < Target) ++Seg;
		const int32 Next = FMath::Min(Seg + 1, Ps.Num() - 1);
		const float SegLen = FMath::Max(Cum[Next] - Cum[Seg], KINDA_SMALL_NUMBER);
		const float T = FMath::Clamp((Target - Cum[Seg]) / SegLen, 0.0f, 1.0f);
		return FMath::Lerp(Ps[Seg], Ps[Next], T);
	};

	TArray<FVector2D> SmoothRoad = Road2D;
	for (int32 Iter = 0; Iter < Cfg.SpineChaikinIter; ++Iter) {
		if (SmoothRoad.Num() < 3) break;
		TArray<FVector2D> Next; Next.Reserve(SmoothRoad.Num() * 2);
		Next.Add(SmoothRoad[0]);
		for (int32 i = 0; i + 1 < SmoothRoad.Num(); ++i) {
			Next.Add(0.75f * SmoothRoad[i] + 0.25f * SmoothRoad[i + 1]);
			Next.Add(0.25f * SmoothRoad[i] + 0.75f * SmoothRoad[i + 1]);
		}
		Next.Add(SmoothRoad.Last());
		SmoothRoad = MoveTemp(Next);
	}

	const TArray<float> RoadCum = CumulativeLen(SmoothRoad);
	const TArray<float> LeftCum = CumulativeLen(Left2D);
	const TArray<float> RightCum = CumulativeLen(Right2D);
	const float RoadTotal = RoadCum.Last();
	if (RoadTotal <= KINDA_SMALL_NUMBER) return 0;

	// Preserve original array length so getGoal(distance) semantics stay intact.
	const int32 NumSamples = OrigN;
	TArray<FVector2D> UR; UR.SetNumUninitialized(NumSamples);
	TArray<FVector2D> UL; UL.SetNumUninitialized(NumSamples);
	TArray<FVector2D> URt; URt.SetNumUninitialized(NumSamples);
	for (int32 i = 0; i < NumSamples; ++i) {
		const float U = static_cast<float>(i) / static_cast<float>(NumSamples - 1);
		UR[i] = ResampleAtU(SmoothRoad, RoadCum, U);
		UL[i] = ResampleAtU(Left2D, LeftCum, U);
		URt[i] = ResampleAtU(Right2D, RightCum, U);
	}

	auto ComputeRotators = [&](const TArray<FVector2D>& Ps, TArray<FRotator>& OutRot) {
		OutRot.SetNumUninitialized(Ps.Num());
		for (int32 i = 0; i < Ps.Num(); ++i) {
			FVector2D Dir;
			if (i == 0) Dir = Ps[1] - Ps[0];
			else if (i == Ps.Num() - 1) Dir = Ps.Last() - Ps[Ps.Num() - 2];
			else Dir = Ps[i + 1] - Ps[i - 1];
			if (Dir.IsNearlyZero()) Dir = FVector2D(1.0f, 0.0f);
			Dir.Normalize();
			OutRot[i] = FRotator(0.0f, FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X)), 0.0f);
		}
	};
	TArray<FRotator> Rotators;
	ComputeRotators(UR, Rotators);

	const float ChicaneRoll = Stream.FRand();
	const float ChicaneThresh = Cfg.ChicaneProbability * (0.5f + 0.5f * Cfg.DifficultyLevel);
	const bool bChicane = (ChicaneRoll < ChicaneThresh && NumSamples > 6);
	if (bChicane) {
		const float K = Stream.FRandRange(2.0f, 4.0f);
		const float ChicPhase = Stream.FRandRange(0.0f, 2.0f * PI);
		const int32 StartIdx = 2;
		const int32 EndIdx = NumSamples - 3;
		for (int32 i = StartIdx; i <= EndIdx; ++i) {
			const float U = static_cast<float>(i - StartIdx) / FMath::Max(1, EndIdx - StartIdx);
			const float Taper = FMath::Sin(PI * U);
			const FVector RightVec = Rotators[i].RotateVector(FVector::RightVector);
			const FVector2D RightXY(RightVec.X, RightVec.Y);
			const float DistLeft = FVector2D::Distance(UR[i], UL[i]);
			const float DistRight = FVector2D::Distance(UR[i], URt[i]);
			const float LocalBudget = FMath::Max(0.0f, FMath::Min(DistLeft, DistRight) - Cfg.VesselBeamCm * Cfg.ClearanceMultiple * 0.5f);
			const float Amp = FMath::Min(Cfg.ChicaneLateralRatio * LocalBudget, LocalBudget) * Taper;
			const float Offset = Amp * FMath::Sin(2.0f * PI * K * U + ChicPhase);
			UR[i] += RightXY * Offset;
		}
		ComputeRotators(UR, Rotators);
	}

	// Width tightening is intentionally disabled for the first pass: the
	// original BP walls define the visible corridor; narrowing them via
	// ChannelWidthMaxCm would also squeeze the navigable BFS grid and cause
	// the placement loop to reject most candidates in tight sections.
	// Keep left/right at their resampled positions so the corridor width
	// matches the actual BP walls 1:1.
	float MinObservedHalf = TNumericLimits<float>::Max();
	float MaxObservedHalf = 0.0f;
	for (int32 i = 0; i < NumSamples; ++i) {
		const float H = 0.5f * FVector2D::Distance(UL[i], URt[i]);
		MinObservedHalf = FMath::Min(MinObservedHalf, H);
		MaxObservedHalf = FMath::Max(MaxObservedHalf, H);
	}

	RoadPoints->Reset(); LeftPoints->Reset(); RightPoints->Reset();
	RoadPoints->Reserve(NumSamples); LeftPoints->Reserve(NumSamples); RightPoints->Reserve(NumSamples);
	const float ZStart = RoadZ.Num() > 0 ? RoadZ[0] : 0.0f;
	const float ZEnd = RoadZ.Num() > 0 ? RoadZ.Last() : 0.0f;
	for (int32 i = 0; i < NumSamples; ++i) {
		const float U = static_cast<float>(i) / static_cast<float>(NumSamples - 1);
		const float Z = FMath::Lerp(ZStart, ZEnd, U);
		RoadPoints->Add(FTransform(Rotators[i], FVector(UR[i].X, UR[i].Y, Z)));
		LeftPoints->Add(FTransform(Rotators[i], FVector(UL[i].X, UL[i].Y, Z)));
		RightPoints->Add(FTransform(Rotators[i], FVector(URt[i].X, URt[i].Y, Z)));
	}

	// Diagnostic: sample first/mid/last after refine.
	if (RoadPoints->Num() >= 2) {
		const int32 Mid = RoadPoints->Num() / 2;
		const FVector R0 = (*RoadPoints)[0].GetLocation();
		const FVector Rm = (*RoadPoints)[Mid].GetLocation();
		const FVector Rn = (*RoadPoints)[RoadPoints->Num() - 1].GetLocation();
		UE_LOG(LogTemp, Warning, TEXT("Refine[OUT]: road[0]=(%.0f,%.0f) road[%d]=(%.0f,%.0f) road[%d]=(%.0f,%.0f) (cm)"),
			R0.X, R0.Y, Mid, Rm.X, Rm.Y, RoadPoints->Num() - 1, Rn.X, Rn.Y);
	}

	UE_LOG(LogTemp, Warning, TEXT("RefineChannelGeometryNative: in=%d out=%d length=%.1fcm halfW=[%.0f,%.0f]cm chicane=%s."),
		OrigN, NumSamples, RoadTotal, MinObservedHalf, MaxObservedHalf, bChicane ? TEXT("true") : TEXT("false"));
	return NumSamples;
}

int32 SpawnAdvancedObstaclesNative(AActor* Actor, const FPCGNativeConfig& Cfg, const TArray<FPCGObstacleSpec>& Catalog)
{
	if (!Actor) return 0;
	if (!Cfg.bSpawnObstacles) return 0;
	UWorld* World = Actor->GetWorld();
	if (!World) return 0;

	TArray<FTransform>* RoadPoints = FindTransformArrayProperty(Actor, TEXT("road"));
	TArray<FTransform>* LeftPoints = FindTransformArrayProperty(Actor, TEXT("left"));
	if (!LeftPoints) LeftPoints = FindTransformArrayProperty(Actor, TEXT("aleft"));
	TArray<FTransform>* RightPoints = FindTransformArrayProperty(Actor, TEXT("right"));
	if (!RightPoints) RightPoints = FindTransformArrayProperty(Actor, TEXT("aright"));
	TArray<AActor*>* GeneratedActors = FindGeneratedActorArrayProperty(Actor);
	if (!RoadPoints || !LeftPoints || !RightPoints) return 0;
	const int32 N = FMath::Min(RoadPoints->Num(), FMath::Min(LeftPoints->Num(), RightPoints->Num()));
	if (N < 4) return 0;

	FRandomStream* StreamProperty = FindRandomStreamProperty(Actor, TEXT("stream"));
	FRandomStream FallbackStream(10);
	FRandomStream& Stream = StreamProperty ? *StreamProperty : FallbackStream;

	FVector2D AABBMin(TNumericLimits<float>::Max(), TNumericLimits<float>::Max());
	FVector2D AABBMax(-TNumericLimits<float>::Max(), -TNumericLimits<float>::Max());
	float TotalLenCm = 0.0f;
	for (int32 i = 0; i < N; ++i) {
		const FVector2D L2D((*LeftPoints)[i].GetLocation().X, (*LeftPoints)[i].GetLocation().Y);
		const FVector2D R2D((*RightPoints)[i].GetLocation().X, (*RightPoints)[i].GetLocation().Y);
		AABBMin = FVector2D(FMath::Min(AABBMin.X, FMath::Min(L2D.X, R2D.X)), FMath::Min(AABBMin.Y, FMath::Min(L2D.Y, R2D.Y)));
		AABBMax = FVector2D(FMath::Max(AABBMax.X, FMath::Max(L2D.X, R2D.X)), FMath::Max(AABBMax.Y, FMath::Max(L2D.Y, R2D.Y)));
		if (i + 1 < N) {
			TotalLenCm += FVector2D::Distance(
				FVector2D((*RoadPoints)[i].GetLocation().X, (*RoadPoints)[i].GetLocation().Y),
				FVector2D((*RoadPoints)[i + 1].GetLocation().X, (*RoadPoints)[i + 1].GetLocation().Y));
		}
	}
	const float Padding = FMath::Max(Cfg.GridCellSizeCm * 2.0f, 200.0f);
	AABBMin -= FVector2D(Padding, Padding);
	AABBMax += FVector2D(Padding, Padding);

	FNavGrid2D Grid;
	const int32 GW = FMath::CeilToInt((AABBMax.X - AABBMin.X) / Cfg.GridCellSizeCm) + 1;
	const int32 GH = FMath::CeilToInt((AABBMax.Y - AABBMin.Y) / Cfg.GridCellSizeCm) + 1;
	if (GW <= 0 || GH <= 0 || GW > 4096 || GH > 4096) {
		UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstaclesNative: invalid grid %dx%d."), GW, GH);
		return 0;
	}
	Grid.Initialize(AABBMin, GW, GH, Cfg.GridCellSizeCm);
	Grid.RasterizeChannel(*RoadPoints, *LeftPoints, *RightPoints);

	const FVector2D StartXY((*RoadPoints)[0].GetLocation().X, (*RoadPoints)[0].GetLocation().Y);
	const FVector2D GoalXY((*RoadPoints)[N - 1].GetLocation().X, (*RoadPoints)[N - 1].GetLocation().Y);
	const int32 ClearanceCells = FMath::Max(1, FMath::CeilToInt((Cfg.VesselBeamCm * Cfg.ClearanceMultiple) / (2.0f * Cfg.GridCellSizeCm)));
	if (!Grid.HasClearPath(StartXY, GoalXY, ClearanceCells)) {
		UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstaclesNative: base channel fails clearance BFS (cells=%d)."), ClearanceCells);
		return 0;
	}

	const float ChannelLenM = TotalLenCm / 100.0f;
	const float DifficultyFactor = 0.4f + 1.2f * Cfg.DifficultyLevel;
	int32 TargetCount = FMath::Clamp(
		FMath::RoundToInt(Cfg.ObstaclesPerKm * (ChannelLenM / 1000.0f) * DifficultyFactor),
		0, FMath::Max(0, Cfg.ObstacleHardCap));

	TArray<UClass*> CatalogClasses;
	CatalogClasses.SetNum(Catalog.Num());
	for (int32 i = 0; i < Catalog.Num(); ++i) {
		CatalogClasses[i] = Catalog[i].ObstacleClass.IsValid()
			? Catalog[i].ObstacleClass.Get()
			: Catalog[i].ObstacleClass.LoadSynchronous();
	}

	auto TryCommit = [&](TArray<FObstacleCandidate>& Cands) -> bool {
		// BFS already enforces a (2*ClearanceCells+1)-cell free window around each
		// traversable cell; extra padding on the blocked disk would double-count
		// the clearance and spuriously reject otherwise-passable placements.
		TArray<FNavGrid2D::FCellPatch> Patches;
		Patches.Reserve(Cands.Num() * 64);
		for (const FObstacleCandidate& C : Cands) {
			Grid.MarkDiskBlocked(FVector2D(C.Location.X, C.Location.Y), C.FootprintRadius, Patches);
		}
		if (Grid.HasClearPath(StartXY, GoalXY, ClearanceCells)) return true;
		Grid.RollbackPatches(Patches);
		return false;
	};

	auto SpawnOne = [&](const FObstacleCandidate& C) -> AActor* {
		UClass* SpawnClass = (Catalog.IsValidIndex(C.CatalogIndex)) ? CatalogClasses[C.CatalogIndex] : nullptr;
		if (!SpawnClass) return nullptr;
		const FTransform SpawnT(C.Rotation, C.Location);
		AActor* Spawned = World->SpawnActorDeferred<AActor>(
			SpawnClass, SpawnT, Actor, nullptr,
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn);
		if (!Spawned) return nullptr;
		const FPCGObstacleSpec& Spec = Catalog[C.CatalogIndex];
		if (Spec.bSupportsVelocityProperties) {
			const float Lo = FMath::Min(Spec.MinSpeed, Spec.MaxSpeed);
			const float Hi = FMath::Max(Spec.MinSpeed, Spec.MaxSpeed);
			const double Speed = (Hi > Lo) ? Stream.FRandRange(Lo, Hi) : Lo;
			SetBoolProperty(Spawned, TEXT("CustomVelocity"), true);
			SetBoolProperty(Spawned, TEXT("ApplyVelocityOnBeginPlay"), false);
			SetRealProperty(Spawned, TEXT("Speed"), Speed);
		}
		UGameplayStatics::FinishSpawningActor(Spawned, SpawnT);
		if (GeneratedActors) GeneratedActors->Add(Spawned);
		return Spawned;
	};

	const int32 StartIdxPath = FMath::Clamp(Cfg.ObstacleStartBuffer, 0, N - 1);
	const int32 EndIdxPath = FMath::Clamp(N - 1 - Cfg.ObstacleEndBuffer, 0, N - 1);

	int32 Spawned = 0;
	int32 ConsecFails = 0;
	int32 Cursor = StartIdxPath + Stream.RandRange(0, FMath::Max(0, Cfg.ObstacleMaxIndexStep - 1));

	while (Spawned < TargetCount && Cursor <= EndIdxPath && ConsecFails < Cfg.MaxPlacementRetries) {
		const EPCGObstaclePattern Pattern = PickPattern(Stream,
			Cfg.PatternWeightSingle, Cfg.PatternWeightGate, Cfg.PatternWeightSlalom, Cfg.PatternWeightCluster);
		const float HalfW = ChannelHalfWidthAt(*LeftPoints, *RightPoints, Cursor);
		if (HalfW * 2.0f < Cfg.ObstacleMinChannelWidth) {
			Cursor += Stream.RandRange(FMath::Max(1, Cfg.ObstacleMinIndexStep), FMath::Max(1, Cfg.ObstacleMaxIndexStep));
			continue;
		}

		TArray<FObstacleCandidate> Cands;
		const float MaxAllowedRadius = FMath::Max(50.0f, HalfW - Cfg.VesselBeamCm * Cfg.ClearanceMultiple * 0.5f - Cfg.ObstacleBoundaryPadding);

		auto MakeSingleCand = [&](int32 Idx, int32 CatIdx, float CurHalfW) -> FObstacleCandidate {
			FObstacleCandidate C;
			const FTransform& Road = (*RoadPoints)[Idx];
			const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
			const float Margin = FMath::Max(0.0f, CurHalfW - Cfg.ObstacleBoundaryPadding - Catalog[CatIdx].FootprintRadius);
			const float LateralOffset = Stream.FRandRange(-Margin, Margin);
			C.Location = Road.GetLocation() + RightVec * LateralOffset;
			C.Location.Z += Cfg.ObstacleZOffset;
			C.Rotation = Road.Rotator();
			C.Rotation.Yaw += Stream.FRandRange(-Cfg.ObstacleYawJitter, Cfg.ObstacleYawJitter);
			C.FootprintRadius = Catalog[CatIdx].FootprintRadius;
			C.CatalogIndex = CatIdx;
			return C;
		};

		switch (Pattern) {
			case EPCGObstaclePattern::Single: {
				const int32 CatIdx = PickCatalogIndex(Catalog, Stream, MaxAllowedRadius);
				if (CatIdx != INDEX_NONE) Cands.Add(MakeSingleCand(Cursor, CatIdx, HalfW));
				break;
			}
			case EPCGObstaclePattern::Gate: {
				const float Gap = Cfg.VesselBeamCm * Cfg.ClearanceMultiple * 1.2f;
				const float SideBudget = FMath::Max(50.0f, HalfW - Gap * 0.5f - Cfg.ObstacleBoundaryPadding);
				const int32 LeftCat = PickCatalogIndex(Catalog, Stream, SideBudget);
				const int32 RightCat = PickCatalogIndex(Catalog, Stream, SideBudget);
				if (LeftCat != INDEX_NONE && RightCat != INDEX_NONE) {
					const FTransform& Road = (*RoadPoints)[Cursor];
					const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
					FObstacleCandidate L; L.Location = Road.GetLocation() + RightVec * -(Gap * 0.5f + Catalog[LeftCat].FootprintRadius);
					L.Location.Z += Cfg.ObstacleZOffset; L.Rotation = Road.Rotator();
					L.FootprintRadius = Catalog[LeftCat].FootprintRadius; L.CatalogIndex = LeftCat;
					FObstacleCandidate R; R.Location = Road.GetLocation() + RightVec * (Gap * 0.5f + Catalog[RightCat].FootprintRadius);
					R.Location.Z += Cfg.ObstacleZOffset; R.Rotation = Road.Rotator();
					R.FootprintRadius = Catalog[RightCat].FootprintRadius; R.CatalogIndex = RightCat;
					Cands.Add(L); Cands.Add(R);
				}
				break;
			}
			case EPCGObstaclePattern::Slalom: {
				const int32 CountS = Stream.RandRange(3, 5);
				int32 Idx = Cursor;
				int32 Sign = (Stream.FRand() < 0.5f) ? -1 : 1;
				for (int32 s = 0; s < CountS && Idx <= EndIdxPath; ++s) {
					const float IdxHalfW = ChannelHalfWidthAt(*LeftPoints, *RightPoints, Idx);
					const float SideBudget = FMath::Max(50.0f, IdxHalfW - Cfg.VesselBeamCm * Cfg.ClearanceMultiple * 0.5f - Cfg.ObstacleBoundaryPadding);
					const int32 CatIdx = PickCatalogIndex(Catalog, Stream, SideBudget);
					if (CatIdx == INDEX_NONE) break;
					const FTransform& Road = (*RoadPoints)[Idx];
					const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
					const float Offset = Sign * (SideBudget * 0.6f + Catalog[CatIdx].FootprintRadius);
					FObstacleCandidate C; C.Location = Road.GetLocation() + RightVec * Offset;
					C.Location.Z += Cfg.ObstacleZOffset; C.Rotation = Road.Rotator();
					C.FootprintRadius = Catalog[CatIdx].FootprintRadius; C.CatalogIndex = CatIdx;
					Cands.Add(C);
					Sign = -Sign;
					Idx += FMath::Max(1, Cfg.ObstacleMinIndexStep + Stream.RandRange(0, 1));
				}
				Cursor = FMath::Max(Cursor, Idx - 1);
				break;
			}
			case EPCGObstaclePattern::Cluster: {
				const int32 CountC = Stream.RandRange(2, 3);
				for (int32 s = 0; s < CountC; ++s) {
					const int32 CatIdx = PickCatalogIndex(Catalog, Stream, FMath::Min(MaxAllowedRadius, 200.0f));
					if (CatIdx == INDEX_NONE) break;
					const FTransform& Road = (*RoadPoints)[Cursor];
					const FVector RightVec = Road.Rotator().RotateVector(FVector::RightVector);
					const FVector FwdVec = Road.Rotator().Vector();
					const float Lat = Stream.FRandRange(-HalfW * 0.6f, HalfW * 0.6f);
					const float Fwd = Stream.FRandRange(-Cfg.ObstacleForwardJitter * 2.0f, Cfg.ObstacleForwardJitter * 2.0f);
					FObstacleCandidate C; C.Location = Road.GetLocation() + RightVec * Lat + FwdVec * Fwd;
					C.Location.Z += Cfg.ObstacleZOffset; C.Rotation = Road.Rotator();
					C.Rotation.Yaw += Stream.FRandRange(-Cfg.ObstacleYawJitter, Cfg.ObstacleYawJitter);
					C.FootprintRadius = Catalog[CatIdx].FootprintRadius; C.CatalogIndex = CatIdx;
					Cands.Add(C);
				}
				break;
			}
		}

		if (Cands.Num() == 0) {
			Cursor += Stream.RandRange(FMath::Max(1, Cfg.ObstacleMinIndexStep), FMath::Max(1, Cfg.ObstacleMaxIndexStep));
			++ConsecFails;
			continue;
		}

		const int32 RemainingBudget = FMath::Max(0, TargetCount - Spawned);
		if (RemainingBudget <= 0) break;
		if (Cands.Num() > RemainingBudget) Cands.SetNum(RemainingBudget, EAllowShrinking::No);

		bool bCommitted = TryCommit(Cands);
		if (!bCommitted) {
			int32 SmallestIdx = INDEX_NONE;
			float SmallestR = TNumericLimits<float>::Max();
			for (int32 i = 0; i < Catalog.Num(); ++i) {
				if (Catalog[i].FootprintRadius < SmallestR) { SmallestR = Catalog[i].FootprintRadius; SmallestIdx = i; }
			}
			if (SmallestIdx != INDEX_NONE) {
				for (FObstacleCandidate& C : Cands) {
					C.CatalogIndex = SmallestIdx;
					C.FootprintRadius = Catalog[SmallestIdx].FootprintRadius;
				}
				bCommitted = TryCommit(Cands);
			}
		}

		if (bCommitted) {
			for (const FObstacleCandidate& C : Cands) {
				if (SpawnOne(C)) {
					UE_LOG(LogTemp, Warning, TEXT("Spawn: cat=%d loc=(%.0f,%.0f) cm"),
						C.CatalogIndex, C.Location.X, C.Location.Y);
					++Spawned;
				}
			}
			ConsecFails = 0;
		}
		else {
			++ConsecFails;
		}
		Cursor += Stream.RandRange(FMath::Max(1, Cfg.ObstacleMinIndexStep), FMath::Max(2, Cfg.ObstacleMaxIndexStep));
	}

	UE_LOG(LogTemp, Warning, TEXT("SpawnAdvancedObstaclesNative: spawned=%d target=%d channelLen=%.1fm clearanceCells=%d."),
		Spawned, TargetCount, ChannelLenM, ClearanceCells);
	return Spawned;
}
} // namespace

int32 AGenerationManager::ApplyAdvancedPCGNative(AActor* GenerationManagerActor)
{
	if (!GenerationManagerActor) return INDEX_NONE;
	FPCGNativeConfig Cfg;
	LoadNativeConfigFromActor(GenerationManagerActor, Cfg);
	const TArray<FPCGObstacleSpec> Catalog = LoadCatalogFromActor(GenerationManagerActor);
	RefineChannelGeometryNative(GenerationManagerActor, Cfg);
	if (!Cfg.bSpawnObstacles) {
		UE_LOG(LogTemp, Warning, TEXT("ApplyAdvancedPCGNative: obstacle spawning disabled by config."));
		return 0;
	}
	return SpawnAdvancedObstaclesNative(GenerationManagerActor, Cfg, Catalog);
}

// Called every frame
void AGenerationManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

