// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/Transform.h"
#include "Math/RandomStream.h"
#include "GenerationManager.generated.h"

USTRUCT(BlueprintType)
struct FPCGObstacleSpec
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCG Obstacle")
	TSoftClassPtr<AActor> ObstacleClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCG Obstacle", meta = (ClampMin = "0.0"))
	float FootprintRadius = 150.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCG Obstacle", meta = (ClampMin = "0.0"))
	float Weight = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCG Obstacle")
	float MinSpeed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCG Obstacle")
	float MaxSpeed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCG Obstacle")
	bool bSupportsVelocityProperties = true;
};

UENUM(BlueprintType)
enum class EPCGObstaclePattern : uint8
{
	Single  UMETA(DisplayName = "Single"),
	Gate    UMETA(DisplayName = "Gate"),
	Slalom  UMETA(DisplayName = "Slalom"),
	Cluster UMETA(DisplayName = "Cluster"),
};

UCLASS()
class AIRSIM_API AGenerationManager : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AGenerationManager();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles")
	bool bSpawnObstacles = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles")
	TSubclassOf<AActor> ObstacleClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0"))
	int32 ObstacleStartBuffer = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0"))
	int32 ObstacleEndBuffer = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "1"))
	int32 ObstacleMinIndexStep = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "1"))
	int32 ObstacleMaxIndexStep = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0"))
	int32 ObstacleMaxCount = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0.0"))
	float ObstacleBoundaryPadding = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ObstacleLateralRatio = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0.0"))
	float ObstacleForwardJitter = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0.0"))
	float ObstacleYawJitter = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles")
	float ObstacleZOffset = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles", meta = (ClampMin = "0.0"))
	float ObstacleMinChannelWidth = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles")
	float ObstacleSpeedMin = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles")
	float ObstacleSpeedMax = 0.0f;

	// ---- Advanced PCG: difficulty / density ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float DifficultyLevel = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0.0"))
	float ObstaclesPerKm = 12.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0"))
	int32 ObstacleHardCap = 40;

	// ---- Advanced PCG: navigability ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "50.0"))
	float VesselBeamCm = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "1.0"))
	float ClearanceMultiple = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "25.0"))
	float GridCellSizeCm = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "1"))
	int32 MaxPlacementRetries = 20;

	// ---- Advanced PCG: channel width profile ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "200.0"))
	float ChannelWidthMinCm = 1400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "200.0"))
	float ChannelWidthMaxCm = 3200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "0.0"))
	float ChannelWidthFreq = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "0"))
	int32 SpineChaikinIter = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "100.0"))
	float SpineResampleCm = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ChicaneProbability = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Channel|Advanced", meta = (ClampMin = "0.0", ClampMax = "0.9"))
	float ChicaneLateralRatio = 0.4f;

	// ---- Advanced PCG: placement pattern weights ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0.0"))
	float PatternWeightSingle = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0.0"))
	float PatternWeightGate = 0.30f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0.0"))
	float PatternWeightSlalom = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced", meta = (ClampMin = "0.0"))
	float PatternWeightCluster = 0.10f;

	// ---- Advanced PCG: obstacle catalog (reused map assets) ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacles|Advanced")
	TArray<FPCGObstacleSpec> ObstacleCatalog;

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	bool HavenStep(UPARAM(ref) TArray<FTransform>& haven, float mina, float maxa, float mind, float maxd);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void WallStep(UPARAM(ref) TArray<FTransform>& haven, UPARAM(ref) TArray<FTransform>& wall, float min, float max);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void GenerateWalls(UPARAM(ref) TArray<FTransform>& points, bool inlets, float min, float max);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void InletFunction(FTransform begin, FTransform end, FRotator rotation, float min, float max, float length);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void InletGeneration(UPARAM(ref) TArray<FTransform>& points);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void Clear();

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void SetSeed(int32 seed);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void generateTerrain(FString type, float mina, float maxa, float mind, float maxd, int length);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void getGoal(int32 distance,FVector& location, FVector& left, FVector& right);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void SpawnObstacles();

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	int32 RefineChannelGeometry();

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	int32 SpawnAdvancedObstacles();

	// Reflection-based runtime cleanup: works on ANY actor that exposes the
	// Generated/generated actor array used for native obstacle ownership.
	// Returns the number of valid spawned actors destroyed.
	static int32 CleanupGeneratedActorsNative(AActor* GenerationManagerActor, bool bResetPathData = true);

	// Reflection-based accessor for the runtime-owned obstacle container.
	static int32 GetGeneratedActorCountNative(const AActor* GenerationManagerActor);

	// Reflection-based entry point: works on ANY actor that exposes the
	// road/left/right/Generated/stream properties (including BP-only actors
	// that do not derive from AGenerationManager).
	// Returns the number of obstacles actually spawned (>=0).
	static int32 ApplyAdvancedPCGNative(AActor* GenerationManagerActor);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	int32 CachedSeed = 10;

	void SpawnObstaclesImmediate();

	void InitializeDefaultObstacleCatalog();
};
