// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/Transform.h"
#include "GenerationManager.generated.h"

UCLASS()
class AIRSIM_API AGenerationManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGenerationManager();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> road;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> aleft;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> aright;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> temp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	int aseed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	FRandomStream stream;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<AActor*> generated;

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

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	void SpawnObstaclesImmediate();

};
