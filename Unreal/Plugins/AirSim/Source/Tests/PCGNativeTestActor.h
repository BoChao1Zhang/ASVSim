#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/RandomStream.h"
#include "PCG/GenerationManager.h"
#include "PCGNativeTestActor.generated.h"

UCLASS()
class AIRSIM_API APCGNativeTestActor : public AActor
{
    GENERATED_BODY()

public:
    APCGNativeTestActor();

    UPROPERTY(EditAnywhere)
    TArray<FTransform> road;

    UPROPERTY(EditAnywhere)
    TArray<FTransform> left;

    UPROPERTY(EditAnywhere)
    TArray<FTransform> right;

    UPROPERTY(EditAnywhere)
    TArray<AActor*> Generated;

    UPROPERTY(EditAnywhere)
    FRandomStream stream;

    UPROPERTY(EditAnywhere)
    bool bSpawnObstacles = true;

    UPROPERTY(EditAnywhere)
    float DifficultyLevel = 0.5f;

    UPROPERTY(EditAnywhere)
    float ObstaclesPerKm = 12.0f;

    UPROPERTY(EditAnywhere)
    int32 ObstacleHardCap = 40;

    UPROPERTY(EditAnywhere)
    float VesselBeamCm = 300.0f;

    UPROPERTY(EditAnywhere)
    float ClearanceMultiple = 2.0f;

    UPROPERTY(EditAnywhere)
    float GridCellSizeCm = 100.0f;

    UPROPERTY(EditAnywhere)
    int32 MaxPlacementRetries = 20;

    UPROPERTY(EditAnywhere)
    float ChannelWidthMinCm = 1400.0f;

    UPROPERTY(EditAnywhere)
    float ChannelWidthMaxCm = 3200.0f;

    UPROPERTY(EditAnywhere)
    float ChannelWidthFreq = 0.8f;

    UPROPERTY(EditAnywhere)
    int32 SpineChaikinIter = 2;

    UPROPERTY(EditAnywhere)
    float ChicaneProbability = 0.25f;

    UPROPERTY(EditAnywhere)
    float ChicaneLateralRatio = 0.4f;

    UPROPERTY(EditAnywhere)
    float PatternWeightSingle = 0.35f;

    UPROPERTY(EditAnywhere)
    float PatternWeightGate = 0.30f;

    UPROPERTY(EditAnywhere)
    float PatternWeightSlalom = 0.25f;

    UPROPERTY(EditAnywhere)
    float PatternWeightCluster = 0.10f;

    UPROPERTY(EditAnywhere)
    float ObstacleBoundaryPadding = 400.0f;

    UPROPERTY(EditAnywhere)
    float ObstacleForwardJitter = 250.0f;

    UPROPERTY(EditAnywhere)
    float ObstacleYawJitter = 20.0f;

    UPROPERTY(EditAnywhere)
    float ObstacleZOffset = 0.0f;

    UPROPERTY(EditAnywhere)
    float ObstacleMinChannelWidth = 800.0f;

    UPROPERTY(EditAnywhere)
    int32 ObstacleStartBuffer = 2;

    UPROPERTY(EditAnywhere)
    int32 ObstacleEndBuffer = 2;

    UPROPERTY(EditAnywhere)
    int32 ObstacleMinIndexStep = 1;

    UPROPERTY(EditAnywhere)
    int32 ObstacleMaxIndexStep = 2;

    UPROPERTY(EditAnywhere)
    TArray<FPCGObstacleSpec> ObstacleCatalog;
};
