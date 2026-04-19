#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

#include "Editor.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "PCG/GenerationManager.h"
#include "SimMode/SimModeBase.h"
#include "Tests/PCGNativeTestActor.h"

namespace
{
struct FPCGScenarioConfig
{
    int32 NumPoints = 24;
    float StepCm = 9000.0f;
    float HalfWidthCm = 8000.0f;
    float DifficultyLevel = 0.5f;
    int32 ObstacleHardCap = 60;
    float ObstacleBoundaryPadding = 50.0f;
    int32 ObstacleStartBuffer = 1;
    int32 ObstacleEndBuffer = 1;
    int32 ObstacleMinIndexStep = 1;
    int32 ObstacleMaxIndexStep = 1;
    float ClearanceMultiple = 1.5f;
    float ChicaneProbability = 0.0f;
    bool bSpawnObstacles = true;
};

UWorld* GetAutomationWorld()
{
    if (GEditor)
    {
        if (GEditor->PlayWorld)
        {
            return GEditor->PlayWorld;
        }

        if (UWorld* EditorWorld = GEditor->GetEditorWorldContext().World())
        {
            return EditorWorld;
        }
    }

    if (!GEngine)
    {
        return nullptr;
    }

    for (const FWorldContext& Context : GEngine->GetWorldContexts())
    {
        if (UWorld* World = Context.World())
        {
            return World;
        }
    }

    return nullptr;
}

void BuildStraightChannel(APCGNativeTestActor* Actor, int32 NumPoints = 13, float StepCm = 8000.0f, float HalfWidthCm = 3500.0f)
{
    check(Actor);

    Actor->road.Reset();
    Actor->left.Reset();
    Actor->right.Reset();
    Actor->aleft.Reset();
    Actor->aright.Reset();
    Actor->Generated.Reset();
    Actor->stream.Initialize(12345);

    for (int32 Index = 0; Index < NumPoints; ++Index)
    {
        const float X = static_cast<float>(Index) * StepCm;
        const FVector RoadLocation(X, 0.0f, 0.0f);
        const FVector LeftLocation(X, -HalfWidthCm, 0.0f);
        const FVector RightLocation(X, HalfWidthCm, 0.0f);
        const FRotator Rotation(0.0f, 0.0f, 0.0f);

        Actor->road.Add(FTransform(Rotation, RoadLocation));
        Actor->left.Add(FTransform(Rotation, LeftLocation));
        Actor->right.Add(FTransform(Rotation, RightLocation));
    }
}

void BuildCurvedChannel(
    APCGNativeTestActor* Actor,
    int32 NumPoints,
    float PathLengthCm,
    float AmplitudeCm,
    float HalfWidthCm,
    float WaveCount = 1.0f)
{
    check(Actor);

    Actor->road.Reset();
    Actor->left.Reset();
    Actor->right.Reset();
    Actor->aleft.Reset();
    Actor->aright.Reset();
    Actor->Generated.Reset();
    Actor->stream.Initialize(12345);

    const float Frequency = 2.0f * PI * WaveCount;
    for (int32 Index = 0; Index < NumPoints; ++Index)
    {
        const float U = (NumPoints > 1) ? static_cast<float>(Index) / static_cast<float>(NumPoints - 1) : 0.0f;
        const float X = PathLengthCm * U;
        const float Y = AmplitudeCm * FMath::Sin(Frequency * U);

        FVector2D Tangent(PathLengthCm, AmplitudeCm * Frequency * FMath::Cos(Frequency * U));
        if (Tangent.IsNearlyZero())
        {
            Tangent = FVector2D(1.0f, 0.0f);
        }
        Tangent.Normalize();
        const FVector2D Normal(-Tangent.Y, Tangent.X);
        const float YawDeg = FMath::RadiansToDegrees(FMath::Atan2(Tangent.Y, Tangent.X));

        const FVector RoadLocation(X, Y, 0.0f);
        const FVector LeftLocation(X - Normal.X * HalfWidthCm, Y - Normal.Y * HalfWidthCm, 0.0f);
        const FVector RightLocation(X + Normal.X * HalfWidthCm, Y + Normal.Y * HalfWidthCm, 0.0f);
        const FRotator Rotation(0.0f, YawDeg, 0.0f);

        Actor->road.Add(FTransform(Rotation, RoadLocation));
        Actor->left.Add(FTransform(Rotation, LeftLocation));
        Actor->right.Add(FTransform(Rotation, RightLocation));
    }
}

APCGNativeTestActor* SpawnTestActor(UWorld* World)
{
    if (!World)
    {
        return nullptr;
    }

    FActorSpawnParameters SpawnParams;
    SpawnParams.ObjectFlags |= RF_Transient;
    return World->SpawnActor<APCGNativeTestActor>(FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
}

void CleanupSpawnedActors(APCGNativeTestActor* Actor)
{
    if (!Actor)
    {
        return;
    }

    for (AActor* Spawned : Actor->Generated)
    {
        if (IsValid(Spawned))
        {
            Spawned->Destroy();
        }
    }

    Actor->Generated.Reset();
}

int32 RunScenario(FAutomationTestBase& Test, UWorld* World, const FPCGScenarioConfig& Config, const TCHAR* ContextLabel)
{
    APCGNativeTestActor* Actor = SpawnTestActor(World);
    Test.TestNotNull(FString::Printf(TEXT("%s actor"), ContextLabel), Actor);
    if (!Actor)
    {
        return INDEX_NONE;
    }

    BuildStraightChannel(Actor, Config.NumPoints, Config.StepCm, Config.HalfWidthCm);
    Actor->DifficultyLevel = Config.DifficultyLevel;
    Actor->ObstacleHardCap = Config.ObstacleHardCap;
    Actor->ObstacleBoundaryPadding = Config.ObstacleBoundaryPadding;
    Actor->ObstacleStartBuffer = Config.ObstacleStartBuffer;
    Actor->ObstacleEndBuffer = Config.ObstacleEndBuffer;
    Actor->ObstacleMinIndexStep = Config.ObstacleMinIndexStep;
    Actor->ObstacleMaxIndexStep = Config.ObstacleMaxIndexStep;
    Actor->ClearanceMultiple = Config.ClearanceMultiple;
    Actor->ChicaneProbability = Config.ChicaneProbability;
    Actor->bSpawnObstacles = Config.bSpawnObstacles;

    const int32 Spawned = AGenerationManager::ApplyAdvancedPCGNative(Actor);

    Test.TestTrue(FString::Printf(TEXT("%s should execute successfully"), ContextLabel), Spawned >= 0);
    Test.TestTrue(FString::Printf(TEXT("%s should respect hard cap"), ContextLabel), Spawned <= Actor->ObstacleHardCap);

    CleanupSpawnedActors(Actor);
    Actor->Destroy();
    return Spawned;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeRespectsObstacleHardCapTest,
    "AirSim.PCG.Native.RespectsObstacleHardCap",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeRespectsObstacleHardCapTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    APCGNativeTestActor* Actor = SpawnTestActor(World);
    TestNotNull(TEXT("PCG test actor"), Actor);
    if (!Actor)
    {
        return false;
    }

    BuildStraightChannel(Actor);
    Actor->ObstacleHardCap = 0;

    const int32 Spawned = AGenerationManager::ApplyAdvancedPCGNative(Actor);

    TestEqual(TEXT("ObstacleHardCap=0 should suppress native spawns"), Spawned, 0);
    TestEqual(TEXT("Generated array should stay empty when hard cap is zero"), Actor->Generated.Num(), 0);

    CleanupSpawnedActors(Actor);
    Actor->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeRespectsSpawnToggleTest,
    "AirSim.PCG.Native.RespectsSpawnToggle",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeRespectsSpawnToggleTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    APCGNativeTestActor* Actor = SpawnTestActor(World);
    TestNotNull(TEXT("PCG test actor"), Actor);
    if (!Actor)
    {
        return false;
    }

    BuildStraightChannel(Actor);
    Actor->bSpawnObstacles = false;

    const int32 Spawned = AGenerationManager::ApplyAdvancedPCGNative(Actor);

    TestEqual(TEXT("bSpawnObstacles=false should suppress native spawns"), Spawned, 0);
    TestEqual(TEXT("Generated array should stay empty when obstacle spawning is disabled"), Actor->Generated.Num(), 0);

    CleanupSpawnedActors(Actor);
    Actor->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeCleanupGeneratedActorsTest,
    "AirSim.PCG.Native.CleanupGeneratedActors",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeCleanupGeneratedActorsTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    APCGNativeTestActor* Actor = SpawnTestActor(World);
    TestNotNull(TEXT("PCG test actor"), Actor);
    if (!Actor)
    {
        return false;
    }

    BuildStraightChannel(Actor, 5);

    FActorSpawnParameters SpawnParams;
    SpawnParams.ObjectFlags |= RF_Transient;
    AActor* SpawnedA = World->SpawnActor<AActor>(FVector(100.0f, 0.0f, 0.0f), FRotator::ZeroRotator, SpawnParams);
    AActor* SpawnedB = World->SpawnActor<AActor>(FVector(200.0f, 0.0f, 0.0f), FRotator::ZeroRotator, SpawnParams);
    TestNotNull(TEXT("First generated actor"), SpawnedA);
    TestNotNull(TEXT("Second generated actor"), SpawnedB);
    if (!SpawnedA || !SpawnedB)
    {
        if (Actor)
        {
            Actor->Destroy();
        }
        return false;
    }

    Actor->Generated.Add(SpawnedA);
    Actor->Generated.Add(SpawnedB);
    Actor->Generated.Add(nullptr);

    SpawnedA->SetActorEnableCollision(true);
    SpawnedA->SetActorHiddenInGame(false);
    SpawnedB->SetActorEnableCollision(true);
    SpawnedB->SetActorHiddenInGame(false);
    SpawnedB->Destroy();

    TestEqual(TEXT("Generated actor count before cleanup should ignore null and pending-destroy entries"), AGenerationManager::GetGeneratedActorCountNative(Actor), 1);
    TestTrue(TEXT("Road points should exist before cleanup"), Actor->road.Num() > 0);
    TestTrue(TEXT("Left points should exist before cleanup"), Actor->left.Num() > 0);
    TestTrue(TEXT("Right points should exist before cleanup"), Actor->right.Num() > 0);

    const int32 Destroyed = AGenerationManager::CleanupGeneratedActorsNative(Actor);

    TestEqual(TEXT("Cleanup should destroy only the remaining live generated actor"), Destroyed, 1);
    TestEqual(TEXT("Generated actor count after cleanup"), AGenerationManager::GetGeneratedActorCountNative(Actor), 0);
    TestEqual(TEXT("Generated array should be empty after cleanup"), Actor->Generated.Num(), 0);
    TestEqual(TEXT("Road points should be reset by cleanup"), Actor->road.Num(), 0);
    TestEqual(TEXT("Left points should be reset by cleanup"), Actor->left.Num(), 0);
    TestEqual(TEXT("Right points should be reset by cleanup"), Actor->right.Num(), 0);
    TestTrue(TEXT("First generated actor should be gone after cleanup"), !IsValid(SpawnedA) || SpawnedA->IsActorBeingDestroyed());
    TestTrue(TEXT("Second generated actor should be gone after cleanup"), !IsValid(SpawnedB) || SpawnedB->IsActorBeingDestroyed());
    TestTrue(TEXT("Cleanup should disable collision immediately for live generated actors"), !IsValid(SpawnedA) || !SpawnedA->GetActorEnableCollision());
    TestTrue(TEXT("Cleanup should hide live generated actors immediately"), !IsValid(SpawnedA) || SpawnedA->IsHidden());

    Actor->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeAliasBoundaryPathValidityTest,
    "AirSim.PCG.Native.AliasBoundaryPathValidity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeAliasBoundaryPathValidityTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    APCGNativeTestActor* Actor = SpawnTestActor(World);
    TestNotNull(TEXT("PCG test actor"), Actor);
    if (!Actor)
    {
        return false;
    }

    Actor->road.Reset();
    Actor->left.Reset();
    Actor->right.Reset();
    Actor->aleft.Reset();
    Actor->aright.Reset();

    for (int32 Index = 0; Index < 4; ++Index)
    {
        const float X = static_cast<float>(Index) * 8000.0f;
        const FRotator Rotation(0.0f, 0.0f, 0.0f);
        Actor->road.Add(FTransform(Rotation, FVector(X, 0.0f, 0.0f)));
        Actor->aleft.Add(FTransform(Rotation, FVector(X, -3500.0f, 0.0f)));
        Actor->aright.Add(FTransform(Rotation, FVector(X, 3500.0f, 0.0f)));
    }

    int32 RoadCount = 0;
    int32 BoundaryCount = 0;
    const bool bHasValidPathData = ASimModeBase::HasValidGenerationPathDataNative(Actor, RoadCount, BoundaryCount);

    TestTrue(TEXT("Path validity should accept aleft/aright when left/right are absent"), bHasValidPathData);
    TestEqual(TEXT("Road count should come from road path"), RoadCount, 4);
    TestEqual(TEXT("Boundary count should come from alias boundary paths"), BoundaryCount, 4);

    Actor->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeComplexityScalesGenerationTest,
    "AirSim.PCG.Native.ComplexityScalesGeneration",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeComplexityScalesGenerationTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    FPCGScenarioConfig LowConfig;
    LowConfig.DifficultyLevel = 0.0f;
    const int32 LowSpawned = RunScenario(*this, World, LowConfig, TEXT("Low complexity"));

    FPCGScenarioConfig MediumConfig;
    MediumConfig.DifficultyLevel = 0.5f;
    const int32 MediumSpawned = RunScenario(*this, World, MediumConfig, TEXT("Medium complexity"));

    FPCGScenarioConfig HighConfig;
    HighConfig.DifficultyLevel = 1.0f;
    const int32 HighSpawned = RunScenario(*this, World, HighConfig, TEXT("High complexity"));

    TestTrue(TEXT("Low complexity should generate at least one obstacle"), LowSpawned > 0);
    TestTrue(TEXT("Medium complexity should generate at least as many obstacles as low complexity"), MediumSpawned >= LowSpawned);
    TestTrue(TEXT("High complexity should generate at least as many obstacles as medium complexity"), HighSpawned >= MediumSpawned);

    return LowSpawned >= 0 && MediumSpawned >= 0 && HighSpawned >= 0;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeNarrowChannelComplexityCurveTest,
    "AirSim.PCG.Native.NarrowChannelComplexityCurve",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeNarrowChannelComplexityCurveTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    FPCGScenarioConfig WideConfig;
    WideConfig.DifficultyLevel = 1.0f;
    const int32 WideHighSpawned = RunScenario(*this, World, WideConfig, TEXT("Wide high complexity"));

    FPCGScenarioConfig NarrowLowConfig;
    NarrowLowConfig.HalfWidthCm = 1600.0f;
    NarrowLowConfig.ObstacleBoundaryPadding = 200.0f;
    NarrowLowConfig.ClearanceMultiple = 2.5f;
    NarrowLowConfig.DifficultyLevel = 0.0f;
    const int32 NarrowLowSpawned = RunScenario(*this, World, NarrowLowConfig, TEXT("Narrow low complexity"));

    FPCGScenarioConfig NarrowMediumConfig = NarrowLowConfig;
    NarrowMediumConfig.DifficultyLevel = 0.5f;
    const int32 NarrowMediumSpawned = RunScenario(*this, World, NarrowMediumConfig, TEXT("Narrow medium complexity"));

    FPCGScenarioConfig NarrowHighConfig = NarrowLowConfig;
    NarrowHighConfig.DifficultyLevel = 1.0f;
    const int32 NarrowHighSpawned = RunScenario(*this, World, NarrowHighConfig, TEXT("Narrow high complexity"));

    TestTrue(TEXT("Narrow low complexity should still be able to place at least one obstacle"), NarrowLowSpawned > 0);
    TestTrue(TEXT("Narrow medium complexity should generate at least as many obstacles as narrow low complexity"), NarrowMediumSpawned >= NarrowLowSpawned);
    TestTrue(TEXT("Narrow high complexity should generate at least as many obstacles as narrow medium complexity"), NarrowHighSpawned >= NarrowMediumSpawned);
    TestTrue(TEXT("Narrow high complexity should not exceed the wide-channel high-complexity spawn count"), NarrowHighSpawned <= WideHighSpawned);

    return WideHighSpawned >= 0 && NarrowLowSpawned >= 0 && NarrowMediumSpawned >= 0 && NarrowHighSpawned >= 0;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeShortPathComplexityCurveTest,
    "AirSim.PCG.Native.ShortPathComplexityCurve",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeShortPathComplexityCurveTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    FPCGScenarioConfig LongConfig;
    LongConfig.DifficultyLevel = 1.0f;
    const int32 LongHighSpawned = RunScenario(*this, World, LongConfig, TEXT("Long high complexity"));

    FPCGScenarioConfig ShortLowConfig;
    ShortLowConfig.NumPoints = 6;
    ShortLowConfig.StepCm = 5000.0f;
    ShortLowConfig.DifficultyLevel = 0.0f;
    const int32 ShortLowSpawned = RunScenario(*this, World, ShortLowConfig, TEXT("Short low complexity"));

    FPCGScenarioConfig ShortMediumConfig = ShortLowConfig;
    ShortMediumConfig.DifficultyLevel = 0.5f;
    const int32 ShortMediumSpawned = RunScenario(*this, World, ShortMediumConfig, TEXT("Short medium complexity"));

    FPCGScenarioConfig ShortHighConfig = ShortLowConfig;
    ShortHighConfig.DifficultyLevel = 1.0f;
    const int32 ShortHighSpawned = RunScenario(*this, World, ShortHighConfig, TEXT("Short high complexity"));

    TestTrue(TEXT("Short low complexity should still be able to place at least one obstacle"), ShortLowSpawned > 0);
    TestTrue(TEXT("Short medium complexity should generate at least as many obstacles as short low complexity"), ShortMediumSpawned >= ShortLowSpawned);
    TestTrue(TEXT("Short high complexity should generate at least as many obstacles as short medium complexity"), ShortHighSpawned >= ShortMediumSpawned);
    TestTrue(TEXT("Short high complexity should not exceed the long-path high-complexity spawn count"), ShortHighSpawned <= LongHighSpawned);

    return LongHighSpawned >= 0 && ShortLowSpawned >= 0 && ShortMediumSpawned >= 0 && ShortHighSpawned >= 0;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPCGNativeCurvedNarrowChannelBoundaryTest,
    "AirSim.PCG.Native.CurvedNarrowChannelBoundary",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPCGNativeCurvedNarrowChannelBoundaryTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetAutomationWorld();
    TestNotNull(TEXT("Automation world"), World);
    if (!World)
    {
        return false;
    }

    auto RunCurvedScenario = [this, World](const FPCGScenarioConfig& Config, const TCHAR* ContextLabel, float HalfWidthCm) -> int32
    {
        APCGNativeTestActor* Actor = SpawnTestActor(World);
        TestNotNull(FString::Printf(TEXT("%s actor"), ContextLabel), Actor);
        if (!Actor)
        {
            return INDEX_NONE;
        }

        BuildCurvedChannel(Actor, Config.NumPoints, Config.StepCm * static_cast<float>(Config.NumPoints - 1), 8000.0f, HalfWidthCm, 1.5f);
        Actor->DifficultyLevel = Config.DifficultyLevel;
        Actor->ObstacleHardCap = Config.ObstacleHardCap;
        Actor->ObstacleBoundaryPadding = Config.ObstacleBoundaryPadding;
        Actor->ObstacleStartBuffer = Config.ObstacleStartBuffer;
        Actor->ObstacleEndBuffer = Config.ObstacleEndBuffer;
        Actor->ObstacleMinIndexStep = Config.ObstacleMinIndexStep;
        Actor->ObstacleMaxIndexStep = Config.ObstacleMaxIndexStep;
        Actor->ClearanceMultiple = Config.ClearanceMultiple;
        Actor->ChicaneProbability = Config.ChicaneProbability;
        Actor->bSpawnObstacles = Config.bSpawnObstacles;

        const int32 Spawned = AGenerationManager::ApplyAdvancedPCGNative(Actor);
        TestTrue(FString::Printf(TEXT("%s should execute successfully"), ContextLabel), Spawned >= 0);
        CleanupSpawnedActors(Actor);
        Actor->Destroy();
        return Spawned;
    };

    FPCGScenarioConfig WideHighConfig;
    WideHighConfig.NumPoints = 16;
    WideHighConfig.StepCm = 5000.0f;
    WideHighConfig.DifficultyLevel = 1.0f;
    WideHighConfig.ObstacleHardCap = 40;
    WideHighConfig.ObstacleBoundaryPadding = 100.0f;
    WideHighConfig.ClearanceMultiple = 1.5f;
    const int32 CurvedWideHigh = RunCurvedScenario(WideHighConfig, TEXT("Curved wide high complexity"), 4000.0f);

    FPCGScenarioConfig NarrowLowConfig = WideHighConfig;
    NarrowLowConfig.DifficultyLevel = 0.0f;
    NarrowLowConfig.ObstacleBoundaryPadding = 250.0f;
    NarrowLowConfig.ClearanceMultiple = 3.0f;
    const int32 CurvedNarrowLow = RunCurvedScenario(NarrowLowConfig, TEXT("Curved narrow low complexity"), 900.0f);

    FPCGScenarioConfig NarrowMediumConfig = NarrowLowConfig;
    NarrowMediumConfig.DifficultyLevel = 0.5f;
    const int32 CurvedNarrowMedium = RunCurvedScenario(NarrowMediumConfig, TEXT("Curved narrow medium complexity"), 900.0f);

    FPCGScenarioConfig NarrowHighConfig = NarrowLowConfig;
    NarrowHighConfig.DifficultyLevel = 1.0f;
    const int32 CurvedNarrowHigh = RunCurvedScenario(NarrowHighConfig, TEXT("Curved narrow high complexity"), 900.0f);

    TestTrue(TEXT("Curved narrow low complexity should still place at least one obstacle"), CurvedNarrowLow > 0);
    TestTrue(TEXT("Curved narrow medium complexity should generate at least as many obstacles as low complexity"), CurvedNarrowMedium >= CurvedNarrowLow);
    TestTrue(TEXT("Curved narrow high complexity should generate at least as many obstacles as medium complexity"), CurvedNarrowHigh >= CurvedNarrowMedium);
    TestTrue(TEXT("Curved narrow high complexity should not exceed curved wide high complexity"), CurvedNarrowHigh <= CurvedWideHigh);

    return CurvedWideHigh >= 0 && CurvedNarrowLow >= 0 && CurvedNarrowMedium >= 0 && CurvedNarrowHigh >= 0;
}

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
