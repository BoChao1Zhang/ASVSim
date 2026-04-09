#include "SimHUDWidget.h"
#include "Rendering/DrawElements.h"
#include "Styling/AppStyle.h"

void USimHUDWidget::updateDebugReport(const std::string& text)
{
    setReportText(FString(text.c_str()));
}

void USimHUDWidget::setReportVisible(bool is_visible)
{
    setReportContainerVisibility(is_visible);
}

void USimHUDWidget::toggleHelpVisibility()
{
    setHelpContainerVisibility(!getHelpContainerVisibility());
}

void USimHUDWidget::updateMiniMap(const TArray<FVector2D>& left_boundary,
                                  const TArray<FVector2D>& right_boundary,
                                  const FVector2D& vessel_position,
                                  const FVector2D& goal_position,
                                  bool is_valid)
{
    minimap_left_boundary_ = left_boundary;
    minimap_right_boundary_ = right_boundary;
    minimap_vessel_position_ = vessel_position;
    minimap_goal_position_ = goal_position;
    minimap_is_valid_ = is_valid;
    Invalidate(EInvalidateWidgetReason::Paint);
}

void USimHUDWidget::setOnToggleRecordingHandler(OnToggleRecording handler)
{
    on_toggle_recording_ = handler;
}

void USimHUDWidget::onToggleRecordingButtonClick()
{
    on_toggle_recording_();
}

int32 USimHUDWidget::NativePaint(const FPaintArgs& Args,
                                 const FGeometry& AllottedGeometry,
                                 const FSlateRect& MyCullingRect,
                                 FSlateWindowElementList& OutDrawElements,
                                 int32 LayerId,
                                 const FWidgetStyle& InWidgetStyle,
                                 bool bParentEnabled) const
{
    LayerId = Super::NativePaint(Args, AllottedGeometry, MyCullingRect, OutDrawElements, LayerId, InWidgetStyle, bParentEnabled);

    const FVector2D MapSize(220.0f, 220.0f);
    const FVector2D Margin(24.0f, 24.0f);
    const FVector2D Origin = FVector2D(AllottedGeometry.GetLocalSize().X - MapSize.X - Margin.X, Margin.Y);
    const FPaintGeometry BackgroundGeometry = AllottedGeometry.ToPaintGeometry(MapSize, FSlateLayoutTransform(Origin));
    const FSlateBrush* WhiteBrush = FAppStyle::Get().GetBrush("WhiteBrush");

    FSlateDrawElement::MakeBox(
        OutDrawElements,
        LayerId,
        BackgroundGeometry,
        WhiteBrush,
        ESlateDrawEffect::None,
        FLinearColor(0.02f, 0.03f, 0.05f, 0.78f));

    FSlateDrawElement::MakeBox(
        OutDrawElements,
        LayerId + 1,
        BackgroundGeometry,
        WhiteBrush,
        ESlateDrawEffect::None,
        FLinearColor(0.85f, 0.9f, 1.0f, 0.08f));

    if (!minimap_is_valid_ || minimap_left_boundary_.Num() < 2 || minimap_right_boundary_.Num() < 2) {
        return LayerId + 1;
    }

    FVector2D Min(FLT_MAX, FLT_MAX);
    FVector2D Max(-FLT_MAX, -FLT_MAX);
    auto ExpandBounds = [&Min, &Max](const FVector2D& Point) {
        Min.X = FMath::Min(Min.X, Point.X);
        Min.Y = FMath::Min(Min.Y, Point.Y);
        Max.X = FMath::Max(Max.X, Point.X);
        Max.Y = FMath::Max(Max.Y, Point.Y);
    };

    for (const FVector2D& Point : minimap_left_boundary_) {
        ExpandBounds(Point);
    }
    for (const FVector2D& Point : minimap_right_boundary_) {
        ExpandBounds(Point);
    }
    ExpandBounds(minimap_vessel_position_);
    ExpandBounds(minimap_goal_position_);

    FVector2D Extent = Max - Min;
    const float MaxAxis = FMath::Max(Extent.X, Extent.Y);
    if (MaxAxis <= KINDA_SMALL_NUMBER) {
        return LayerId + 1;
    }

    const FVector2D Center = (Min + Max) * 0.5f;
    const FVector2D HalfExtent(MaxAxis * 0.55f, MaxAxis * 0.55f);
    Min = Center - HalfExtent;
    Max = Center + HalfExtent;
    Extent = Max - Min;

    auto ProjectPoint = [&](const FVector2D& Point) {
        const float X = (Point.X - Min.X) / Extent.X;
        const float Y = (Point.Y - Min.Y) / Extent.Y;
        return Origin + FVector2D(X * MapSize.X, (1.0f - Y) * MapSize.Y);
    };

    TArray<FVector2D> LeftLine;
    TArray<FVector2D> RightLine;
    LeftLine.Reserve(minimap_left_boundary_.Num());
    RightLine.Reserve(minimap_right_boundary_.Num());

    for (const FVector2D& Point : minimap_left_boundary_) {
        LeftLine.Add(ProjectPoint(Point));
    }
    for (const FVector2D& Point : minimap_right_boundary_) {
        RightLine.Add(ProjectPoint(Point));
    }

    FSlateDrawElement::MakeLines(
        OutDrawElements,
        LayerId + 2,
        AllottedGeometry.ToPaintGeometry(),
        LeftLine,
        ESlateDrawEffect::None,
        FLinearColor(0.22f, 0.85f, 1.0f, 0.95f),
        true,
        2.0f);

    FSlateDrawElement::MakeLines(
        OutDrawElements,
        LayerId + 2,
        AllottedGeometry.ToPaintGeometry(),
        RightLine,
        ESlateDrawEffect::None,
        FLinearColor(0.22f, 0.85f, 1.0f, 0.95f),
        true,
        2.0f);

    const FVector2D VesselPos = ProjectPoint(minimap_vessel_position_);
    const FVector2D GoalPos = ProjectPoint(minimap_goal_position_);

    const FVector2D VesselSize(8.0f, 8.0f);
    const FVector2D GoalSize(10.0f, 10.0f);

    FSlateDrawElement::MakeBox(
        OutDrawElements,
        LayerId + 3,
        AllottedGeometry.ToPaintGeometry(VesselSize, FSlateLayoutTransform(VesselPos - VesselSize * 0.5f)),
        WhiteBrush,
        ESlateDrawEffect::None,
        FLinearColor(0.1f, 1.0f, 0.3f, 1.0f));

    FSlateDrawElement::MakeBox(
        OutDrawElements,
        LayerId + 3,
        AllottedGeometry.ToPaintGeometry(GoalSize, FSlateLayoutTransform(GoalPos - GoalSize * 0.5f)),
        WhiteBrush,
        ESlateDrawEffect::None,
        FLinearColor(1.0f, 0.25f, 0.2f, 1.0f));

    return LayerId + 3;
}
