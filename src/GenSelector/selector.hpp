#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "liblvgl/lvgl.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace robot {

using AutonFunc = void (*)();
using AutonRoutineList = std::vector<std::pair<std::string, AutonFunc>>;
using TerminalValueGetter = double (*)();

enum class SelectorInputType {
    BrainScreen,
    ControllerButton,
    AdiDigitalIn,
    Custom
};

struct SelectorInputConfig {
    SelectorInputType type = SelectorInputType::BrainScreen;
    pros::Controller* controller = nullptr;
    pros::controller_digital_e_t controllerButton = pros::E_CONTROLLER_DIGITAL_X;
    pros::adi::DigitalIn* adiDigitalIn = nullptr;
    void* customDevice = nullptr;
    bool (*customNewPress)(void* device) = nullptr;
};

struct SelectorMenuConfig {
    std::string teamNumber = "0000A";
};

struct SelectorTemperatureSource {
    std::string label;
    pros::AbstractMotor* device = nullptr;
    std::uint8_t index = 1;

    SelectorTemperatureSource() = default;
    SelectorTemperatureSource(std::string sourceLabel, pros::AbstractMotor* sourceDevice, std::uint8_t sourceIndex = 1)
        : label(std::move(sourceLabel)),
          device(sourceDevice),
          index(sourceIndex) {}
};

struct SelectorDevicesConfig {
    std::array<SelectorTemperatureSource, 3> sources{};

    SelectorDevicesConfig() = default;
    SelectorDevicesConfig(SelectorTemperatureSource firstSource,
                          SelectorTemperatureSource secondSource,
                          SelectorTemperatureSource thirdSource)
        : sources{std::move(firstSource), std::move(secondSource), std::move(thirdSource)} {}
};

struct TerminalField {
    std::string label;
    TerminalValueGetter value = nullptr;
    std::uint8_t precision = 2;
};

struct SelectorTerminalConfig {
    std::vector<TerminalField> fields{};
    std::uint32_t refreshMs = 60;
};

struct SelectorConfig {
    SelectorInputConfig input{};
    SelectorMenuConfig menu{};
    SelectorDevicesConfig devices{};
    SelectorTerminalConfig terminal{};
    std::uint32_t lcdLine = 4;
    std::uint32_t pollDelayMs = 20;
};

class AutonSelector {
  public:
    AutonSelector(SelectorConfig config, AutonRoutineList routines);

    void start();
    void tick();

    std::size_t selectedIndex() const;
    const AutonRoutineList& routines() const;
    void runSelected(AutonFunc fallback) const;

  private:
    struct UiState {
        lv_obj_t* root = nullptr;
        lv_obj_t* backgroundImage = nullptr;
        lv_obj_t* logoSmallImage = nullptr;
        lv_obj_t* batteryLabel = nullptr;
        lv_obj_t* teamIconLabel = nullptr;
        lv_obj_t* headerLine = nullptr;
        lv_obj_t* teamLabel = nullptr;
        lv_obj_t* titleLabel = nullptr;
        lv_obj_t* subtitleLabel = nullptr;

        lv_obj_t* infoPanel = nullptr;
        lv_obj_t* infoHeader = nullptr;
        std::array<lv_obj_t*, 3> infoLines{nullptr, nullptr, nullptr};

        std::array<lv_obj_t*, 3> gaugeArcs{nullptr, nullptr, nullptr};
        std::array<lv_obj_t*, 3> gaugeValues{nullptr, nullptr, nullptr};
        std::array<lv_obj_t*, 3> gaugeNames{nullptr, nullptr, nullptr};

        lv_obj_t* selectorPanel = nullptr;
        lv_obj_t* selectorHighlight = nullptr;
        lv_obj_t* selectorTempRow = nullptr;
        lv_obj_t* selectorTempText = nullptr;
        std::array<lv_obj_t*, 3> selectorRows{nullptr, nullptr, nullptr};
        std::array<lv_obj_t*, 3> selectorTexts{nullptr, nullptr, nullptr};
    };

    static void selectorRowEventThunk(lv_event_t* event);
    static void selectorAnimDoneThunk(lv_anim_t* anim);

    void onSelectorRowTapped(lv_obj_t* rowObj);
    void buildUi();
    void updateUi();
    void updateInfoLines();
    void updateGauges();
    void updateSelectorList();
    void animateSelectorTransition(int direction);
    void onSelectorAnimDone();

    bool newPress() const;
    double readGaugeSource(std::size_t gaugeIndex) const;
    std::size_t visibleRowFromSelected() const;
    void advanceSelectionDown();

    SelectorConfig config_;
    AutonRoutineList routines_;
    std::size_t selectedIndex_ = 0;
    std::size_t listWindowStart_ = 0;
    std::unique_ptr<pros::Task> task_{};
    UiState ui_{};
    mutable pros::Mutex uiMutex_{};
    std::uint32_t uiElapsedMs_ = 0;
    std::atomic<int> pendingStep_{0};
    bool suppressUiEvent_ = false;
    bool selectorFirstPaint_ = true;
    bool selectorAnimating_ = false;
    std::uint32_t selectorAnimEndMs_ = 0;
    int selectorAnimDirection_ = 0;
};

}  // namespace robot
