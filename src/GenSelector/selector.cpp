#include "selector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <utility>

extern const lv_image_dsc_t background;
extern const lv_image_dsc_t logosmall;

namespace robot {
namespace {

constexpr std::int32_t kScreenWidth = 480;
constexpr std::int32_t kScreenHeight = 240;
constexpr std::int32_t kSafeTop = 0;
constexpr std::int32_t kTopRowY = kSafeTop + 8;

constexpr std::int32_t kInfoX = 12;
constexpr std::int32_t kInfoY = kSafeTop + 46;
constexpr std::int32_t kInfoW = 190;
constexpr std::int32_t kInfoH = 105;

constexpr std::int32_t kGaugeBaseY = kInfoY + kInfoH + 6;
constexpr std::int32_t kGaugeSize = 54;
constexpr std::int32_t kGaugeGap = 12;
constexpr std::int32_t kGaugeStartX = 14;
constexpr std::int32_t kGaugeLabelY = kGaugeBaseY + kGaugeSize + 2;

constexpr std::int32_t kSelectorX = 220;
constexpr std::int32_t kSelectorY = 70;
constexpr std::int32_t kSelectorW = 245;
constexpr std::int32_t kSelectorH = 155;
constexpr std::int32_t kSelectorInnerX = 8;
constexpr std::int32_t kSelectorInnerY = 7;
constexpr std::int32_t kSelectorRowH = 52;
constexpr std::size_t kVisibleRows = 3;
constexpr double kGaugeMaxValue = 50.0;
constexpr std::array<std::int32_t, 3> kSelectorRowYs = {22, 54, 102};
constexpr std::array<std::int32_t, 3> kSelectorRowHeights = {28, 42, 28};
constexpr std::int32_t kSelectorHighlightY = 54;
constexpr std::int32_t kSelectorTempTopY = -16;
constexpr std::int32_t kSelectorTempBottomY = 134;
constexpr std::int32_t kSelectorAnimTimeMs = 220;

constexpr std::array<const char*, 3> kGaugeNames = {"Chassis", "Intake", "Indexer"};

double clampDouble(double value, double low, double high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

void stylePanel(lv_obj_t* panel, std::int32_t width, std::int32_t height, lv_color_t border) {
    lv_obj_set_size(panel, width, height);
    lv_obj_set_style_radius(panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, border, LV_PART_MAIN);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x021a3a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 0, LV_PART_MAIN);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
}

std::string fallbackLabel(const std::string& label) {
    return label.empty() ? "?" : label;
}

}  // namespace

AutonSelector::AutonSelector(SelectorConfig config, AutonRoutineList routines)
    : config_(std::move(config)),
      routines_(std::move(routines)) {}

void AutonSelector::start() {
    if (task_ != nullptr) {
        return;
    }

    {
        std::lock_guard<pros::Mutex> lock(uiMutex_);
        buildUi();
        updateUi();
    }

    task_ = std::make_unique<pros::Task>([this]() {
        const std::uint32_t delayMs = std::max<std::uint32_t>(1, config_.pollDelayMs);
        while (true) {
            tick();
            pros::delay(delayMs);
        }
    }, "Auton Selector");
}

void AutonSelector::tick() {
    std::lock_guard<pros::Mutex> lock(uiMutex_);

    const int pendingStep = pendingStep_.exchange(0);
    if (!routines_.empty()) {
        if (pendingStep < 0) {
            selectedIndex_ = (selectedIndex_ + routines_.size() - 1) % routines_.size();
            updateSelectorList();
        } else if (pendingStep > 0) {
            advanceSelectionDown();
        }
    }

    if (!routines_.empty() && newPress()) {
        advanceSelectionDown();
    }

    uiElapsedMs_ += std::max<std::uint32_t>(1, config_.pollDelayMs);
    if (uiElapsedMs_ >= std::max<std::uint32_t>(1, config_.terminal.refreshMs)) {
        uiElapsedMs_ = 0;
        updateUi();
    }
}

std::size_t AutonSelector::selectedIndex() const {
    return selectedIndex_;
}

const AutonRoutineList& AutonSelector::routines() const {
    return routines_;
}

void AutonSelector::runSelected(AutonFunc fallback) const {
    AutonFunc selected = fallback;
    {
        std::lock_guard<pros::Mutex> lock(uiMutex_);
        if (selectedIndex_ < routines_.size() && routines_[selectedIndex_].second != nullptr) {
            selected = routines_[selectedIndex_].second;
        }
    }

    if (selected != nullptr) {
        selected();
    }
}

void AutonSelector::selectorRowEventThunk(lv_event_t* event) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(event));
    if (self == nullptr) {
        return;
    }
    self->onSelectorRowTapped(static_cast<lv_obj_t*>(lv_event_get_target(event)));
}

void AutonSelector::selectorAnimDoneThunk(lv_anim_t* anim) {
    auto* self = static_cast<AutonSelector*>(lv_anim_get_user_data(anim));
    if (self == nullptr) {
        return;
    }
    self->onSelectorAnimDone();
}

void AutonSelector::onSelectorRowTapped(lv_obj_t* rowObj) {
    if (suppressUiEvent_ || config_.input.type != SelectorInputType::BrainScreen || routines_.empty()) {
        return;
    }

    if (rowObj != nullptr) {
        const lv_obj_t* parent = lv_obj_get_parent(rowObj);
        if (parent == ui_.selectorHighlight) {
            rowObj = ui_.selectorHighlight;
        } else {
            for (std::size_t i = 0; i < kVisibleRows; ++i) {
                if (parent == ui_.selectorRows[i]) {
                    rowObj = ui_.selectorRows[i];
                    break;
                }
            }
        }
    }

    if (rowObj == ui_.selectorHighlight) {
        pendingStep_.store(1);
        return;
    }

    for (std::size_t i = 0; i < kVisibleRows; ++i) {
        if (ui_.selectorRows[i] != rowObj) {
            continue;
        }
        if (i == 0) {
            pendingStep_.store(-1);
        } else {
            pendingStep_.store(1);
        }
        return;
    }
}

void AutonSelector::buildUi() {
    lv_obj_t* screen = lv_screen_active();
    lv_obj_clean(screen);
    lv_obj_set_size(screen, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    ui_.backgroundImage = lv_image_create(screen);
    lv_image_set_src(ui_.backgroundImage, &background);
    lv_obj_set_pos(ui_.backgroundImage, 0, 0);
    lv_obj_set_size(ui_.backgroundImage, kScreenWidth, kScreenHeight);
    // lv_image_set_inner_align(ui_.backgroundImage, LV_IMAGE_ALIGN_STRETCH);

    ui_.batteryLabel = lv_label_create(screen);
    lv_obj_set_style_text_color(ui_.batteryLabel, lv_color_hex(0x49b6ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.batteryLabel, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_pos(ui_.batteryLabel, 14, kTopRowY);

    lv_obj_t* dividerLabel = lv_label_create(screen);
    lv_label_set_text(dividerLabel, "|");
    lv_obj_set_style_text_color(dividerLabel, lv_color_hex(0x4e8dcc), LV_PART_MAIN);
    lv_obj_set_style_text_font(dividerLabel, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_pos(dividerLabel, 90, kTopRowY - 3);

    ui_.teamIconLabel = lv_obj_create(screen);
    lv_obj_set_pos(ui_.teamIconLabel, 106, kTopRowY + 4);
    lv_obj_set_size(ui_.teamIconLabel, 16, 16);
    lv_obj_set_style_radius(ui_.teamIconLabel, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_.teamIconLabel, lv_color_hex(0x7ec6ff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_.teamIconLabel, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_.teamIconLabel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(ui_.teamIconLabel, lv_color_hex(0x9ddbff), LV_PART_MAIN);
    lv_obj_set_style_pad_all(ui_.teamIconLabel, 0, LV_PART_MAIN);
    lv_obj_remove_flag(ui_.teamIconLabel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* teamIconHead = lv_obj_create(ui_.teamIconLabel);
    lv_obj_set_size(teamIconHead, 5, 5);
    lv_obj_set_pos(teamIconHead, 4, 1);
    lv_obj_set_style_radius(teamIconHead, 3, LV_PART_MAIN);
    lv_obj_set_style_bg_color(teamIconHead, lv_color_hex(0xd8f1ff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(teamIconHead, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(teamIconHead, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(teamIconHead, 0, LV_PART_MAIN);
    lv_obj_remove_flag(teamIconHead, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* teamIconBody = lv_obj_create(ui_.teamIconLabel);
    lv_obj_set_size(teamIconBody, 10, 5);
    lv_obj_set_pos(teamIconBody, 2, 8);
    lv_obj_set_style_radius(teamIconBody, 3, LV_PART_MAIN);
    lv_obj_set_style_bg_color(teamIconBody, lv_color_hex(0xd8f1ff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(teamIconBody, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(teamIconBody, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(teamIconBody, 0, LV_PART_MAIN);
    lv_obj_remove_flag(teamIconBody, LV_OBJ_FLAG_SCROLLABLE);

    ui_.teamLabel = lv_label_create(screen);
    lv_obj_set_style_text_color(ui_.teamLabel, lv_color_hex(0x8cc9ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.teamLabel, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_pos(ui_.teamLabel, 130, kTopRowY + 1);

    ui_.logoSmallImage = lv_image_create(screen);
    lv_image_set_src(ui_.logoSmallImage, &logosmall);
    lv_obj_set_pos(ui_.logoSmallImage, 211, kTopRowY - 4);
    lv_obj_set_size(ui_.logoSmallImage, 45, 45);
    lv_image_set_inner_align(ui_.logoSmallImage, LV_IMAGE_ALIGN_STRETCH);

    lv_obj_t* titleDivider = lv_label_create(screen);
    lv_label_set_text(titleDivider, "|");
    lv_obj_set_style_text_color(titleDivider, lv_color_hex(0x6cb8f8), LV_PART_MAIN);
    lv_obj_set_style_text_font(titleDivider, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_pos(titleDivider, 258, kTopRowY - 1);

    ui_.titleLabel = lv_label_create(screen);
    lv_label_set_text(ui_.titleLabel, "Gen-Selector");
    lv_obj_set_style_text_color(ui_.titleLabel, lv_color_hex(0x6ec0ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.titleLabel, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_pos(ui_.titleLabel, 272, kTopRowY);

    ui_.subtitleLabel = lv_label_create(screen);
    lv_label_set_text(ui_.subtitleLabel, "Created by 78181A Genesis");
    lv_obj_set_style_text_color(ui_.subtitleLabel, lv_color_hex(0x8ab9e2), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.subtitleLabel, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_pos(ui_.subtitleLabel, 305, kTopRowY + 35);

    ui_.infoPanel = lv_obj_create(screen);
    lv_obj_set_pos(ui_.infoPanel, kInfoX, kInfoY);
    stylePanel(ui_.infoPanel, kInfoW, kInfoH, lv_color_hex(0x2f79c4));

    ui_.infoHeader = lv_label_create(ui_.infoPanel);
    lv_label_set_text(ui_.infoHeader, "Info");
    lv_obj_set_style_text_color(ui_.infoHeader, lv_color_hex(0x6ec0ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.infoHeader, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_pos(ui_.infoHeader, 10, 4);

    lv_obj_t* infoDivider = lv_obj_create(ui_.infoPanel);
    lv_obj_set_pos(infoDivider, 0, 26);
    lv_obj_set_size(infoDivider, kInfoW - 2, 1);
    lv_obj_set_style_border_width(infoDivider, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(infoDivider, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(infoDivider, lv_color_hex(0x2a6ca8), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(infoDivider, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_pad_all(infoDivider, 0, LV_PART_MAIN);
    lv_obj_remove_flag(infoDivider, LV_OBJ_FLAG_SCROLLABLE);

    for (std::size_t i = 0; i < ui_.infoLines.size(); ++i) {
        ui_.infoLines[i] = lv_label_create(ui_.infoPanel);
        lv_obj_set_style_text_color(ui_.infoLines[i], lv_color_hex(0xb2d8ff), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.infoLines[i], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_set_pos(ui_.infoLines[i], 10, 31 + static_cast<std::int32_t>(i * 22));
    }

    for (std::size_t i = 0; i < 3; ++i) {
        const std::int32_t x = kGaugeStartX + static_cast<std::int32_t>(i) * (kGaugeSize + kGaugeGap);

        ui_.gaugeArcs[i] = lv_arc_create(screen);
        lv_obj_set_pos(ui_.gaugeArcs[i], x, 163);
        lv_obj_set_size(ui_.gaugeArcs[i], kGaugeSize, kGaugeSize);
        lv_arc_set_rotation(ui_.gaugeArcs[i], 135);
        lv_arc_set_bg_angles(ui_.gaugeArcs[i], 0, 270);
        lv_arc_set_value(ui_.gaugeArcs[i], 0);
        lv_obj_set_style_arc_width(ui_.gaugeArcs[i], 7, LV_PART_MAIN);
        lv_obj_set_style_arc_width(ui_.gaugeArcs[i], 7, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(ui_.gaugeArcs[i], lv_color_hex(0x2f6ca8), LV_PART_MAIN);
        lv_obj_set_style_arc_color(ui_.gaugeArcs[i], lv_color_hex(0x4ab1ff), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(ui_.gaugeArcs[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_remove_style(ui_.gaugeArcs[i], nullptr, LV_PART_KNOB);
        lv_obj_remove_flag(ui_.gaugeArcs[i], LV_OBJ_FLAG_CLICKABLE);

        ui_.gaugeValues[i] = lv_label_create(screen);
        lv_obj_set_style_text_color(ui_.gaugeValues[i], lv_color_hex(0x4ab1ff), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.gaugeValues[i], &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_width(ui_.gaugeValues[i], kGaugeSize);
        lv_obj_set_style_text_align(ui_.gaugeValues[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(ui_.gaugeValues[i], x, 180);

        ui_.gaugeNames[i] = lv_label_create(screen);
        lv_label_set_text(ui_.gaugeNames[i], kGaugeNames[i]);
        lv_obj_set_style_text_color(ui_.gaugeNames[i], lv_color_hex(0x87bee8), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.gaugeNames[i], &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_width(ui_.gaugeNames[i], kGaugeSize + 16);
        lv_obj_set_style_text_align(ui_.gaugeNames[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(ui_.gaugeNames[i], x - 8, 216);
    }

    ui_.selectorPanel = lv_obj_create(screen);
    lv_obj_set_pos(ui_.selectorPanel, kSelectorX, kSelectorY);
    stylePanel(ui_.selectorPanel, kSelectorW, kSelectorH, lv_color_hex(0x1d65a5));

    ui_.selectorHighlight = lv_obj_create(ui_.selectorPanel);
    lv_obj_set_pos(ui_.selectorHighlight, kSelectorInnerX, kSelectorHighlightY);
    lv_obj_set_size(ui_.selectorHighlight, kSelectorW - 2 * kSelectorInnerX, 42);
    lv_obj_set_style_radius(ui_.selectorHighlight, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_.selectorHighlight, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(ui_.selectorHighlight, lv_color_hex(0xaadfff), LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_.selectorHighlight, lv_color_hex(0xa6ddff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_.selectorHighlight, LV_OPA_20, LV_PART_MAIN);
    lv_obj_remove_flag(ui_.selectorHighlight, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ui_.selectorHighlight, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui_.selectorHighlight, AutonSelector::selectorRowEventThunk, LV_EVENT_CLICKED, this);

    ui_.selectorTempRow = lv_obj_create(ui_.selectorPanel);
    lv_obj_set_size(ui_.selectorTempRow, kSelectorW - 2 * kSelectorInnerX, kSelectorRowHeights[0]);
    lv_obj_set_style_bg_opa(ui_.selectorTempRow, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_.selectorTempRow, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ui_.selectorTempRow, 0, LV_PART_MAIN);
    lv_obj_remove_flag(ui_.selectorTempRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ui_.selectorTempRow, LV_OBJ_FLAG_HIDDEN);

    ui_.selectorTempText = lv_label_create(ui_.selectorTempRow);
    lv_obj_set_width(ui_.selectorTempText, kSelectorW - 2 * kSelectorInnerX);
    lv_obj_set_style_text_align(ui_.selectorTempText, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(ui_.selectorTempText, 0, 0);

    for (std::size_t i = 0; i < kVisibleRows; ++i) {
        ui_.selectorRows[i] = lv_obj_create(ui_.selectorPanel);
        lv_obj_set_pos(ui_.selectorRows[i], kSelectorInnerX, kSelectorRowYs[i]);
        lv_obj_set_size(ui_.selectorRows[i], kSelectorW - 2 * kSelectorInnerX, kSelectorRowHeights[i]);
        lv_obj_set_style_bg_opa(ui_.selectorRows[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(ui_.selectorRows[i], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(ui_.selectorRows[i], 0, LV_PART_MAIN);
        lv_obj_remove_flag(ui_.selectorRows[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(ui_.selectorRows[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(ui_.selectorRows[i], AutonSelector::selectorRowEventThunk, LV_EVENT_CLICKED, this);

        ui_.selectorTexts[i] = lv_label_create(ui_.selectorRows[i]);
        lv_obj_set_width(ui_.selectorTexts[i], kSelectorW - 2 * kSelectorInnerX);
        lv_obj_set_style_text_align(ui_.selectorTexts[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(ui_.selectorTexts[i], 0, (i == 1) ? 6 : 0);
        lv_obj_add_flag(ui_.selectorTexts[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    }
}

void AutonSelector::updateUi() {
    if (ui_.batteryLabel != nullptr) {
        const int batteryPercent = static_cast<int>(std::lround(pros::battery::get_capacity()));
        lv_label_set_text_fmt(ui_.batteryLabel, LV_SYMBOL_BATTERY_3 " %d%%", batteryPercent);
    }
    if (ui_.teamLabel != nullptr) {
        lv_label_set_text_fmt(ui_.teamLabel, "%s", config_.menu.teamNumber.c_str());
    }
    for (std::size_t i = 0; i < ui_.gaugeNames.size(); ++i) {
        if (ui_.gaugeNames[i] == nullptr) {
            continue;
        }
        const std::string label = i < config_.devices.sources.size() ? fallbackLabel(config_.devices.sources[i].label) : std::string(kGaugeNames[i]);
        lv_label_set_text(ui_.gaugeNames[i], label.c_str());
    }
    updateInfoLines();
    updateGauges();
    updateSelectorList();
}

void AutonSelector::updateInfoLines() {
    const std::size_t fieldCount = std::min<std::size_t>(3, config_.terminal.fields.size());

    for (std::size_t i = 0; i < 3; ++i) {
        if (ui_.infoLines[i] == nullptr) {
            continue;
        }

        if (i >= fieldCount) {
            static const std::array<const char*, 3> defaults = {"x: 0", "y: 0", "theta: 0"};
            lv_label_set_text(ui_.infoLines[i], defaults[i]);
            continue;
        }

        const auto& field = config_.terminal.fields[i];
        const std::string label = fallbackLabel(field.label);
        const double value = field.value == nullptr ? 0.0 : field.value();
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "%s: %.*f", label.c_str(), static_cast<int>(field.precision), value);
        lv_label_set_text(ui_.infoLines[i], buffer);
    }
}

void AutonSelector::updateGauges() {
    for (std::size_t i = 0; i < 3; ++i) {
        const double value = readGaugeSource(i);
        const double clamped = clampDouble(value, 0.0, kGaugeMaxValue);
        const int percent = static_cast<int>(std::lround((clamped / kGaugeMaxValue) * 100.0));
        const int shown = static_cast<int>(std::lround(clamped));

        if (ui_.gaugeArcs[i] != nullptr) {
            lv_arc_set_value(ui_.gaugeArcs[i], percent);
        }
        if (ui_.gaugeValues[i] != nullptr) {
            lv_label_set_text_fmt(ui_.gaugeValues[i], "%d", shown);
        }
    }
}

void AutonSelector::updateSelectorList() {
    if (routines_.empty()) {
        for (auto* label : ui_.selectorTexts) {
            if (label != nullptr) {
                lv_label_set_text(label, "-");
            }
        }
        return;
    }

    if (ui_.selectorTempRow != nullptr) {
        lv_obj_add_flag(ui_.selectorTempRow, LV_OBJ_FLAG_HIDDEN);
    }
    if (ui_.selectorHighlight != nullptr) {
        lv_obj_set_y(ui_.selectorHighlight, kSelectorHighlightY);
    }

    const std::size_t prevIndex = (selectedIndex_ + routines_.size() - 1) % routines_.size();
    const std::size_t nextIndex = (selectedIndex_ + 1) % routines_.size();

    for (std::size_t i = 0; i < kVisibleRows; ++i) {
        if (ui_.selectorRows[i] == nullptr) {
            continue;
        }
        lv_obj_set_y(ui_.selectorRows[i], kSelectorRowYs[i]);
        lv_obj_set_height(ui_.selectorRows[i], kSelectorRowHeights[i]);
        if (ui_.selectorTexts[i] != nullptr) {
            lv_obj_set_y(ui_.selectorTexts[i], (i == 1) ? 6 : 0);
        }
    }

    lv_label_set_text(ui_.selectorTexts[0], routines_[prevIndex].first.c_str());
    lv_obj_set_style_text_color(ui_.selectorTexts[0], lv_color_hex(0xc2ddf7), LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_.selectorTexts[0], LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.selectorTexts[0], &lv_font_montserrat_18, LV_PART_MAIN);

    lv_label_set_text_fmt(ui_.selectorTexts[1], "> %s <", routines_[selectedIndex_].first.c_str());
    lv_obj_set_style_text_color(ui_.selectorTexts[1], lv_color_hex(0xeaf7ff), LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_.selectorTexts[1], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.selectorTexts[1], &lv_font_montserrat_20, LV_PART_MAIN);

    lv_label_set_text(ui_.selectorTexts[2], routines_[nextIndex].first.c_str());
    lv_obj_set_style_text_color(ui_.selectorTexts[2], lv_color_hex(0xc2ddf7), LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_.selectorTexts[2], LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.selectorTexts[2], &lv_font_montserrat_18, LV_PART_MAIN);
}

void AutonSelector::animateSelectorTransition(int direction) {
    if (routines_.empty() || direction == 0) {
        return;
    }
    if (direction > 0) {
        selectedIndex_ = (selectedIndex_ + 1) % routines_.size();
    } else {
        selectedIndex_ = (selectedIndex_ + routines_.size() - 1) % routines_.size();
    }
    updateSelectorList();
}

void AutonSelector::onSelectorAnimDone() {
    updateSelectorList();
}

bool AutonSelector::newPress() const {
    switch (config_.input.type) {
        case SelectorInputType::BrainScreen:
            return false;

        case SelectorInputType::ControllerButton:
            if (config_.input.controller == nullptr) {
                return false;
            }
            return config_.input.controller->get_digital_new_press(config_.input.controllerButton);

        case SelectorInputType::AdiDigitalIn:
            if (config_.input.adiDigitalIn == nullptr) {
                return false;
            }
            return config_.input.adiDigitalIn->get_new_press();

        case SelectorInputType::Custom:
            if (config_.input.customNewPress == nullptr) {
                return false;
            }
            return config_.input.customNewPress(config_.input.customDevice);
    }

    return false;
}

double AutonSelector::readGaugeSource(std::size_t gaugeIndex) const {
    if (gaugeIndex < config_.devices.sources.size() && config_.devices.sources[gaugeIndex].device != nullptr) {
        const auto& source = config_.devices.sources[gaugeIndex];
        const std::uint8_t index = source.device->size() > source.index ? source.index : 0;
        return source.device->get_temperature(index);
    }

    if (gaugeIndex < config_.terminal.fields.size() && config_.terminal.fields[gaugeIndex].value != nullptr) {
        return config_.terminal.fields[gaugeIndex].value();
    }

    return 0.0;
}

std::size_t AutonSelector::visibleRowFromSelected() const {
    return 0;
}

void AutonSelector::advanceSelectionDown() {
    if (routines_.empty()) {
        return;
    }

    selectedIndex_ = (selectedIndex_ + 1) % routines_.size();
    updateSelectorList();
}

}  // namespace robot
