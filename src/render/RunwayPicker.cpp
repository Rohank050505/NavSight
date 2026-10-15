#include "render/RunwayPicker.h"

#include <cctype>

#include "nav/RunwayOverride.h"

namespace navsight::render {
namespace {

// XPLM virtual key codes (XPLMDefs.h); duplicated so this stays X-Plane free.
constexpr int kVkBack = 0x08;
constexpr int kVkReturn = 0x0D;
constexpr int kVkEscape = 0x1B;
constexpr int kVkEnter = 0xBB;
constexpr int kVkNumpadEnter = 0xBC;

}  // namespace

void RunwayPicker::openFor(bool hasCourse, double course, const std::string& currentOverride) {
    open = true;
    courseValid = hasCourse;
    courseDegMag = course;
    candidates = hasCourse ? nav::runwayCandidates(course) : std::vector<std::string>{};
    active = currentOverride;
    typed.clear();
    side = 0;
    selected = candidates.empty() ? -1 : 1;  // the runway nearest the course
    if (!currentOverride.empty()) {
        // Pre-select the current override.
        const std::string num = currentOverride.substr(0, 2);
        side = currentOverride.size() > 2 ? currentOverride[2] : 0;
        selected = -1;
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i)
            if (candidates[i] == num) selected = i;
        if (selected < 0) typed = num;
    }
}

std::string RunwayPicker::designator() const {
    std::string base = typed;
    if (base.empty() && selected >= 0 && selected < static_cast<int>(candidates.size())) base = candidates[selected];
    if (base.empty()) return {};
    if (side) base += side;
    return nav::normaliseRunway(base);
}

std::string RunwayPicker::entryText() const {
    if (!typed.empty() && typed.size() < 2 && designator().empty()) return typed + "_";
    const std::string d = designator();
    return d.empty() ? "--" : d;
}

RunwayPicker::Result RunwayPicker::press(Hit hit) {
    switch (hit) {
        case Hit::PickerCandidate0:
        case Hit::PickerCandidate1:
        case Hit::PickerCandidate2: {
            const int i = static_cast<int>(hit) - static_cast<int>(Hit::PickerCandidate0);
            if (i < static_cast<int>(candidates.size())) {
                selected = i;
                typed.clear();
            }
            return {};
        }
        case Hit::PickerSideNone: side = 0; return {};
        case Hit::PickerSideL: side = 'L'; return {};
        case Hit::PickerSideC: side = 'C'; return {};
        case Hit::PickerSideR: side = 'R'; return {};
        case Hit::PickerAuto:
            open = false;
            return {Action::Clear, {}};
        case Hit::PickerApply: {
            const std::string d = designator();
            if (d.empty()) return {};
            open = false;
            return {Action::Apply, d};
        }
        case Hit::PickerOutside:
            open = false;
            return {Action::Close, {}};
        default:
            return {};
    }
}

RunwayPicker::Result RunwayPicker::key(char ch, int vkey) {
    if (vkey == kVkEscape) {
        open = false;
        return {Action::Close, {}};
    }
    if (vkey == kVkReturn || vkey == kVkEnter || vkey == kVkNumpadEnter) return press(Hit::PickerApply);
    if (vkey == kVkBack) {
        if (side) side = 0;
        else if (!typed.empty()) typed.pop_back();
        return {};
    }
    const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (std::isdigit(static_cast<unsigned char>(c))) {
        if (typed.size() >= 2) typed.clear();  // start a new number
        typed += c;
        selected = -1;
    } else if (c == 'L' || c == 'C' || c == 'R') {
        side = c;
    }
    return {};
}

}  // namespace navsight::render
