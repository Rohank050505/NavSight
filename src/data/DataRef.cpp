#include "data/DataRef.h"

#include "core/Log.h"

namespace navsight {

bool DataRefBase::bind(XPLMDataTypeID wantType, const char* typeName) {
    ref_ = XPLMFindDataRef(name_.c_str());
    if (!ref_) {
        NAVS_ERROR("dataref not found: %s", name_.c_str());
        return false;
    }
    const XPLMDataTypeID types = XPLMGetDataRefTypes(ref_);
    if ((types & wantType) == 0) {
        NAVS_ERROR("dataref %s is not readable as %s (type mask 0x%x)", name_.c_str(), typeName,
                   static_cast<unsigned>(types));
        ref_ = nullptr;
        return false;
    }
    NAVS_DEBUG("bound dataref %s", name_.c_str());
    return true;
}

std::string StringDataRef::get() const {
    if (!ref_) return {};
    char buf[160] = {};
    const int n = XPLMGetDatab(ref_, buf, 0, static_cast<int>(sizeof(buf) - 1));
    if (n <= 0) return {};
    std::string s(buf, static_cast<std::size_t>(n));
    const auto end = s.find('\0');
    if (end != std::string::npos) s.resize(end);
    // Trim padding spaces.
    while (!s.empty() && (s.back() == ' ')) s.pop_back();
    return s;
}

}  // namespace navsight
