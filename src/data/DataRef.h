#pragma once

#include <string>

#include "XPLMDataAccess.h"

// Thin, failure-tolerant wrappers around read-only X-Plane datarefs.
// A missing or wrongly-typed dataref is logged once at bind time and then
// reads return the fallback value, so a renamed dataref degrades the
// instrument instead of crashing the sim.
namespace navsight {

class DataRefBase {
public:
    explicit DataRefBase(const char* name) : name_(name) {}
    const std::string& name() const { return name_; }
    bool valid() const { return ref_ != nullptr; }

protected:
    bool bind(XPLMDataTypeID wantType, const char* typeName);
    XPLMDataRef ref_ = nullptr;

private:
    std::string name_;
};

class FloatDataRef : public DataRefBase {
public:
    using DataRefBase::DataRefBase;
    bool bind() { return DataRefBase::bind(xplmType_Float, "float"); }
    float get(float fallback = 0.0f) const { return ref_ ? XPLMGetDataf(ref_) : fallback; }
};

class DoubleDataRef : public DataRefBase {
public:
    using DataRefBase::DataRefBase;
    bool bind() { return DataRefBase::bind(xplmType_Double, "double"); }
    double get(double fallback = 0.0) const { return ref_ ? XPLMGetDatad(ref_) : fallback; }
};

class IntDataRef : public DataRefBase {
public:
    using DataRefBase::DataRefBase;
    bool bind() { return DataRefBase::bind(xplmType_Int, "int"); }
    int get(int fallback = 0) const { return ref_ ? XPLMGetDatai(ref_) : fallback; }
};

// byte[] datarefs holding a C string (e.g. nav idents).
class StringDataRef : public DataRefBase {
public:
    using DataRefBase::DataRefBase;
    bool bind() { return DataRefBase::bind(xplmType_Data, "byte[]"); }
    std::string get() const;
};

}  // namespace navsight
