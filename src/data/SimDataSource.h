#pragma once

#include "data/SimSnapshot.h"

namespace navsight {

// Abstract producer of SimSnapshots. XPlaneDataSource is the live
// implementation; tests (and later a replay or companion-app link) can supply
// their own.
class SimDataSource {
public:
    virtual ~SimDataSource() = default;

    // Resolve whatever handles are needed. Returns false if the source can't
    // provide its essential data (the instrument then shows failure flags).
    virtual bool init() = 0;

    // Fill `out` with the current state. Must be cheap: called every frame.
    virtual void sample(SimSnapshot& out) = 0;
};

}  // namespace navsight
