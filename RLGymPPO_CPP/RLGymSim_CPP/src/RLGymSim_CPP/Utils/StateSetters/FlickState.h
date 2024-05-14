// FlickSetter.h 
#pragma once
#include "StateSetter.h"

namespace RLGSC {
    class FlickState : public StateSetter {
    public:
        FlickState() {}

        virtual GameState ResetState(Arena* arena);
    };
}