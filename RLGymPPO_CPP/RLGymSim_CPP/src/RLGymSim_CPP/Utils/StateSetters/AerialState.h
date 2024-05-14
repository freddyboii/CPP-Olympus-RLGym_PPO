// AerialState.h
#pragma once
#include "StateSetter.h"

namespace RLGSC {
    class AerialState : public StateSetter {
    public:
        AerialState(float max_rand_z = 500.0f, float speed_min = 1375.0f, float speed_max = 1425.0f)
            : m_rand_z_max(max_rand_z), m_speed_min(speed_min), m_speed_max(speed_max) {}

        virtual GameState ResetState(Arena* arena);

    private:
        float m_rand_z_max;
        float m_speed_min;
        float m_speed_max;
    };
}
