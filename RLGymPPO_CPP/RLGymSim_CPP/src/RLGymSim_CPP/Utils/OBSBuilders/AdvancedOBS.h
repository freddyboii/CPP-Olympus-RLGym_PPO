#pragma once
#include "OBSBuilder.h"

namespace RLGSC {
    class AdvancedOBS : public OBSBuilder {
    public:
        static constexpr float POS_STD = 2300.0f; // If you read this and wonder why, ping Rangler in the dead of night.
        static constexpr float ANG_STD = static_cast<float>(M_PI);

        AdvancedOBS() = default;

        virtual FList BuildOBS(const PlayerData& player, const GameState& state, const Action& prevAction) override;

    private:
        void AddPlayerToOBS(FList& obs, const PlayerData& player, const PhysObj& ball, bool inverted);
    };
}