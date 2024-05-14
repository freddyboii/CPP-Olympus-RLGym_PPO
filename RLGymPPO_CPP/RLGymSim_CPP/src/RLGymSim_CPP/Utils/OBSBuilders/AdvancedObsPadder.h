#pragma once
#include "OBSBuilder.h"

namespace RLGSC {
    class AdvancedOBSPadder : public OBSBuilder {
    public:
        static constexpr float POS_STD = 2300.0f;
        static constexpr float ANG_STD = static_cast<float>(M_PI);

        AdvancedOBSPadder(int teamSize = 3, bool expanding = true, bool shuffle = true) 
            : teamSize(teamSize), expanding(expanding), shuffle(shuffle) {}

        virtual FList BuildOBS(const PlayerData& player, const GameState& state, const Action& prevAction) override;

    private:
        void AddPlayerToOBS(FList& obs, const PlayerData& player, const PhysObj& ball, bool inverted);
        void AddDummy(FList& obs);
        int teamSize;
        bool expanding;
        bool shuffle;
    };
}
