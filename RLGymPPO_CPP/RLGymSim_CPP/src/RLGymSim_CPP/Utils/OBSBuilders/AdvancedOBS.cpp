#include "AdvancedOBS.h"

namespace RLGSC {
    FList AdvancedOBS::BuildOBS(const PlayerData& player, const GameState& state, const Action& prevAction) {
        FList obs = {};

        const PhysObj& ball = state.GetBallPhys(player.team == Team::ORANGE);
        const auto& pads = state.GetBoostPads(player.team == Team::ORANGE);

        obs += ball.pos / POS_STD;
        obs += ball.vel / POS_STD;
        obs += ball.angVel / ANG_STD;

        // Append previous action values to obs
        obs += prevAction.steer;
        obs += prevAction.throttle;
        obs += prevAction.pitch;
        obs += prevAction.yaw;
        obs += prevAction.roll;
        obs += static_cast<float>(prevAction.jump);
        obs += static_cast<float>(prevAction.boost);
        obs += static_cast<float>(prevAction.handbrake);

        // Convert boost pad states to floats and append to obs
        for (const auto& pad : pads) {
            obs += static_cast<float>(pad);
        }

        AddPlayerToOBS(obs, player, ball, player.team == Team::ORANGE);

        FList teammates = {}, opponents = {};

        for (const auto& otherPlayer : state.players) {
            if (otherPlayer.carId == player.carId)
                continue;

            const PhysObj& otherCar = otherPlayer.GetPhys(player.team == Team::ORANGE);
            const PhysObj& playerCar = player.GetPhys(player.team == Team::ORANGE);

            if (otherPlayer.team == player.team) {
                AddPlayerToOBS(teammates, otherPlayer, ball, player.team == Team::ORANGE);
                teammates += (otherCar.pos - playerCar.pos) / POS_STD;
                teammates += (otherCar.vel - playerCar.vel) / POS_STD;
            } else {
                AddPlayerToOBS(opponents, otherPlayer, ball, player.team == Team::ORANGE);
                opponents += (otherCar.pos - playerCar.pos) / POS_STD;
                opponents += (otherCar.vel - playerCar.vel) / POS_STD;
            }
        }

        obs += teammates;
        obs += opponents;
        return obs;
    }

    void AdvancedOBS::AddPlayerToOBS(FList& obs, const PlayerData& player, const PhysObj& ball, bool inverted) {
        const PhysObj& playerCar = player.GetPhys(inverted);

        Vec relPos = ball.pos - playerCar.pos;
        Vec relVel = ball.vel - playerCar.vel;

        obs += relPos / POS_STD;
        obs += relVel / POS_STD;
        obs += playerCar.pos / POS_STD;
        obs += playerCar.rotMat.forward;
        obs += playerCar.rotMat.up;
        obs += playerCar.vel / POS_STD;
        obs += playerCar.angVel / ANG_STD;
        obs += { player.boostFraction, static_cast<float>(player.carState.isOnGround), static_cast<float>(player.hasFlip), static_cast<float>(player.carState.isDemoed) };
    }
}