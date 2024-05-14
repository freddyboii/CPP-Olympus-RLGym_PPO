#include "AdvancedOBSPadder.h"

namespace RLGSC {
    FList AdvancedOBSPadder::BuildOBS(const PlayerData& player, const GameState& state, const Action& prevAction) {
        FList obs = {};

        const PhysObj& ball = state.GetBallPhys(player.team == Team::ORANGE);
        const auto& pads = state.GetBoostPads(player.team == Team::ORANGE);

        obs += ball.pos / POS_STD;
        obs += ball.vel / POS_STD;
        obs += ball.angVel / ANG_STD;

        obs += prevAction.steer;
        obs += prevAction.throttle;
        obs += prevAction.pitch;
        obs += prevAction.yaw;
        obs += prevAction.roll;
        obs += static_cast<float>(prevAction.jump);
        obs += static_cast<float>(prevAction.boost);
        obs += static_cast<float>(prevAction.handbrake);

        for (const auto& pad : pads) {
            obs += static_cast<float>(pad);
        }

        AddPlayerToOBS(obs, player, ball, player.team == Team::ORANGE);

        std::vector<const PlayerData*> alliedPlayers, enemyPlayers;
        for (const auto& otherPlayer : state.players) {
            if (otherPlayer.carId == player.carId)
                continue;

            if (otherPlayer.team == player.team) {
                alliedPlayers.push_back(&otherPlayer);
            } else {
                enemyPlayers.push_back(&otherPlayer);
            }
        }

        // Add dummy allied players
        while (alliedPlayers.size() < teamSize - 1) {
            alliedPlayers.push_back(nullptr);
        }

        // Add dummy opponent players
        while (enemyPlayers.size() < teamSize) {
            enemyPlayers.push_back(nullptr);
        }

        if (shuffle) {
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(alliedPlayers.begin(), alliedPlayers.end(), g);
        std::shuffle(enemyPlayers.begin(), enemyPlayers.end(), g);
        }

        for (const auto* alliedPlayer : alliedPlayers) {
            if (alliedPlayer == nullptr) {
                AddDummy(obs);
            } else {
                const PhysObj& playerCar = player.GetPhys(player.team == Team::ORANGE);
                const PhysObj& alliedCar = alliedPlayer->GetPhys(player.team == Team::ORANGE);
                AddPlayerToOBS(obs, *alliedPlayer, ball, player.team == Team::ORANGE);
                obs += (alliedCar.pos - playerCar.pos) / POS_STD;
                obs += (alliedCar.vel - playerCar.vel) / POS_STD;
            }
        }

        for (const auto* enemyPlayer : enemyPlayers) {
            if (enemyPlayer == nullptr) {
                AddDummy(obs);
            } else {
                const PhysObj& playerCar = player.GetPhys(player.team == Team::ORANGE);
                const PhysObj& enemyCar = enemyPlayer->GetPhys(player.team == Team::ORANGE);
                AddPlayerToOBS(obs, *enemyPlayer, ball, player.team == Team::ORANGE);
                obs += (enemyCar.pos - playerCar.pos) / POS_STD;
                obs += (enemyCar.vel - playerCar.vel) / POS_STD;
            }
        }

        if (expanding) {
            return { obs };
        }
        return obs;
    }

    void AdvancedOBSPadder::AddPlayerToOBS(FList& obs, const PlayerData& player, const PhysObj& ball, bool inverted) {
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
        obs += { player.boostFraction, static_cast<float>(player.carState.isOnGround), static_cast<float>(player.hasFlip), static_cast<float>(player.carState.isDemoed),static_cast<float>(player.hasJump) };
    }

    void AdvancedOBSPadder::AddDummy(FList& obs) {
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f}; 
        obs += { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        obs += { 0.0f, 0.0f, 0.0f};
        obs += { 0.0f, 0.0f, 0.0f};
    }
}