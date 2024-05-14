#pragma once
#include "RewardFunction.h"
#include <cmath> // For std::sqrt
#include "../Gamestates/GameState.h"
#include "../BasicTypes/Action.h"
#include <optional>
#include <algorithm>
#include <vector>
#include <array>
#include <numeric>
#include "../RocketSim/src/Math/MathTypes/MathTypes.h" // Ensure this path is correct

namespace RLGSC {
	

	/*Reward List
    
    WaveDash
    Energy
    CradleFlick
    FlipReset
    Aerial
    
    
    */

	class WaveDashReward2 : public RewardFunction {
    public:
        WaveDashReward2() = default;

        virtual void Reset(const GameState& initialState) override {
            for (auto& player : initialState.players) {
                playerData[player.carId] = {0, 0, false};
            }
        }

        virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
            const float FLIP_TORQUE_TIME = 0.65f;
            const float DOUBLEJUMP_MAX_DELAY = 1.25f;
            const float MAX_FORWARD_SPEED = CommonValues::CAR_MAX_SPEED;

            float reward = 0.0f;
            auto& playerInfo = playerData[player.carId];

            // Check if the player is not in the air, has recently jumped, and has flipped
            if (player.phys.pos[2] < 100 && player.carState.isOnGround && playerInfo.airTime < DOUBLEJUMP_MAX_DELAY &&
                playerInfo.hasFlipped && playerInfo.flipTime > 0.0f) {
                // Calculate the reward based on the flip torque duration and forward speed
                Vec dirToBall = (state.ball.pos - player.phys.pos).Normalized();
                Vec normVel = player.phys.vel / MAX_FORWARD_SPEED;
                float dashReward = (FLIP_TORQUE_TIME - player.carState.flipTime) / FLIP_TORQUE_TIME;
                float speedReward = player.phys.vel.Length() / MAX_FORWARD_SPEED;

                reward = dashReward * speedReward;
            }

            playerInfo.airTime = player.carState.airTimeSinceJump;
            playerInfo.flipTime = player.carState.flipTime;
            playerInfo.hasFlipped = player.carState.hasFlipped;

            return reward;
        }

    private:
        struct PlayerInfo {
            float airTime;
            float flipTime;
            bool hasFlipped;
        };

        std::unordered_map<int, PlayerInfo> playerData;
    };


    class DribbleReward : public RewardFunction {
    public:

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        const float MIN_BALL_HEIGHT = 109.0f;
        const float MAX_BALL_HEIGHT = 180.0f;
        const float MAX_DISTANCE = 197.0f;
        const float SPEED_MATCH_FACTOR = 2.0f; // Adjust this value to control the importance of speed matching
        const float CAR_MAX_SPEED = CommonValues::CAR_MAX_SPEED;

        if (player.carState.isOnGround && state.ball.pos.z >= MIN_BALL_HEIGHT && state.ball.pos.z <= MAX_BALL_HEIGHT && (player.phys.pos - state.ball.pos).Length() < MAX_DISTANCE) {
            float playerSpeed = player.phys.vel.Length();
            float ballSpeed = state.ball.vel.Length();
            float speedMatchReward = ((playerSpeed/CAR_MAX_SPEED) + SPEED_MATCH_FACTOR * (1.0f - std::abs(playerSpeed - ballSpeed) / (playerSpeed + ballSpeed))) / 2.0f;
            return speedMatchReward; // Reward for successful dribbling, with a bonus for speed matching, normalized to 1
        } else {
            return 0.0f; // No reward
        }
    }
    }; 



	class EnergyReward : public RewardFunction {
    public:
    EnergyReward(float rewardWeight = 1.0f) : rewardWeight(rewardWeight) {}

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        constexpr float MASS = 180.0f; // Assuming mass is 180.0f, adjust as necessary
        constexpr float GRAVITY = CommonValues::GRAVITY_Z;
        constexpr float CEILING_Z = CommonValues::CEILING_Z;
        constexpr float CAR_MAX_SPEED = CommonValues::CAR_MAX_SPEED;

        float max_energy = (MASS * GRAVITY * (CEILING_Z - 17.0f)) +
                           (0.5f * MASS * (CAR_MAX_SPEED * CAR_MAX_SPEED));
        float energy = 0.0f;

        if (player.hasJump) {
            energy += 0.35f * MASS * 292.0f * 292.0f;
        }
        if (player.hasFlip) {
            energy += 0.35f * MASS * 550.0f * 550.0f;
        }
        // Height
        energy += MASS * GRAVITY * (player.phys.pos.z - 17.0f) * 0.75f; // Fudge factor to reduce height
        // KE
        float player_norm_vel = player.phys.vel.Length();
        energy += 0.5f * MASS * player_norm_vel * player_norm_vel;
        // Boost
        energy += 7.97e6f * player.boostFraction * 100.0f;

        float std_energy = player.carState.isDemoed ? 0.0f : (energy / max_energy);
        float rew = std_energy * rewardWeight;

        return rew;
    }

    private:
    float rewardWeight;
    };

    


    class FlipResetReward : public RewardFunction {
    public:
    FlipResetReward(float flipResetR = 1.0f , float holdFlipResetR = 0.0f)
        : flipResetR(flipResetR), holdFlipResetR(holdFlipResetR) {
        prevCanJump.fill(false);
        hasReset.fill(false);
    }

    virtual void Reset(const GameState& initialState) override {
        prevCanJump.fill(false);
        hasReset.fill(false);
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        const float BALL_RADIUS = CommonValues::BALL_RADIUS;
        const float CEILING_Z = CommonValues::CEILING_Z;
        const float SIDE_WALL_X = CommonValues::SIDE_WALL_X;
        const float BACK_WALL_Y = CommonValues::BACK_WALL_Y;
        const float BALL_UNDERCARRIAGE_THRESHOLD = 0.7f;

        size_t carId = player.carId;
        float reward = 0.0f;

        bool nearBall = (player.phys.pos - state.ball.pos).Length() < 170.0f;
        bool heightCheck = (player.phys.pos.z < 300.0f) || (player.phys.pos.z > CEILING_Z - 300.0f);
        bool wallDisCheck = ((-SIDE_WALL_X + 700.0f) > player.phys.pos.x) ||
                            ((SIDE_WALL_X - 700.0f) < player.phys.pos.x) ||
                            ((-BACK_WALL_Y + 700.0f) > player.phys.pos.y) ||
                            ((BACK_WALL_Y - 700.0f) < player.phys.pos.y);
        bool canJump = !player.carState.hasJumped;
        bool hasFlipped = player.carState.isJumping && player.carState.isFlipping;

        if (wallDisCheck || hasFlipped) {
            hasReset[carId] = false;
        }

        if (nearBall && !heightCheck && !wallDisCheck) {
            bool gotReset = prevCanJump[carId] < canJump;
            if (gotReset) {
                hasReset[carId] = true;
                reward = flipResetR; // Reward when a flip reset is detected
            }
        }

        if (hasReset[carId]) {
            reward += holdFlipResetR; // Reward if holds reset
        }

        prevCanJump[carId] = canJump;

        return reward;
    }

    private:
    std::array<bool, 7> prevCanJump;
    std::array<bool, 7> hasReset;
    float flipResetR;
    float holdFlipResetR;
    };



   
 

    class BouncyAirDribbleReward : public RewardFunction {
    public:
    BouncyAirDribbleReward(float heightThreshold = 200.0f, float touchRewardBase = 50.0f, float heightMultiplier = 0.5f,
                           float minBoost = 15.0f, float minPlayerToBallZ = 0.5f, float minPlayerToBallSpeed = 1000.0f,
                           float minPlayerToBallDist = 100.0f, float minPlayerToGoalSpeed = 1000.0f, float maxBallHeightAboveCrossbar = 100.0f,
                           float nearGoalShutoffTime = 0.0f)
        : heightThreshold(heightThreshold), touchRewardBase(touchRewardBase), heightMultiplier(heightMultiplier),
          minBoost(minBoost), minPlayerToBallZ(minPlayerToBallZ), minPlayerToBallSpeed(minPlayerToBallSpeed),
          minPlayerToBallDist(minPlayerToBallDist), minPlayerToGoalSpeed(minPlayerToGoalSpeed),
          maxBallHeightAboveCrossbar(maxBallHeightAboveCrossbar), nearGoalShutoffTime(nearGoalShutoffTime) {}

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        float reward = 0.0f; // Start with a minimum reward of 0

        Vec ballPos = state.ball.pos;
        Vec playerPos = player.phys.pos;
        Vec ballToGoal = Vec(0, CommonValues::BACK_WALL_Y, 0) - ballPos; // Use CommonValues for goal position
        Vec playerToBall = ballPos - playerPos;
        Vec playerToGoal = Vec(0, CommonValues::BACK_WALL_Y, 0) - playerPos; // Use CommonValues for goal position

        // Check if the ball is high up in the air or the player and ball are gaining height quickly
        bool isHighEnough = ballPos.z > heightThreshold || (player.phys.vel.z > 0 && state.ball.vel.z > 0);
        bool isBouncy = (player.phys.vel - state.ball.vel).Length() > minPlayerToBallSpeed || playerToBall.Length() > minPlayerToBallDist;
        bool isNearGoal = playerToGoal.Length() < 5000;
        bool hasEnoughBoost = player.boostFraction > minBoost / 100.0f;
        bool isMovingTowardsGoal = playerToGoal.Normalized().Dot(player.phys.vel.Normalized()) > 0.8f;
        bool isPlayerToBallDirectionNearGoal = playerToBall.Normalized().z > minPlayerToBallZ;
        bool isBallHeightAboveCrossbarAcceptable = ballPos.z + ballToGoal.Length() * ballToGoal.Normalized().z < maxBallHeightAboveCrossbar;
        bool isNearGoalShutoff = ballToGoal.Length() / ballToGoal.Normalized().z < nearGoalShutoffTime;

        if (isHighEnough && isBouncy && isNearGoal && hasEnoughBoost && isMovingTowardsGoal &&
            isPlayerToBallDirectionNearGoal && isBallHeightAboveCrossbarAcceptable && !isNearGoalShutoff) {
            reward += touchRewardBase; // Base reward for a bouncy air dribble
            reward += (ballPos.z - heightThreshold) * heightMultiplier; // Additional reward based on height
        }

        return reward;
    }

    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
        if (rewardName == "BouncyAirDribbleReward") {
            return GetReward(player, state, Action());
        }
        return 0.0f;
    }

    private:
    float heightThreshold;
    float touchRewardBase;
    float heightMultiplier;
    float minBoost;
    float minPlayerToBallZ;
    float minPlayerToBallSpeed;
    float minPlayerToBallDist;
    float minPlayerToGoalSpeed;
    float maxBallHeightAboveCrossbar;
    float nearGoalShutoffTime;
    };



    class ContinuousFlipResetReward : public RewardFunction {
    public:
        ContinuousFlipResetReward(float minResetHeight = 300.0f, float minPlayerToBallDist = 100.0f)
            : minResetHeight(minResetHeight), minPlayerToBallDist(minPlayerToBallDist), lastCloseness(0.0f) {}
    
        virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
            float reward = 0.0f;
            Vec ballPos = state.ball.pos;
            Vec playerPos = player.phys.pos;
            Vec playerToBall = ballPos - playerPos;
    
            // Check conditions for flip reset
            bool isHighEnough = playerPos.z > minResetHeight;
            bool isFacingDown = player.phys.vel.z < 0; // Simplified check, adjust as necessary
            bool isNearBall = playerToBall.Length() < minPlayerToBallDist;
            bool isMovingTowardsBall = player.phys.vel.Dot(playerToBall.Normalized()) > 0.0f;
    
            if (isHighEnough && isFacingDown && isNearBall && isMovingTowardsBall) {
                // Calculate proximity to flip reset
                float relativeSpeedTowardsBall = (player.phys.vel - state.ball.vel).Dot(playerToBall.Normalized());
                float distanceToBall = playerToBall.Length() / minPlayerToBallDist;

                // Since we cannot directly access player.phys.rot.Roll(), we simplify the check for facing downwards
                // For a more nuanced alignment check, additional logic would be needed based on available data

                // Since we cannot directly access player.phys.rot.Roll(), we omit downwardAlignment from the calculation
                float closeness = std::min({relativeSpeedTowardsBall, distanceToBall});
    
                // Normalize measurements and calculate overall reset proximity
    
                // Reward based on getting closer to the reset
                reward = std::max(0.0f, closeness - lastCloseness);
                lastCloseness = closeness;
            }
    
            return reward;
        }
    
    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
        if (rewardName == "ContinuousFlipResetReward") {
            return GetReward(player, state, Action());
        }
        return 0.0f;
    }

    private:
        float minResetHeight;
        float minPlayerToBallDist;
        float lastCloseness; // Track the last closeness value
    };


    class FlipResetEventReward : public RewardFunction {
    public:
        FlipResetEventReward(float minResetHeight = 300.0f)
            : minResetHeight(minResetHeight) {}
    
        virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
            float reward = 0.0f;
    
            // Check if the player has performed a flip reset
            if (player.carState.hasFlipped && !player.carState.isOnGround && player.phys.pos.z > minResetHeight) {
                // Additional checks can be added here, such as minimum height requirement and downward alignment
                reward = 1.0f; // Base reward for performing a flip reset
            }
    
            return reward;
        }
    
    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
        if (rewardName == "FlipResetEventReward") {
            return GetReward(player, state, Action());
        }
        return 0.0f;
    }

    private:
        float minResetHeight;
    };




    class ResetShotReward : public RewardFunction {
    public:
        ResetShotReward(float forceMultiplier = 1.0f, float speedMultiplier = 1.0f)
            : forceMultiplier(forceMultiplier), speedMultiplier(speedMultiplier) {}
    
        virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
            float reward = 0.0f;
    
            // Check if the player has used a reset and is not flipping
            if (player.carState.hasFlipped && !player.carState.isOnGround && !player.carState.hasJumped) {
                // Calculate the reward based on the force of the hit and the resulting speed of the ball
                // Infer hit force based on the change in player and ball velocities
                float playerVelChange = (player.phys.vel - prevPlayerVel).Length();
                float ballVelChange = (state.ball.vel - prevBallVel).Length();

                // Combine player and ball velocity changes to infer hit force
                // This is a simplification and might not accurately reflect the actual hit force
                float inferredHitForce = playerVelChange + ballVelChange;

                float ballSpeed = state.ball.vel.Length();
                reward = inferredHitForce * speedMultiplier + ballSpeed * forceMultiplier;
            }

            // Update previous velocities for the next calculation
            prevPlayerVel = player.phys.vel;
            prevBallVel = state.ball.vel;
    
            return reward;
        }
    
    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
        if (rewardName == "ResetShotReward") {
            return GetReward(player, state, Action());
        }
        return 0.0f;
    }

    private:
        float forceMultiplier;
        float speedMultiplier;
        Vec prevPlayerVel = Vec(0, 0, 0); // Initialize with zero vector
        Vec prevBallVel = Vec(0, 0, 0); // Initialize with zero vector
    };






    class AerialChallenge : public RewardFunction {
    public:
    AerialChallenge(float heightThreshold = 300.0f, float touchRewardBase = 10.0f, float heightMultiplier = 0.5f)
        : heightThreshold(heightThreshold), touchRewardBase(touchRewardBase), heightMultiplier(heightMultiplier) {}

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        float reward = 0.0f;
        Vec ballPos = state.ball.pos;
        Vec playerPos = player.phys.pos;

        // Check if the ball is high up in the air
        if (ballPos.z > heightThreshold) {
            // Calculate the distance to the ball
            float distance = (playerPos - ballPos).Length();
            float closestDistance = std::numeric_limits<float>::max();
            for (const auto& otherPlayer : state.players) {
                float otherDistance = (otherPlayer.phys.pos - ballPos).Length();
                if (otherDistance < closestDistance) {
                    closestDistance = otherDistance;
                }
            }

            // Reward for being the closest to the ball
            if (distance == closestDistance) {
                reward += 1.0f; // Base reward for being the closest
                reward += (ballPos.z - heightThreshold) * heightMultiplier; // Additional reward based on height
            }

            // Reward for touching the ball
            if (player.ballTouchedStep) {
                reward += touchRewardBase * (ballPos.z / heightThreshold); // Increase touch reward based on height
            }
        }

        return reward;
    }
    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "AerialChallenge") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
	}

    private:
    float heightThreshold;
    float touchRewardBase;
    float heightMultiplier;
    };


    class MultiTouchAerialVelocityChange : public RewardFunction {
    public:
    MultiTouchAerialVelocityChange(float aerial_w = 0.1f, int exp = 2, float min_aerial = 500.0f, float touch_w = 0.3f, float vel_w = 0.8f)
        : aerial_w(aerial_w), exp(exp), min_aerial(min_aerial), touch_w(touch_w), vel_w(vel_w), last_vel(Vec(0, 0, 0)) {}

    virtual void Reset(const GameState& initialState) override {
        last_vel = Vec(0, 0, 0);
        ball_count = 0;
        ball_touch_vel_change = 0;
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        constexpr float BALL_RADIUS = 92.75f;
        constexpr float CEILING_Z = 2048.0f; // Assuming a standard ceiling height
        float reward = 0;
        ball_touch_vel_change = 0;
        float reward_height = 0;
        float reward_vel = 0;

        if (player.ballTouchedStep && !player.carState.isOnGround) {
            if (state.ball.pos.z > min_aerial) {
                Vec vel_difference = last_vel - state.ball.vel;
                ball_touch_vel_change += vel_difference.Length() / 4600.0f;
                ball_count += 1;
                reward_height += pow((state.ball.pos.z * aerial_w), exp) / (CEILING_Z - BALL_RADIUS);
                reward += pow(ball_count, exp) * touch_w;
                reward_vel += ball_touch_vel_change * pow(ball_count, exp) * vel_w;

                if (ball_count >= 3 && player.ballTouchedStep && !player.carState.isOnGround) {
                    // Debug print statement
                    // std::cout << "Reward: " << reward << ", " << reward_vel << ", " << reward_height << ", Count: " << ball_count << std::endl;
                }
            } else {
                ball_touch_vel_change = 0;
                ball_count = 0;
                reward_height = 0;
                reward = 0;
                reward_vel = 0;
            }
        }

        // Add min boost if player.ball_touched and not player.on_ground and player.boost_amount > 0
        if (player.ballTouchedStep && !player.carState.isOnGround && player.boostFraction > 0) {
            reward += 1.0f; // Example boost reward, adjust as necessary
        }

        last_vel = state.ball.vel;

        return reward + reward_vel + reward_height;
    }
    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "MultiTouchAerialVelocityChange") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
	}

    private:
    float aerial_w;
    int exp;
    float min_aerial;
    float touch_w;
    float vel_w;
    Vec last_vel;
    int ball_count;
    float ball_touch_vel_change;
    };



    class DoubleTapReward : public RewardFunction {
    public:
    DoubleTapReward(float dtapHelperW = 1.0f, float maxDistanceWeight = 1.0f, float maxTimeWeight = 1.0f)
        : dtapHelperW(dtapHelperW), maxDistanceWeight(maxDistanceWeight), maxTimeWeight(maxTimeWeight) {}

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        constexpr float BACK_WALL_Y = 5120.0f; // Assuming a standard back wall Y position
        constexpr float BALL_RADIUS = 92.75f; // Assuming the ball radius is 92.75 units
        constexpr float MIN_T_POST_WALL_BOUNCE = 0.2f;

        if (player.ballTouchedStep && !player.carState.isOnGround) {
            Vec objective(state.ball.pos.x / 2, BACK_WALL_Y, 1200); // Objective position
            Vec posDiff = objective - state.ball.pos;
            posDiff.y *= 5; // Weight the y component more
            Vec normPosDiff = posDiff.Normalized();
            Vec vel = state.ball.vel;
            vel.y *= 5; // Weight the y component more
            Vec normVel = vel.Normalized();
            float dtapHelperRew = normPosDiff.Dot(normVel);
            float reward = dtapHelperW * dtapHelperRew;

            // Calculate the minimum distance based on the ball's bounce time and maximum time
            float maxTime = (vel.z - BALL_RADIUS + std::sqrt(vel.z * vel.z + 1300 * (state.ball.pos.z - BALL_RADIUS))) / 650;
            float bounceTime = (BACK_WALL_Y - BALL_RADIUS - state.ball.pos.y) / vel.y;
            float bounceYCoord = state.ball.pos.y + vel.y * bounceTime;

            if (maxTime < bounceTime + MIN_T_POST_WALL_BOUNCE || bounceTime < 0) {
                return reward; // No additional reward if conditions are not met
            } else {
                // Calculate the minimum distance
                float minDist = std::min({
                    dist(bounceTime + MIN_T_POST_WALL_BOUNCE, state.ball.pos, vel, player.phys.pos, player.phys.vel),
                    dist(maxTime, state.ball.pos, vel, player.phys.pos, player.phys.vel)
                });

                // Additional reward based on the minimum distance and maximum time
                reward += maxDistanceWeight * (1.0f - minDist / (state.ball.pos - player.phys.pos).Length());
                reward += maxTimeWeight * (1.0f - (maxTime - bounceTime - MIN_T_POST_WALL_BOUNCE) / maxTime);
            }

            return reward;
        }

        return 0.0f;
    }

    private:
    float dtapHelperW;
    float maxDistanceWeight;
    float maxTimeWeight;

    float dist(float t, const Vec& ballPos, const Vec& ballVel, const Vec& carPos, const Vec& carVel) {
        Vec ballPosAtT = ballPos + ballVel * t;
        Vec carPosAtT = carPos + carVel * t;
        Vec ballPosAtTAdjusted = ballPosAtT - Vec(0, 0, 325 * t * t);
        Vec carPosAtTAdjusted = carPosAtT - Vec(0, 0, 325 * t * t);
        return (ballPosAtTAdjusted - carPosAtTAdjusted).Length();
    }
    };


    class DistToBallReward : public RewardFunction {
    public:
    virtual void Reset(const GameState& initialState) override {
        // No specific reset logic needed for this reward function
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        Vec ballToPlayer = state.ball.pos - player.phys.pos;
        // Assuming Vec has x, y, and z members
        float lengthSquared = ballToPlayer.x * ballToPlayer.x + ballToPlayer.y * ballToPlayer.y + ballToPlayer.z * ballToPlayer.z;
        float distance = 1.0f / std::sqrt(lengthSquared);
        return distance * 100.0f;
    }
    };

    class AerialReward : public RewardFunction {
    public:
    AerialReward() : in_air_reward(0), last_touch_pos(std::nullopt), lost_jump(false), reward(0), dist_to_ball(DistToBallReward()), on_wall(false) {}

    virtual void Reset(const GameState& initialState) override {
        last_touch_pos = std::nullopt;
        lost_jump = false;
        reward = 0;
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        // Starting dribble reward
        if (130 < state.ball.pos.z && state.ball.pos.z < 180 && player.ballTouchedStep) {
            float ball_speed = state.ball.vel.Length();
            if (ball_speed < 2000) {
                reward += 2;
            }
        }

        if (state.ball.pos.z > 230) {
            if (player.ballTouchedStep) {
                // Aerial touch
                reward += 1;
                last_touch_pos = state.ball.pos;

                // For power flicks while dribbling
                if (220 < state.ball.pos.z && state.ball.pos.z < 400) {
                    float ball_speed = state.ball.vel.Length();
                    float ball_speed_reward = ball_speed / CommonValues::BALL_MAX_SPEED;
                    reward += ball_speed_reward;
                }

                // If no jump, reward ball power
                if (!player.carState.hasJumped) {
                    if (!lost_jump) {
                        lost_jump = true;
                    }

                    float ball_speed = state.ball.vel.Length();
                    float ball_speed_reward = ball_speed / (10 * CommonValues::BALL_MAX_SPEED);
                    reward += ball_speed_reward;
                }
            } else if (player.phys.pos.z > 300) {
                Vec ballToPlayer = state.ball.pos - player.phys.pos;
                float dist = ballToPlayer.Length() - CommonValues::BALL_RADIUS;
                float dist_reward = (dist * dist) / 600000.0f + 0.5f;
                if (dist_reward < 0.6f) {
                    float height_reward = state.ball.pos.z / CommonValues::CEILING_Z;
                    reward += height_reward;
                    float dist_to_ball_reward = dist_to_ball.GetReward(player, state, prevAction) * 10;
                    reward += dist_to_ball_reward;
                }
            }
            return reward;
        } else {
            reward = 0;
            if (lost_jump) {
                lost_jump = false;
            }
            return reward;
        }
    }

    private:
    int in_air_reward;
    std::optional<Vec> last_touch_pos;
    bool lost_jump;
    float reward;
    DistToBallReward dist_to_ball;
    bool on_wall;
    };

    class WallReward : public RewardFunction {
    public:
    WallReward() : dist_to_ball(DistToBallReward()), on_wall(false), lost_jump(false), reward(0) {}

    virtual void Reset(const GameState& initialState) override {
        on_wall = false;
        lost_jump = false;
        reward = 0;
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        // Reward for distance to ball + wall encouragement + after wall near the ball reward
        if (!player.carState.hasJumped) {
            if (!lost_jump) {
                lost_jump = true;
            }
        }

        float dist_to_ball_reward = dist_to_ball.GetReward(player, state, prevAction) * 0.04f;

        // Starting encouragement
        if (state.ball.pos.x < -4000 || state.ball.pos.x > 4000 ||
            (state.ball.pos.y > 5020 && state.ball.pos.x > -900 && state.ball.pos.x < 900) ||
            (state.ball.pos.y < -5020 && state.ball.pos.x > -900 && state.ball.pos.x < 900)) {
            reward += dist_to_ball_reward * 2;
        }

        if (player.phys.pos.x < -4000 || player.phys.pos.x > 4000 ||
            (player.phys.pos.y > 5020 && player.phys.pos.x > -900 && player.phys.pos.x < 900) ||
            (player.phys.pos.y < -5020 && player.phys.pos.x > -900 && player.phys.pos.x < 900)) {

            float addup = (player.phys.pos.z / CommonValues::CEILING_Z) * 2;
            dist_to_ball_reward *= (10 + addup);
            if (!on_wall) {
                on_wall = true;
            } else {
                dist_to_ball_reward *= 10;
            }
        }

        if (player.carState.hasFlipped && lost_jump) {
            if ((player.phys.pos.x < -4000 || player.phys.pos.x > 4000) && state.ball.pos.z > 500 && on_wall) {
                dist_to_ball_reward *= 20;
            }
        }

        reward += dist_to_ball_reward;

        if (!player.carState.hasJumped) {
            if (!lost_jump) {
                lost_jump = true;
            }

            float ball_speed = state.ball.vel.Length();
            float ball_speed_reward = ball_speed / CommonValues::BALL_MAX_SPEED;
            reward += ball_speed_reward / 10;
        }

        if (player.ballTouchedStep) {
            reward += 5;
        }

        if (player.carState.hasJumped) {
            lost_jump = false;
        }
        return reward;
    }

    virtual float GetFinalReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        return 0;
    }

    private:
    DistToBallReward dist_to_ball;
    bool on_wall;
    bool lost_jump;
    float reward;
    };

    class PositiveRollReward : public RewardFunction {
    public:
    PositiveRollReward(float height_threshold = 300.0f, float distance_threshold = 500.0f)
        : height_threshold(height_threshold), distance_threshold(distance_threshold) {}

    virtual void Reset(const GameState& initialState) override {
        // No specific reset logic needed for this reward function
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        float reward = 0.0f;
        if (player.phys.pos.z > height_threshold &&
            (player.phys.pos - state.ball.pos).Length() < distance_threshold &&
            prevAction.roll > 0.0f) {
            reward = 1.0f;
        }
        return reward;
    }

    private:
    float height_threshold;
    float distance_threshold;
    };






     class HeightTouchReward : public RewardFunction {
    public:
    HeightTouchReward(float min_height = 92.0f, float exp = 0.2f, float coop_dist = 0.0f)
        : min_height(min_height), exp(exp), cooperation_dist(coop_dist) {}

    virtual void Reset(const GameState& initialState) override {
        // No specific reset logic needed for this reward function
    }

    bool cooperation_detector(const PlayerData& player, const GameState& state) {
        for (const auto& p : state.players) {
            if (p.carId != player.carId && (player.phys.pos - p.phys.pos).Length() < cooperation_dist) {
                return true;
            }
        }
        return false;
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        float reward = 0.0f;
        if (player.ballTouchedStep) {
            if (state.ball.pos.z >= min_height) {
                if (!player.carState.isOnGround || cooperation_dist < 90.0f || !cooperation_detector(player, state)) {
                    if (player.carState.isOnGround) {
                        reward += std::clamp(5000.0f, 0.0001f, (state.ball.pos.z - 92.0f)) * std::pow(exp, 2);
                    } else {
                        reward += std::clamp(500.0f, 1.0f, std::pow(state.ball.pos.z, exp * 2));
                    }
                }
            } else if (!player.carState.isOnGround) {
                reward += 1.0f;
            }
        }
        return reward;
    }

    private:
    float min_height;
    float exp;
    float cooperation_dist;
    };






    /*3s has very different rewards: idea
- no dribble reward
- 2x air touch reward
- 2x strong touch reward
- 5x speed reward
- 4x shot/shot-pass reward
- added 3s pressure reward for having the ball near the opponents net*/
    




};
