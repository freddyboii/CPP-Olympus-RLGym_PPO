#pragma once
#include "RewardFunction.h"
#include <cmath> // For std::sqrt
#include "../Gamestates/GameState.h"
#include "../BasicTypes/Action.h"
#include <optional>
#include <chrono>
#include <vector>
#include <array>
#include <numeric>
#include <iostream> // For std::cout
#include <vector> // For std::vector

namespace RLGSC {
	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/misc_rewards.py
	class EventReward : public RewardFunction {
	public:
		struct ValSet {
			constexpr static int VAL_AMOUNT = 11;
			float vals[VAL_AMOUNT] = {};

			float& operator[](int index) { return vals[index]; }
		};
		ValSet weights;

		std::unordered_map<int, ValSet> lastRegisteredValues;

		struct WeightScales {
			float
				goal = 50,
				teamGoal = 25,
				concede = -50,
				assist = 30,

				touch = 1.0,
				shot = 10,
				shotPass = 10,
				save = 0,
				demo = 10,
				demoed = -14,
				boostPickup = 0.5;

			float& operator[](size_t index) { 
				// Make sure members line up
				static_assert(
					offsetof(WeightScales, boostPickup) - offsetof(WeightScales, goal) == 
					sizeof(float) * (ValSet::VAL_AMOUNT - 1)
					);
				return (&goal)[index]; 
			}
		};

		EventReward(WeightScales weightScales);

		static ValSet ExtractValues(const PlayerData& player, const GameState& state);

		virtual void Reset(const GameState& state);
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction);
	};

	
	class VelocityReward : public RewardFunction {
    public:
    bool isNegative;
    VelocityReward(bool isNegative = false) : isNegative(isNegative) {}
    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        return player.phys.vel.Length() / CommonValues::CAR_MAX_SPEED * (1 - 2 * isNegative);
    }
    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
        if (rewardName == "VelocityReward") {
            return GetReward(player, state, Action());
        }
        return 0.0f; 
    }
    };
	
	
	class TouchVelChange : public RewardFunction {
	public:
		Vec last_vel;
	
		TouchVelChange() : last_vel(Vec(0, 0, 0)) {}
	
		virtual void Reset(const GameState& initialState) override {
			last_vel = Vec(0, 0, 0);
		}
	
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;
			if (player.ballTouchedStep) {
				float vel_difference = std::abs((last_vel - state.ball.vel).Length());
				reward = vel_difference / 4600.0f;
			}
	
			last_vel = state.ball.vel;
	
			return reward;
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "TouchVelChange") {
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};
	
    
	
	class SaveBoostReward : public RewardFunction {
	public:
		float exponent;
		SaveBoostReward(float exponent = 0.5f) : exponent(exponent) {}
	
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) {
			return RS_CLAMP(powf(player.boostFraction, exponent), 0, 1);
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) {
			if (rewardName == "SaveBoostReward") {
				// Since there's no prevAction parameter in graphR, we use a default Action object
				// This might not be ideal in all cases, so adjust accordingly based on your needs
				return GetReward(player, state, Action());
			}
			return 0.0f;
		}
	};

	
	class VelocityBallToGoalReward : public RewardFunction {
	public:
		bool ownGoal = false;
		VelocityBallToGoalReward(bool ownGoal = false) : ownGoal(ownGoal) {}
	
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;
	
			Vec targetPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			
			Vec ballDirToGoal = (targetPos - state.ball.pos).Normalized();
			return ballDirToGoal.Dot(state.ball.vel / CommonValues::BALL_MAX_SPEED);
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "VelocityBallToGoalReward") {
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};
	

	
	
	class VelocityPlayerToBallReward : public RewardFunction {
	public:
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			Vec dirToBall = (state.ball.pos - player.phys.pos).Normalized();
			Vec normVel = player.phys.vel / CommonValues::CAR_MAX_SPEED;
			return dirToBall.Dot(normVel) * 10;
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "VelocityPlayerToBallReward") {
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};
	
	
	class FaceBallReward : public RewardFunction {
	public:
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			Vec dirToBall = (state.ball.pos - player.phys.pos).Normalized();
			return player.carState.rotMat.forward.Dot(dirToBall);
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "FaceBallReward") {
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};
	
	
	class TouchBallReward : public RewardFunction {
	public:
		float aerialWeight;
		TouchBallReward(float aerialWeight = 0.2) : aerialWeight(aerialWeight) {}
	
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			using namespace CommonValues;
	
			if (player.ballTouchedStep) {
				return powf((state.ball.pos.z + BALL_RADIUS) / (BALL_RADIUS * 2), aerialWeight);
			} else {
				return 0;
			}
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "TouchBallReward") {
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};
	
	
	class WallTouchReward : public RewardFunction {
	public:
		float min_height;
		float max_height;
		float range;
		bool flatten_wall_height;
		float cached_reward;
	
		WallTouchReward(float min_height = 500.0f) : min_height(min_height), max_height(2044.0f - 92.75f), range(max_height - min_height), flatten_wall_height(true), cached_reward(0.0f) {}
	
		virtual void Reset(const GameState& initialState) override {}
	
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			if (cached_reward != 0.0f) { // If the reward is already calculated, return the cached value
				return cached_reward;
			}
	
			bool isOnGround = player.carState.isOnGround;
			float ballZ = state.ball.pos.z;
			float coef_1 = flatten_wall_height ? 0.5f : 0.0f;
			float coef_2 = flatten_wall_height ? 1.5f : 1.0f;
	
			if (isOnGround && ballZ > min_height && player.ballTouchedStep) {
				float reward = (coef_1 + ballZ - min_height) / (coef_2 * range) + (player.boostFraction / 4.0f);
				cached_reward = reward; // Cache the calculated reward
				return reward * (1000*100);
			}
	
			return 0.0f;
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "WallTouchReward") {
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};
	
	
	class JumpTouchReward : public RewardFunction {
    public:
    JumpTouchReward(float min_height = CommonValues::BALL_RADIUS, float exp = 1.0f)
        : min_height(min_height), exp(exp), div(std::pow(CommonValues::CEILING_Z / 2 - CommonValues::BALL_RADIUS, exp)) {}

    virtual void Reset(const GameState& initialState) {
        ticks_until_next_reward = 0;
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) {
        if (player.ballTouchedStep && player.carState.isOnGround && state.ball.pos[2] >= min_height && ticks_until_next_reward <= 0) {
            ticks_until_next_reward = 149;
            float reward = std::pow(std::min(state.ball.pos[2], CommonValues::CEILING_Z / 2) - CommonValues::BALL_RADIUS, exp) / div;
            if (reward > 0.05)
                return reward * 100;
        }
        ticks_until_next_reward -= 1;
        return 0.0f;
    }

    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) {
        if (rewardName == "JumpTouchReward") {
            return GetReward(player, state, Action());
        }
        return 0.0f;
    }

private:
    float min_height;
    float exp;
    float div;
    int ticks_until_next_reward = 0;
};
	



    class KickoffReward : public RewardFunction {
	private:
		VelocityPlayerToBallReward vel_dir_reward;
	
	public:
		// Constructor to initialize vel_dir_reward
		KickoffReward() : vel_dir_reward() {}
	
		virtual void Reset(const GameState& initialState) override {
			vel_dir_reward.Reset(initialState);
		}
	
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;
			if (state.ball.pos.x == 0 && state.ball.pos.y == 0) {
				reward += vel_dir_reward.GetReward(player, state, prevAction);
			}
			return reward;
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "KickoffReward") {
				// Corrected to use the passed parameters
				return GetReward(player, state, Action());
			}
			return 0.0f; 
		}
	};


   
	
    class AerialDistanceReward : public RewardFunction {
    public:
    AerialDistanceReward() : height_scale(0.0f), distance_scale(0.0f), ang_vel_w(0.0f) {}
    AerialDistanceReward(float height_scale, float distance_scale, float ang_vel_w)
        : height_scale(height_scale), distance_scale(distance_scale), ang_vel_w(ang_vel_w) {}

    void Reset(const GameState& initialState) override {
        currentCar = nullptr;
        prevState = initialState;
        ballDistance = 0.0f;
        carDistance = 0.0f;
        angVelAccumulated = 0.0f;
    }

    class Vec {
    public:
        float x, y, z;

        Vec(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

        void Normalize() {
            float length = std::sqrt(x * x + y * y + z * z);
            if (length > 0.0001f) { // A small threshold to avoid division by zero
                x /= length;
                y /= length;
                z /= length;
            }
        }

        float Dot(const Vec& other) const {
            return x * other.x + y * other.y + z * other.z;
        }
    };

    float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        float rew = 0.0f;
        bool isCurrent = currentCar != nullptr && currentCar->carId == player.carId;

        // Test if player is on the ground
        if (player.phys.pos[2] < CommonValues::RAMP_HEIGHT) {
            if (isCurrent) {
                isCurrent = false;
                currentCar = nullptr;
            }
        }
        // First non ground touch detection
        else if (player.ballTouchedStep && !isCurrent) {
            isCurrent = true;
            ballDistance = 0.0f;
            carDistance = 0.0f;
            angVelAccumulated = 0.0f;
            rew = height_scale * std::max(player.phys.pos[2] + state.ball.pos[2] - 2 * CommonValues::RAMP_HEIGHT, 0.0f);
        }
        // Still off the ground after a touch, add distance and reward for more touches
        else if (isCurrent && !player.carState.isOnGround) {
            Vec dirToBall = Vec(state.ball.pos[0] - player.phys.pos[0], state.ball.pos[1] - player.phys.pos[1], state.ball.pos[2] - player.phys.pos[2]);
            dirToBall.Normalize();

            Vec playerVel = Vec(player.phys.vel[0], player.phys.vel[1], player.phys.vel[2]);
            float velTowardsBall = playerVel.Dot(dirToBall);

            // Add a reward based on the player's velocity towards the ball
            rew += velTowardsBall * distance_scale;

            // Check if the player is moving towards its own net
            bool movingTowardsOwnNet = (static_cast<int>(player.team) == static_cast<int>(CommonValues::BLUE_TEAM) && dirToBall.y < 0) ||
                                 (static_cast<int>(player.team) == static_cast<int>(CommonValues::ORANGE_TEAM) && dirToBall.y > 0);
            if (movingTowardsOwnNet) {
            // Reduce or adjust the reward to discourage air dribbling into own net
            rew *= 0.5f; // Example: reduce the reward by half
            }

            // Cash out on touches
            if (player.ballTouchedStep) {
                rew = distance_scale * (carDistance + ballDistance) + angVelAccumulated;
                carDistance = 0.0f;
                ballDistance = 0.0f;
                angVelAccumulated = 0.0f;
            }
        }

        if (isCurrent) {
            currentCar = &player; // Update to get latest physics info
        }

        prevState = state;

        return rew / (2 * CommonValues::BACK_WALL_Y) * 100;
    }




	float GetBallDistance() const { return ballDistance; }
    float GetCarDistance() const { return carDistance; }
    float GetAngVelAccumulated() const { return angVelAccumulated; }

    virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
		if (rewardName == "AerialDistanceReward") {
			return GetReward(player, state, Action());
		}
		return 0.0f; 
	}


    private:
    float height_scale;
    float distance_scale;
    float ang_vel_w;
    const PlayerData* currentCar = nullptr;
    GameState prevState;
    float ballDistance = 0.0f;
    float carDistance = 0.0f;
    float angVelAccumulated = 0.0f;
    };

	



	
	// Assuming necessary includes and using directives are already provided
	
	class AirRollReward : public RewardFunction {
	public:
		// Constructor
		AirRollReward(float air_roll_w = 1.0f) : air_roll_w(air_roll_w) {}
	
		// Override the GetReward method
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;
	
			// Check if the player car is within 1000 units of the ball
			float car_to_ball_distance = (state.ball.pos - player.phys.pos).Length();
			if (car_to_ball_distance < 300.0f) {
				if (!player.carState.isOnGround && player.phys.pos.z > 3 * CommonValues::BALL_RADIUS &&
					!player.carState.hasFlipped && player.carState.rotMat.forward.z > 0) {
					Vec car_to_ball = state.ball.pos - player.phys.pos;
					float dot_product = player.carState.rotMat.forward.Dot(car_to_ball);
					if (dot_product > 0) {
						reward += prevAction.roll * air_roll_w;
					}
				}
			}
	
			return reward;
		}
	
		// Override the graphR method
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "AirRollReward") {
				Action prevAction; // Assuming Action is a default-constructible class
				return GetReward(player, state, prevAction);
			}
			return 0.0f;
		}
	
	private:
		float air_roll_w; // Weight for the air roll reward
	};

    class SuccessfulShots : public RewardFunction {
	public:
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			if (player.matchShots > 0) {
				return static_cast<float>(player.matchGoals) / (player.matchShots) * 1000;
			}
			return 0.0f;
		}
	
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "SuccessfulShots") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
		}
	};

	
	class BumpTeammatePunishment : public RewardFunction {
	public:
		// Constructor
		BumpTeammatePunishment(float punishmentFactor = 1.0f) : punishmentFactor(punishmentFactor) {}
	
		// Override the GetReward method
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;
	
			// Check if the player has bumped someone
			if (player.matchBumps > 0) {
				// Iterate through all players to find if any of them are teammates
				for (const auto& otherPlayer : state.players) {
					if (otherPlayer.carId != player.carId && otherPlayer.team == player.team) {
						// Apply a negative reward for bumping a teammate
						reward -= punishmentFactor;
						break; // No need to check other players
					}
				}
			}
	
			return reward;
		}
	
		
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "BumpTeammatePunishment") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
		}
	
	private:
		float punishmentFactor; // Factor to multiply the punishment by
	};
	
	
	
	class GotBumpedPunishment : public RewardFunction {
	public:
		// Constructor
		GotBumpedPunishment(float punishmentFactor = 1.0f, float teammatePunishmentFactor = 2.0f)
			: punishmentFactor(punishmentFactor), teammatePunishmentFactor(teammatePunishmentFactor) {}
	
		// Override the GetReward method
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;
	
			// Check if the player has been bumped
			if (player.matchBumps > 0) {
				// Iterate through all players to find if any of them bumped the player
				for (const auto& otherPlayer : state.players) {
					if (otherPlayer.carId != player.carId && otherPlayer.matchBumps > 0) {
						// Check if the bump came from a teammate
						if (otherPlayer.team == player.team) {
							// Apply a double negative reward for being bumped by a teammate
							reward -= teammatePunishmentFactor;
						} else {
							// Apply a negative reward for being bumped by an opponent
							reward -= punishmentFactor;
						}
						break; // No need to check other players
					}
				}
			}
	
			return reward;
		}
	
		// Override the graphR method if needed
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "GotBumpedPunishment") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
		}
	
	private:
		float punishmentFactor; // Factor to multiply the punishment by for opponents
		float teammatePunishmentFactor; // Factor to multiply the punishment by for teammates
	};
	
	
	class PunishDistGoalScore : public RewardFunction {
	public:
		// Constructor
		PunishDistGoalScore(float punishmentFactor = 0.01f) : punishmentFactor(punishmentFactor) {}
	
		// Override the GetReward method
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;
	
			// Check if the player's team has been scored on
			if (state.scoreLine[(int)player.team] > 0) {
				// Calculate the distance from the player to the goal
				Vec goalPosition = (player.team == Team::BLUE) ? CommonValues::BLUE_GOAL_BACK : CommonValues::ORANGE_GOAL_BACK;
				float distanceToGoal = std::sqrt(std::pow(player.phys.pos.x - goalPosition.x, 2) + std::pow(player.phys.pos.y - goalPosition.y, 2));
	
				// Apply a punishment based on the distance to the goal
				reward -= punishmentFactor * distanceToGoal;
			}
	
			return reward;
		}
	
		// Override the graphR method if needed
		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "PunishDistGoalScore") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
		}
	
	private:
		float punishmentFactor; // Factor to multiply the punishment by
	};
	

	class GoalSpeedReward : public RewardFunction {
	public:
		// Constructor
		GoalSpeedReward(float rewardFactor = 0.1f) : rewardFactor(rewardFactor) {}

		// Override the GetReward method
		virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
			float reward = 0.0f;

			// Check if a goal has been scored
			if (state.scoreLine[(int)player.team] > 0) {
				// Calculate the speed of the ball at the moment of scoring
				float ballSpeed = state.ball.vel.Length();

				// Reward based on the ball's speed
				reward += ballSpeed * rewardFactor;
			}

			return reward;
		}

		virtual float graphR(const std::string& rewardName, const PlayerData& player, const GameState& state) override {
			if (rewardName == "GoalSpeedReward") {
				return GetReward(player, state, Action());
			}
			return 0.0f;
		}

	private:
		float rewardFactor;
	};


    class WaveDashReward : public RewardFunction {
    public:
        WaveDashReward() = default;

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


	class SpeedKickoffReward : public RewardFunction {
    public:
    SpeedKickoffReward(float reward_value = 1.0f) : reward_value(reward_value) {}

    virtual void Reset(const GameState& initialState) override {
        // No specific reset logic needed for this reward function
    }

    virtual float GetReward(const PlayerData& player, const GameState& state, const Action& prevAction) override {
        // Determine if the game is in a kickoff state
        bool is_kickoff = state.ball.vel.Length() < 100.0f &&
                          std::abs(state.ball.pos.x) < CommonValues::BALL_RADIUS &&
                          std::abs(state.ball.pos.y) < CommonValues::BALL_RADIUS;

        if (is_kickoff) {
            // Find the closest player to the ball
            float closest_dist = std::numeric_limits<float>::max();
            PlayerData closest_player;
            for (const auto& other_player : state.players) {
                float dist = (other_player.phys.pos - state.ball.pos).Length();
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_player = other_player;
                }
            }

            // Check if the current player is the closest
            if (closest_player.carId == player.carId) {
                // Return the reward value for the closest player during kickoff
                return reward_value;
            }
        }

        // Default reward if conditions are not met
        return 0.0f;
    }

    private:
    float reward_value;
    };



};


