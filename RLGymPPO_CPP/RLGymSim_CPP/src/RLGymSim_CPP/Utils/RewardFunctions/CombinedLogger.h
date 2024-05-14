#pragma once
#include "RewardFunction.h"
#include <typeinfo> // For typeid
#include <iostream>

namespace RLGSC {
	class LoggerCombinedReward : public RewardFunction {
	public:
		std::vector<RewardFunction*> rewardFuncs;
		std::vector<float> rewardWeights;
		bool ownsFuncs;

		LoggerCombinedReward(std::vector<RewardFunction*> rewardFuncs, std::vector<float> rewardWeights, bool ownsFuncs = false) :
			rewardFuncs(rewardFuncs), rewardWeights(rewardWeights), ownsFuncs(ownsFuncs) {
			assert(rewardFuncs.size() == rewardWeights.size());
			printRewardFunctionNames();
		}

		LoggerCombinedReward(std::vector<std::pair<RewardFunction*, float>> funcsWithWeights, bool ownsFuncs = false) :
			ownsFuncs(ownsFuncs) {
			for (auto& pair : funcsWithWeights) {
				rewardFuncs.push_back(pair.first);
				rewardWeights.push_back(pair.second);
			}
			printRewardFunctionNames();
		}

	protected:
		virtual void Reset(const GameState& initialState) {
			for (auto func : rewardFuncs)
				func->Reset(initialState);
		}

		virtual void PreStep(const GameState& state) {
			for (auto func : rewardFuncs)
				func->PreStep(state);
		}

		virtual std::vector<float> GetAllRewards(const GameState& state, const ActionSet& prevAction, bool final) {
			std::vector<float> allRewards(state.players.size());

			for (int i = 0; i < rewardFuncs.size(); i++) {
				auto rewards = rewardFuncs[i]->GetAllRewards(state, prevAction, final);
				for (int j = 0; j < rewards.size(); j++)
					allRewards[j] += rewards[j] * rewardWeights[i];
			}

			return allRewards;
		}

		virtual ~LoggerCombinedReward() {
			if (ownsFuncs)
				for (auto func : rewardFuncs)
					delete func;
		}

	private:
		void printRewardFunctionNames() {
			for (auto func : rewardFuncs) {
				std::cout << typeid(*func).name() << std::endl;
			}
		}
	};
}