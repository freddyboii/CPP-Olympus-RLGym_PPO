// WeightedStateSetter.h
#pragma once

#include <vector>
#include <random>
#include <memory>
#include <string>

#include "StateSetter.h"
#include "FlickState.h"
#include "RandomState.h"
#include "KickoffState.h"
#include "ReplaySetter.h"
#include "AerialState.h"

namespace RLGSC {

class WeightedStateSetter : public StateSetter {
public:
    WeightedStateSetter(int gameMode) {
        // Change the weight of the kickoff state based on the gameMode value
        double kickoffWeight, flickWeight, randomWeight, aerialState;
        switch (gameMode) {
            case 1:
                kickoffWeight = 0.03;
                flickWeight = 0.17;
                aerialState = 0.00;
                randomWeight = 0.00;
                break;
            case 2:
                kickoffWeight = 0.03;
                flickWeight = 0.17;
                aerialState = 0.00;
                randomWeight = 0.00;
                break;
            case 3:
                kickoffWeight = 0.03;
                flickWeight = 0.17;
                aerialState = 0.00;
                randomWeight = 0.00;
                break;
            default:
                kickoffWeight = 0.03;
                flickWeight = 0.17;
                aerialState = 0.00;
                randomWeight = 0.00;
                break;
        }

        // Add state setters and their weights
        stateSetters.emplace_back(std::make_pair(std::make_unique<FlickState>(), flickWeight));
        stateSetters.emplace_back(std::make_pair(std::make_unique<RandomState>(true, true, true), randomWeight));
        stateSetters.emplace_back(std::make_pair(std::make_unique<KickoffState>(), kickoffWeight));
        stateSetters.emplace_back(std::make_pair(std::make_unique<AerialState>(), aerialState));
  

        // Change the string based on the gameMode value
/*         std::string replayFile;
        switch (gameMode) {
            case 1:
                replayFile = "SSL_1v1.bin";
                break;
            case 2:
                replayFile = "SSL_2v2.bin";
                break;
            case 3:
                replayFile = "SSL_3v3.bin";
                break;
            default:
                replayFile = "SSL_1v1.bin";
                break;
        } */

        stateSetters.emplace_back(std::make_pair(std::make_unique<ReplaySetter>(gameMode), 0.8));

        initializeWeights();
    }

    virtual GameState ResetState(Arena* arena) override {
        // Select a state setter based on the weights
        std::random_device rd;
        std::mt19937 gen(rd());
        std::discrete_distribution<int> distribution(weights.begin(), weights.end());
        int selectedIndex = distribution(gen);

        // Call the ResetState function of the selected state setter
        return stateSetters[selectedIndex].first->ResetState(arena);
    }

private:
    std::vector<std::pair<std::unique_ptr<StateSetter>, double>> stateSetters;
    std::vector<double> weights;

    void initializeWeights() {
        weights.clear();
        for (const auto& pair : stateSetters) {
            weights.push_back(pair.second);
        }
    }
};


}


