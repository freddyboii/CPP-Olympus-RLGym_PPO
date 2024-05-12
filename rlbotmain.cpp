#include <RLGymSim_CPP/Utils/ActionParsers/DiscreteAction.h>
#include <RLGymSim_CPP/Utils/OBSBuilders/AdvancedObsPadder.h>

#include <filesystem>
#include "RLBotClient.h"

using namespace RLGPC; // RLGymPPO
using namespace RLGSC; // RLGymSim


int main(int argc, char* argv[]) {
    std::filesystem::path exePath = std::filesystem::path(argv[0]);
    std::filesystem::path policyPath = exePath.parent_path() / "PPO_POLICY.lt";

    RLBotParams params = {
        .port = 42653,
        .obsBuilder = new AdvancedOBSPadder(),
        .actionParser = new DiscreteAction(),
        .policyPath = policyPath,
        .obsSize = 237,
        .policyLayerSizes = { 2048, 1024, 1024, 1024 },
        .tickSkip = 8
    };
    RLBotClient::Run(params);

    return 0;
}
