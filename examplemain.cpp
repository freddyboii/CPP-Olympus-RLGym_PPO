#include <RLGymPPO_CPP/Learner.h>

#include <RLGymSim_CPP/Utils/RewardFunctions/CommonRewards.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/AdvancedRewards.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/CombinedReward.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/CombinedLogger.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/ZeroMean.h>
#include <RLGymSim_CPP/Utils/TerminalConditions/NoTouchCondition.h>
#include <RLGymSim_CPP/Utils/TerminalConditions/GoalScoreCondition.h>
#include <RLGymSim_CPP/Utils/StateSetters/WeightedStateSetter.h>
#include <RLGymSim_CPP/Utils/ActionParsers/DiscreteAction.h>
#include <RLGymSim_CPP/Utils/OBSBuilders/AdvancedOBS.h>
#include <RLGymSim_CPP/Utils/OBSBuilders/AdvancedObsPadder.h>


#include <RLGymSim_CPP/Utils/BasicTypes/Action.h>
#include <random>


using namespace RLGPC; // RLGymPPO
using namespace RLGSC; // RLGymSim


void OnStep(GameInst* gameInst, const RLGSC::Gym::StepResult& stepResult, Report& gameMetrics) {

	auto& gameState = stepResult.state;
	for (auto& player : gameState.players) {

		float speed = player.phys.vel.Length();
		gameMetrics.AccumAvg("player_speed", speed);
		gameMetrics.AccumAvg("ball_touch_ratio", player.ballTouchedStep);
		gameMetrics.AccumAvg("in_air_ratio", !player.carState.isOnGround);
        float distToBall = (player.phys.pos - gameState.ball.pos).Length();
        gameMetrics.AccumAvg("dist_to_ball", distToBall);
        gameMetrics.AccumAvg("team_goals", gameState.scoreLine[(int)player.team]);
        gameMetrics.AccumAvg("match_shots", player.matchShots);
        gameMetrics.AccumAvg("match_saves", player.matchSaves);
        gameMetrics.AccumAvg("match_demos", player.matchDemos);
        gameMetrics.AccumAvg("match_assists", player.matchAssists);
        gameMetrics.AccumAvg("match_shot_passes", player.matchShotPasses);
        gameMetrics.AccumAvg("match_goals", player.matchGoals);
        gameMetrics.AccumAvg("boost_pickups", player.boostPickups);
        gameMetrics.AccumAvg("car_demoed", player.carState.isDemoed);
        gameMetrics.AccumAvg("boost_fraction", player.boostFraction);
		gameMetrics.AccumAvg("IsSupersonic", player.carState.isSupersonic);

        float ballSpeed = gameState.ball.vel.Length();
        gameMetrics.AccumAvg("ball_speed", ballSpeed);

		float playerHeight = player.phys.pos.z;
        gameMetrics.AccumAvg("player_height", playerHeight);

	    float ball_height = gameState.ball.pos.z;
        gameMetrics.AccumAvg("ball_height", ball_height);

		RLGSC::TouchVelChange touchVelChange;
		RLGSC::VelocityBallToGoalReward velocityBallToGoalReward;
		RLGSC::VelocityPlayerToBallReward velocityPlayerToBallReward;
		RLGSC::FaceBallReward faceBallReward;
		RLGSC::TouchBallReward touchBallReward;
		RLGSC::JumpTouchReward jumpTouchReward;
        RLGSC::GoalSpeedReward goalSpeedReward;
		
		
		float graphRewardValueTouchVelChange = touchVelChange.graphR("TouchVelChange", player, gameState);
		gameMetrics.AccumAvg("TouchVelChange", graphRewardValueTouchVelChange);
		
		float graphRewardValueVelocityBallToGoal = velocityBallToGoalReward.graphR("VelocityBallToGoalReward", player, gameState);
		gameMetrics.AccumAvg("VelocityBallToGoalReward", graphRewardValueVelocityBallToGoal);
		
		float graphRewardValueVelocityPlayerToBall = velocityPlayerToBallReward.graphR("VelocityPlayerToBallReward", player, gameState);
		gameMetrics.AccumAvg("VelocityPlayerToBallReward", graphRewardValueVelocityPlayerToBall);
		
		float graphRewardValueFaceBall = faceBallReward.graphR("FaceBallReward", player, gameState);
		gameMetrics.AccumAvg("FaceBallReward", graphRewardValueFaceBall);
		
		float graphRewardValueTouchBall = touchBallReward.graphR("TouchBallReward", player, gameState);
		gameMetrics.AccumAvg("TouchBallReward", graphRewardValueTouchBall);
		
		float graphRewardValueJumpTouch = jumpTouchReward.graphR("JumpTouchReward", player, gameState);
		gameMetrics.AccumAvg("JumpTouchReward", graphRewardValueJumpTouch);

		float graphRewardValueGoalSpeed = goalSpeedReward.graphR("GoalSpeedReward", player, gameState);

        gameMetrics.AccumAvg("GoalSpeedReward", graphRewardValueGoalSpeed);

		RLGSC::WallTouchReward wallTouchReward;
		float graphRewardValueWallTouch = wallTouchReward.graphR("WallTouchReward", player, gameState);
		gameMetrics.AccumAvg("WallTouchReward", graphRewardValueWallTouch);


		RLGSC::VelocityReward velocityReward;

        float graphRewardValueVelocityReward = velocityReward.graphR("VelocityReward", player, gameState);
        gameMetrics.AccumAvg("VelocityReward", graphRewardValueVelocityReward);


		RLGSC::AerialDistanceReward aerialDistanceReward;
        RLGSC::DribbleReward dribbleReward;
        RLGSC::SaveBoostReward saveBoostReward;
        RLGSC::AerialReward aerialReward;

        float graphAerialDistanceReward = aerialDistanceReward.graphR("AerialDistanceReward", player, gameState);
        gameMetrics.AccumAvg("AerialDistanceReward", graphAerialDistanceReward);

        float graphRewardValueDribble = dribbleReward.graphR("DribbleReward", player, gameState);
        gameMetrics.AccumAvg("DribbleReward", graphRewardValueDribble);

        float graphRewardValueSaveBoost = saveBoostReward.graphR("SaveBoostReward", player, gameState);
        gameMetrics.AccumAvg("SaveBoostReward", graphRewardValueSaveBoost);

        float graphaerialReward = aerialReward.graphR("AerialReward", player, gameState);
        gameMetrics.AccumAvg("AerialReward", graphaerialReward);

        RLGSC::BouncyAirDribbleReward bouncyAirDribbleReward;
        float graphRewardValueAirDribble = bouncyAirDribbleReward.graphR("BouncyAirDribbleReward", player, gameState);
        gameMetrics.AccumAvg("BouncyAirDribbleReward", graphRewardValueAirDribble);

		RLGSC::AerialChallenge aerialChallenge;
		RLGSC::FlipResetReward flipResetReward;
		RLGSC::MultiTouchAerialVelocityChange multitouchAerialVelocityChange;
		RLGSC::SpeedKickoffReward speedKickoffReward;
		RLGSC::ContinuousFlipResetReward continuousFlipResetReward;
		RLGSC::FlipResetEventReward flipResetEventReward;
		RLGSC::ResetShotReward resetShotReward;
		
		float graphRewardValueContinuousFlipReset = continuousFlipResetReward.graphR("ContinuousFlipResetReward", player, gameState);
		gameMetrics.AccumAvg("ContinuousFlipResetReward", graphRewardValueContinuousFlipReset);
		
		float graphRewardValueFlipResetEvent = flipResetEventReward.graphR("FlipResetEventReward", player, gameState);
		gameMetrics.AccumAvg("FlipResetEventReward", graphRewardValueFlipResetEvent);
		
		float graphRewardValueResetShot = resetShotReward.graphR("ResetShotReward", player, gameState);
		gameMetrics.AccumAvg("ResetShotReward", graphRewardValueResetShot);

		float graphRewardValueAerialChallenge = aerialChallenge.graphR("AerialChallenge", player, gameState);
		gameMetrics.AccumAvg("AerialChallenge", graphRewardValueAerialChallenge);

		float graphRewardValueFlipReset = flipResetReward.graphR("FlipResetReward", player, gameState);
		gameMetrics.AccumAvg("FlipResetReward", graphRewardValueFlipReset);

		float graphRewardValueMultitouchAerialVelocityChange = multitouchAerialVelocityChange.graphR("MultitouchAerialVelocityChange", player, gameState);
		gameMetrics.AccumAvg("MultitouchAerialVelocityChange", graphRewardValueMultitouchAerialVelocityChange);

		float graphRewardValueSpeedKickoff = speedKickoffReward.graphR("SpeedKickoffReward", player, gameState);
		gameMetrics.AccumAvg("SpeedKickoffReward", graphRewardValueSpeedKickoff);
	}
	
	
}

void calculateTotalTime(uint64_t totalTimesteps, double& totalTimeInYears, double& totalTimeInHours) {
    double totalTimeInSeconds = totalTimesteps / (120.0 / 8);
    totalTimeInHours = totalTimeInSeconds / 3600.0; // Convert seconds to hours
    totalTimeInYears = totalTimeInHours / (24.0 * 365.0); // Convert hours to years
}
        

void OnIteration(Learner* learner, Report& allMetrics) {
    // Assuming you have access to the total timesteps
    uint64_t totalTimesteps = learner->totalTimesteps;


    double totalTimeInYears, totalTimeInHours;
    calculateTotalTime(totalTimesteps, totalTimeInYears, totalTimeInHours);

	AvgTracker avgPlayerSpeed = {};
	AvgTracker avgBallTouchRatio = {};
	AvgTracker avgAirRatio = {};
	AvgTracker avgball_height = {};
	AvgTracker avgDistToBall = {};
    AvgTracker avgTeamGoals = {};
    AvgTracker avgMatchShots = {};
    AvgTracker avgMatchSaves = {};
    AvgTracker avgMatchDemos = {};
    AvgTracker avgMatchAssists = {};
    AvgTracker avgMatchShotPasses = {};
    AvgTracker avgMatchGoals = {};
    AvgTracker avgBoostPickups = {};
    AvgTracker avgCarDemoed = {};
    AvgTracker avgBoostFraction = {};
    AvgTracker avgBallSpeed = {};
	AvgTracker avgIsSupersonic = {};
	AvgTracker avgPlayerHeight = {};

    AvgTracker avgTouchVelChange;
    AvgTracker avgVelocityBallToGoal;
    AvgTracker avgVelocityPlayerToBall;
    AvgTracker avgFaceBall;
    AvgTracker avgTouchBall;
    AvgTracker avgJumpTouch;

	AvgTracker avgGoalSpeedReward = {};
	AvgTracker avgVelocityReward = {};
    AvgTracker avgWallTouchReward = {};

	AvgTracker avgAerialDistanceReward = {};
    AvgTracker avgDribbleReward = {};
    AvgTracker avgSaveBoostReward = {};
    AvgTracker avgAerialReward = {};
    AvgTracker avgBouncyAirDribbleReward = {};
    
    AvgTracker avgAerialChallenge = {};
    AvgTracker avgFlipResetReward = {};
    AvgTracker avgMultitouchAerialVelocityChange = {};
    AvgTracker avgSpeedKickoffReward = {};
    
    AvgTracker avgContinuousFlipResetReward = {};
    AvgTracker avgFlipResetEventReward = {};
    AvgTracker avgResetShotReward = {};
    

	
	// Get metrics for every gameInst
	auto allGameMetrics = learner->GetAllGameMetrics();
	for (auto& gameReport : allGameMetrics) {
		avgPlayerSpeed += gameReport.GetAvg("player_speed");
		avgBallTouchRatio += gameReport.GetAvg("ball_touch_ratio");
		avgAirRatio += gameReport.GetAvg("in_air_ratio");
		avgball_height += gameReport.GetAvg("ball_height");
		avgDistToBall += gameReport.GetAvg("dist_to_ball");
        avgTeamGoals += gameReport.GetAvg("team_goals");
        avgMatchShots += gameReport.GetAvg("match_shots");
        avgMatchSaves += gameReport.GetAvg("match_saves");
        avgMatchDemos += gameReport.GetAvg("match_demos");
        avgMatchAssists += gameReport.GetAvg("match_assists");
        avgCarDemoed += gameReport.GetAvg("car_demoed");
        avgBoostFraction += gameReport.GetAvg("boost_fraction");
        avgBallSpeed += gameReport.GetAvg("ball_speed");
		avgIsSupersonic += gameReport.GetAvg("IsSupersonic");
		avgPlayerHeight += gameReport.GetAvg("player_height");

        avgMatchShotPasses += gameReport.GetAvg("match_shot_passes");
        avgMatchGoals += gameReport.GetAvg("match_goals");
        avgBoostPickups += gameReport.GetAvg("boost_pickups");


        avgTouchVelChange += gameReport.GetAvg("TouchVelChange");
        avgVelocityBallToGoal += gameReport.GetAvg("VelocityBallToGoalReward");
        avgVelocityPlayerToBall += gameReport.GetAvg("VelocityPlayerToBallReward");
        avgFaceBall += gameReport.GetAvg("FaceBallReward");
        avgTouchBall += gameReport.GetAvg("TouchBallReward");
        avgJumpTouch += gameReport.GetAvg("JumpTouchReward");
        avgGoalSpeedReward += gameReport.GetAvg("GoalSpeedReward");
		avgVelocityReward += gameReport.GetAvg("VelocityReward");
		avgWallTouchReward += gameReport.GetAvg("WallTouchReward");

		avgAerialDistanceReward += gameReport.GetAvg("AerialDistanceReward");
        avgDribbleReward += gameReport.GetAvg("DribbleReward");
        avgSaveBoostReward += gameReport.GetAvg("SaveBoostReward");
        avgAerialReward += gameReport.GetAvg("AerialReward");

		avgAerialChallenge += gameReport.GetAvg("AerialChallenge");
		avgFlipResetReward += gameReport.GetAvg("FlipResetReward");
		avgMultitouchAerialVelocityChange += gameReport.GetAvg("MultitouchAerialVelocityChange");
		avgSpeedKickoffReward += gameReport.GetAvg("SpeedKickoffReward");
        
        avgBouncyAirDribbleReward += gameReport.GetAvg("BouncyAirDribbleReward");
        

		avgContinuousFlipResetReward += gameReport.GetAvg("ContinuousFlipResetReward");
		avgFlipResetEventReward += gameReport.GetAvg("FlipResetEventReward");
		avgResetShotReward += gameReport.GetAvg("ResetShotReward");
	}

	allMetrics["player_speed"] = avgPlayerSpeed.Get();
    allMetrics["ball_touch_ratio"] = avgBallTouchRatio.Get();
    allMetrics["in_air_ratio"] = avgAirRatio.Get();
    allMetrics["ball_height"] = avgball_height.Get();
    allMetrics["dist_to_ball"] = avgDistToBall.Get();
    allMetrics["team_goals"] = avgTeamGoals.Get();
    allMetrics["match_shots"] = avgMatchShots.Get();
    allMetrics["match_saves"] = avgMatchSaves.Get();
    allMetrics["match_demos"] = avgMatchDemos.Get();
    allMetrics["car_demoed"] = avgCarDemoed.Get();
    allMetrics["boost_fraction"] = avgBoostFraction.Get();
    allMetrics["ball_speed"] = avgBallSpeed.Get();
    allMetrics["IsSupersonic"] = avgIsSupersonic.Get();
    allMetrics["player_height"] = avgPlayerHeight.Get();

    allMetrics["TouchVelChange"] = avgTouchVelChange.Get();
    allMetrics["VelocityBallToGoalReward"] = avgVelocityBallToGoal.Get();
    allMetrics["VelocityPlayerToBallReward"] = avgVelocityPlayerToBall.Get();
    allMetrics["FaceBallReward"] = avgFaceBall.Get();
    allMetrics["TouchBallReward"] = avgTouchBall.Get();
    allMetrics["JumpTouchReward"] = avgJumpTouch.Get();
	allMetrics["GoalSpeedReward"] = avgGoalSpeedReward.Get();
	allMetrics["VelocityReward"] = avgVelocityReward.Get();
	
	allMetrics["WallTouchReward"] = avgWallTouchReward.Get();
	allMetrics["TotalTimeInYears"] = totalTimeInYears;
    allMetrics["TotalTimeInHours"] = totalTimeInHours;

	allMetrics["AerialDistanceReward"] = avgAerialDistanceReward.Get();
    allMetrics["DribbleReward"] = avgDribbleReward.Get();
    allMetrics["SaveBoostReward"] = avgSaveBoostReward.Get();
    allMetrics["AerialReward"] = avgAerialReward.Get();

    // Set new metrics
    allMetrics["match_assists"] = avgMatchAssists.Get();
    allMetrics["match_shot_passes"] = avgMatchShotPasses.Get();
    allMetrics["match_goals"] = avgMatchGoals.Get();
    allMetrics["boost_pickups"] = avgBoostPickups.Get();
    allMetrics["AerialChallenge"] = avgAerialChallenge.Get();
    allMetrics["FlipResetReward"] = avgFlipResetReward.Get();
    allMetrics["MultitouchAerialVelocityChange"] = avgMultitouchAerialVelocityChange.Get();
    allMetrics["SpeedKickoffReward"] = avgSpeedKickoffReward.Get();
    allMetrics["BouncyAirDribbleReward"] = avgBouncyAirDribbleReward.Get();
    allMetrics["ContinuousFlipResetReward"] = avgContinuousFlipResetReward.Get();
    allMetrics["FlipResetEventReward"] = avgFlipResetEventReward.Get();
    allMetrics["ResetShotReward"] = avgResetShotReward.Get();
}


int callCount = -1;
int threads = 19;
int GamesPerThread = 8;

EnvCreateResult EnvCreateFunc() {
    constexpr int TICK_SKIP = 12;
    constexpr float NO_TOUCH_TIMEOUT_SECS = 10.f;
    
    
    // Adjust these values to change the percentage of each team size
   float ones_pct = 1.0f;
   float twos_pct = 0.0f;

    //assign team based on call
    int cycleCount = (callCount / GamesPerThread) + 1;
    callCount++;

    // Calculate team size based on cycleCount
    int team_size;
    if (cycleCount <= (threads * ones_pct)) {
        team_size = 1;
    } else if (cycleCount <= (threads * (ones_pct + twos_pct))) {
        team_size = 2;
    } else {
        team_size = 3;
    }

    std::cout << "Team Size: " << team_size << std::endl;
	
        auto rewards = new CombinedReward{
    {
        // Player-related rewards
        { new TouchVelChange(), 0.7f },
        { new SaveBoostReward(), 0.1f },

        // Ball-related rewards
        { new VelocityBallToGoalReward(), 1.0f },
        { new TouchBallReward(), 0.5f },
        { new GoalSpeedReward(), 0.8f },
        { new DribbleReward(), 0.1f },

        // Movement-related rewards
        { new VelocityPlayerToBallReward(), 0.3f },
        { new VelocityReward(), 0.05f },

        // Mechanical-related rewards
		{ new JumpTouchReward(), 10.0f },
        { new AerialDistanceReward(10.0, 10.0, 0.0), 2.5f },

        // Event-related rewards
        { new EventReward({.teamGoal = 1.f, .concede = -1.f}), 50.f },
    },
    true // Assuming you want to own the reward functions
};

	std::vector<TerminalCondition*> terminalConditions = {
		new NoTouchCondition(NO_TOUCH_TIMEOUT_SECS * 120 / TICK_SKIP),
		new GoalScoreCondition()
	};

	auto obs = new AdvancedOBSPadder();
	auto actionParser = new DiscreteAction();
	auto stateSetter = new WeightedStateSetter(team_size);

	Match* match = new Match(
		rewards,
		terminalConditions,
		obs,
		actionParser,
		stateSetter,

		team_size, // Team size
		true // Spawn opponents
	);

	Gym* gym = new Gym(match, TICK_SKIP);
	return { match, gym };
}

// In examplemain.cpp
int main() {
	// Initialize RocketSim with collision meshes
	RocketSim::Init("./collision_meshes");

	// Make configuration for the learner
	LearnerConfig cfg = {};

	// Play around with these to see what the optimal is for your machine, more isn't always better
	cfg.numThreads = 19;
	cfg.numGamesPerThread = GamesPerThread;
	int tsPerItr = 1000 * 100;
	cfg.timestepsPerIteration = tsPerItr;
	cfg.ppo.batchSize = tsPerItr;
	cfg.ppo.miniBatchSize = 25 * 1000; 
	cfg.expBufferSize = tsPerItr * 3;
	
	cfg.ppo.epochs = 3; 
	cfg.ppo.entCoef = 0.01f;
	cfg.ppo.policyLR = 3e-4; //change too 1e-3 after some time
	cfg.ppo.criticLR = 3e-4;
	
	cfg.ppo.policyLayerSizes = {  2048, 1024, 1024, 1024 };
	cfg.ppo.criticLayerSizes = {  2048, 1024, 1024, 1024 };
	
	cfg.sendMetrics = true; 
	cfg.renderMode = false; 
	cfg.renderTimeScale = 1.0f;
	cfg.timestepLimit = 0;
	
    cfg.ppo.autocastLearn = false;
	 
	cfg.deterministic = false;

	cfg.metricsProjectName = "Olympus-v1.1"; // Project name for the python metrics receiver
    cfg.metricsGroupName = "Runs"; // Group name for the python metrics receiver
    cfg.metricsRunName = "v2.6"; // Run name for the 
	
	Learner learner = Learner(EnvCreateFunc, cfg);
	
	learner.stepCallback = OnStep;
	learner.iterationCallback = OnIteration;
	learner.Learn();

	return 0;
}