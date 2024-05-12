#include <random>
#include <RLGymPPO_CPP/Learner.h>

#include <RLGymSim_CPP/Utils/RewardFunctions/CommonRewards.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/CombinedReward.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/AdvancedRewards.h>
#include <RLGymSim_CPP/Utils/RewardFunctions/ZeroMean.h>
#include <RLGymSim_CPP/Utils/TerminalConditions/NoTouchCondition.h>
#include <RLGymSim_CPP/Utils/TerminalConditions/GoalScoreCondition.h>
#include <RLGymSim_CPP/Utils/StateSetters/WeightedStateSetter.h>
#include <RLGymSim_CPP/Utils/ActionParsers/DiscreteAction.h>
#include <RLGymSim_CPP/Utils/OBSBuilders/AdvancedOBS.h>
#include <RLGymSim_CPP/Utils/OBSBuilders/AdvancedObsPadder.h>


#include <RLGymSim_CPP/Utils/BasicTypes/Action.h>


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
        avgCarDemoed += gameReport.GetAvg("car_demoed");
        avgBoostFraction += gameReport.GetAvg("boost_fraction");
        avgBallSpeed += gameReport.GetAvg("ball_speed");
		avgIsSupersonic += gameReport.GetAvg("IsSupersonic");
		avgPlayerHeight += gameReport.GetAvg("player_height");


        avgTouchVelChange += gameReport.GetAvg("TouchVelChange");
        avgVelocityBallToGoal += gameReport.GetAvg("VelocityBallToGoalReward");
        avgVelocityPlayerToBall += gameReport.GetAvg("VelocityPlayerToBallReward");
        avgFaceBall += gameReport.GetAvg("FaceBallReward");
        avgTouchBall += gameReport.GetAvg("TouchBallReward");
        avgJumpTouch += gameReport.GetAvg("JumpTouchReward");
        avgGoalSpeedReward += gameReport.GetAvg("GoalSpeedReward");

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
	allMetrics["TotalTimeInYears"] = totalTimeInYears;
    allMetrics["TotalTimeInHours"] = totalTimeInHours;

}


int callCount = -1;
int threads = 15;
int GamesPerThread = 10;

EnvCreateResult EnvCreateFunc() {
    constexpr int TICK_SKIP = 8;
    constexpr float NO_TOUCH_TIMEOUT_SECS = 10.f;
    
    
    // Use a random number generator to decide the team size
std::random_device rd; // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
std::uniform_int_distribution<> distrib(1, 2);

int team_size = distrib(gen); // Generate a random team size between 1 and 3

	// Construct the replay file name based on the number of players per team
    std::string replayFileName = "ssl_" + std::to_string(team_size) + "v" + std::to_string(team_size) + ".npy";

	// Print out the current replay file name
    //std::cout << "Current replay file name: " << replayFileName << std::endl;

	std::cout << "Team Size: " << team_size << std::endl;
	
    
	auto rewards = new CombinedReward{
    {
        // Player-related rewards
        { new TouchVelChange(), 0.7f },
        { new JumpTouchReward(), 0.1f },
        { new SaveBoostReward(), 0.1f },
        //{ new EnergyReward(), 0.1f },

        // Ball-related rewards
        { new FaceBallReward(), 0.05f },
        { new VelocityBallToGoalReward(), 1.0f },
        { new TouchBallReward(), 0.5f },
        { new GoalSpeedReward(), 0.8f },
        { new DribbleReward(), 0.1f },

        // Movement-related rewards
        { new VelocityPlayerToBallReward(), 0.5f },
        { new VelocityReward(), 0.05f },

        // Mechanical-related rewards
        { new AirRollReward(), 0.1f },
        { new WallTouchReward(), 0.2f },
        //{ new WaveDashReward(), 0.3f },
        { new AerialReward(), 0.2f },
        //{ new CradleFlickReward(), 0.5f },

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
	cfg.numThreads = 15;
	cfg.numGamesPerThread = GamesPerThread;
	int tsPerItr = 1000 * 100;
	cfg.timestepsPerIteration = tsPerItr;
	cfg.ppo.batchSize = tsPerItr;
	cfg.ppo.miniBatchSize = 25 * 1000; 
	cfg.expBufferSize = tsPerItr * 2;
	
	cfg.ppo.epochs = 3; 
	cfg.ppo.entCoef = 0.01f;
	cfg.ppo.policyLR = 7e-4; //change too 1e-3 after some time
	cfg.ppo.criticLR = 7e-4;
	
	cfg.ppo.policyLayerSizes = {  2048, 1024, 1024, 1024 };
	cfg.ppo.criticLayerSizes = {  2048, 1024, 1024, 1024 };
	
	cfg.sendMetrics = false; 
	cfg.renderMode = true; 
	cfg.renderTimeScale = 1.0f; //lower = faster
	cfg.timestepLimit = 0;
	
    cfg.ppo.autocastLearn = false;
	 
	cfg.deterministic = false;

	cfg.metricsProjectName = "Olympus-v1.1"; // Project name for the python metrics receiver
    cfg.metricsGroupName = "Official"; // Group name for the python metrics receiver
    cfg.metricsRunName = "OF-runs"; // Run name for the 
	
	Learner learner = Learner(EnvCreateFunc, cfg);
	
	learner.stepCallback = OnStep;
	learner.iterationCallback = OnIteration;
	learner.Learn();

	return 0;
}
