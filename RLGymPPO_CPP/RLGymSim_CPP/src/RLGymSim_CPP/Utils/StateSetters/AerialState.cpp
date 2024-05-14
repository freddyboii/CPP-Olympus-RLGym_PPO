// AerialState.cpp
#include "AerialState.h"
#include "../../Math.h"
#include "../CommonValues.h"

constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;

RLGSC::GameState RLGSC::AerialState::ResetState(Arena* arena) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> rand_x_dist(-CommonValues::SIDE_WALL_X + CommonValues::BALL_RADIUS, CommonValues::SIDE_WALL_X - CommonValues::BALL_RADIUS);
    std::uniform_real_distribution<float> rand_y_dist(-CommonValues::BACK_WALL_Y + 1300, CommonValues::BACK_WALL_Y - 1300);
    std::uniform_real_distribution<float> rand_z_dist(100.0f, m_rand_z_max);
    std::uniform_real_distribution<float> rand_speed_dist(m_speed_min, m_speed_max);

    Car* car_attack = nullptr;
    Car* car_defend = nullptr;
    for (Car* car : arena->_cars) {
        if (car->team == Team::BLUE) {
            car_attack = car;
        } else if (car->team == Team::ORANGE) {
            car_defend = car;
        }
    }

    int orange_fix = 1;
    if (::Math::RandFloat() < 0.5) {
        std::swap(car_attack, car_defend);
        orange_fix = -1;
    }

    float rand_x = rand_x_dist(gen);
    float rand_y = rand_y_dist(gen);
    float rand_z = rand_z_dist(gen);
    Vec desired_car_pos(rand_x, rand_y, rand_z);
    float desired_pitch = (90 + orange_fix * ::Math::RandFloat(-20, -5)) * DEG_TO_RAD;
    float desired_yaw = 90 * DEG_TO_RAD;
    float desired_roll = 90 * (rand_x < 0 ? -1 : 1) * DEG_TO_RAD;
    Angle desired_rotation(desired_pitch, desired_yaw, desired_roll);

    CarState cs_attack = {};
    cs_attack.pos = desired_car_pos;
    cs_attack.rotMat = desired_rotation.ToRotMat();
    cs_attack.boost = ::Math::RandFloat(0.3f, 1.0f);
    cs_attack.vel = Vec(0, orange_fix * 200 * (rand_x < 0 ? -1 : 1), rand_speed_dist(gen));
    cs_attack.angVel = Vec(0, 0, 0);
    car_attack->SetState(cs_attack);

    // Spawn the ball in front of the car_attack
    BallState bs = {};
    if (rand_x < 0) {
        bs.pos = Vec(rand_x - CommonValues::BALL_RADIUS, rand_y + orange_fix * ::Math::RandFloat(20, 60), rand_z + ::Math::RandFloat(150, 200));
    } else {
        bs.pos = Vec(rand_x + CommonValues::BALL_RADIUS, rand_y + orange_fix * ::Math::RandFloat(20, 60), rand_z + ::Math::RandFloat(150, 200));
    }
    bs.vel = Vec(0, orange_fix * 200, rand_speed_dist(gen) - 150);
    bs.angVel = Vec(0, 0, 0);
    arena->ball->SetState(bs);

    // Loop over every car in the game, skipping the attack car since we already set it
    for (Car* car : arena->_cars) {
        if (car == car_attack) {
            continue;
        }

        CarState cs = {};
        if (car == car_defend) {
            cs.pos = Vec(::Math::RandFloat(-1600, 1600), orange_fix * ::Math::RandFloat(3800, 5000), 17);
            cs.rotMat = Angle(0, ::Math::RandFloat(-M_PI, M_PI), 0).ToRotMat();
            cs.boost = 1.0f;
        } else {
            cs.pos = Vec(::Math::RandFloat(-1472, 1472), ::Math::RandFloat(-1984, 1984), 17);
            cs.rotMat = Angle(0, ::Math::RandFloat(-M_PI, M_PI), 0).ToRotMat();
            cs.boost = 1.0f;
        }
        car->SetState(cs);
    }

    return GameState(arena);
}
