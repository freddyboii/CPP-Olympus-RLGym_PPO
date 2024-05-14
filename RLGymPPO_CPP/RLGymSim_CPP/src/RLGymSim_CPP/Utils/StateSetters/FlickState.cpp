// FlickState.cpp 
#include "FlickState.h"
#include "../../Math.h"

constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;

RLGSC::GameState RLGSC::FlickState::ResetState(Arena* arena) {
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

    int x_choice = ::Math::RandFloat() < 0.5 ? -1 : 1;

    float rand_x = x_choice * ::Math::RandFloat(0, 3000);
    float rand_y = ::Math::RandFloat(-2000, 2000);
    float rand_z = 19;
    float rand_x_vel = ::Math::RandFloat(0, 250);
    float rand_y_vel = ::Math::RandFloat(0, 2000);
    Vec desired_car_pos(rand_x, rand_y, rand_z);
    float desired_yaw = (orange_fix * 90 + x_choice * orange_fix * ::Math::RandFloat(5, 15)) * DEG_TO_RAD;
    float desired_pitch = 0;
    float desired_roll = 0;
    Angle desired_rotation(desired_yaw, desired_pitch,  desired_roll);

    CarState cs_attack = {};
    cs_attack.pos = desired_car_pos;
    cs_attack.rotMat = Angle(desired_yaw, desired_pitch,  desired_roll).ToRotMat();
    cs_attack.boost = 0.33f;
    cs_attack.vel = Vec(rand_x_vel * x_choice, rand_y_vel * orange_fix, 0);
    cs_attack.angVel = Vec(0, 0, 0);
    car_attack->SetState(cs_attack);

    // Put ball on top of car, slight random perturbations
    BallState bs = {};
    bs.pos = Vec(
        desired_car_pos.x + ::Math::RandFloat(-5, 5),
        desired_car_pos.y + ::Math::RandFloat(-5, 5) + orange_fix * 10,
        150 + ::Math::RandFloat(-10, 20)
    );
    bs.vel = cs_attack.vel;
    bs.angVel = Vec(::Math::RandFloat(-2, 2), ::Math::RandFloat(-2, 2), ::Math::RandFloat(-2, 2));
    arena->ball->SetState(bs);

    // Loop over every car in the game, skipping the attack car since we already set it
    for (Car* car : arena->_cars) {
        if (car == car_attack) {
            continue;
        }

        CarState cs = {};
        if (car == car_defend) {
            cs.pos = Vec(::Math::RandFloat(-1600, 1600), orange_fix * ::Math::RandFloat(3800, 5000), 17);
            cs.rotMat = Angle(::Math::RandFloat(-M_PI, M_PI), 0, 0).ToRotMat();
            cs.boost = 0.33f;
        } else {
            cs.pos = Vec(::Math::RandFloat(-1472, 1472), ::Math::RandFloat(-1984, 1984), 17);
            cs.rotMat = Angle(::Math::RandFloat(-M_PI, M_PI), 0, 0).ToRotMat();
            cs.boost = 0.33f;
}
car->SetState(cs);
}

return GameState(arena);

}