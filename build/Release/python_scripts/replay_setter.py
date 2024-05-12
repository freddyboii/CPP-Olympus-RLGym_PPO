import numpy as np
from typing import List
import random

class StateWrapper:
    def __init__(self):
        self.cars = [None] * 2
        self.ball = None

def get_state_data(filename):
    states = np.load(filename)
    random_state_index = random.randint(0, len(states) - 1)  # Choose a random state index
    state = states[random_state_index]  # Get the randomly selected state

    ball_position = state[:3].tolist()
    ball_linear_velocity = state[3:6].tolist()
    ball_angular_velocity = state[6:9].tolist()

    car_states_data = []
    num_cars = (len(state) - 9) // 13
    for i in range(num_cars):
        car_state = state[9 + i * 13:9 + (i + 1) * 13].tolist()
        car_states_data.append(car_state)

    return ball_position, ball_linear_velocity, ball_angular_velocity, car_states_data
