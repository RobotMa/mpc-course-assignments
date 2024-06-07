import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {"FIG_SIZE": [8, 8], "OBSTACLES": True}


class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.2
        self.car_length = 2.5

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t_1 = x_t + v_t * dt * np.cos(psi_t)
        y_t_1 = y_t + v_t * dt * np.sin(psi_t)
        psi_t_1 = psi_t + v_t / self.car_length * dt * np.tan(steering)
        v_t_1 = v_t + pedal * dt - v_t / 25.0 * dt

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(0, self.horizon):
            state = self.plant_model(state, self.dt, u[2 * i], u[2 * i + 1])

            cost += abs(ref[0] - state[0]) ** 2 + abs(ref[1] - state[1]) ** 2
            cost += abs(ref[2] - state[2]) ** 2

            # try to keep the cost differentiable by removing the abs
            # cost += abs(ref[0] - state[0]) + 50 * abs(ref[1] - state[1])

            x_diff = self.x_obs - state[0]
            y_diff = self.y_obs - state[1]

            dist = np.sqrt(x_diff**2 + y_diff**2)

            # if the car is close to the obstacle, increase the cost
            if dist < np.sqrt(2):
                cost += 1 / (abs(self.x_obs - state[0]) ** 2) + 1 / (
                    abs(self.y_obs - state[1]) ** 2
                )
            else:
                cost += 1

        return cost


sim_run(options, ModelPredictiveControl)
