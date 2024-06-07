import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {"FIG_SIZE": [8, 8], "OBSTACLES": False}


class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2
        self.car_length = 2.5

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3.14 / 2]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t_1 = x_t + v_t * np.cos(psi_t) * dt
        y_t_1 = y_t + v_t * np.sin(psi_t) * dt
        psi_t_1 = psi_t + v_t * np.tan(steering) / self.car_length * dt
        v_t_1 = v_t + pedal * dt - v_t / 25.0  # last term is air resistance

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(0, self.horizon):
            state = self.plant_model(state, self.dt, u[2 * i], u[2 * i + 1])

            # position cost 1
            # cost += 4 * abs(ref[0] - state[0]) ** 2 + 4 * abs(ref[1] - state[1]) ** 2

            # position cost 2
            cost += abs(ref[0] - state[0]) + abs(ref[1] - state[1])

            # heading cost
            cost += abs(ref[2] - state[2]) ** 2

        return cost


sim_run(options, ModelPredictiveControl)
