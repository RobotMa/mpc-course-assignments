import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {"FIG_SIZE": [8, 8], "FULL_RECALCULATE": True}


class ModelPredictiveControl:
    def __init__(self):
        self.horizon: int = 20
        self.dt: float = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50.0, 0.0, 0.0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3]  # m/s
        a_t = pedal

        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + pedal * dt - v_t / 25.0  # last term is air resistance

        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(0, self.horizon):
            state = self.plant_model(state, self.dt, u[2 * i], u[2 * i + 1])

            # position cost 1 (prioritize reaching the goal)
            cost += abs(ref[0] - state[0]) ** 2

            # position cost 2 (prioritize staying close to the goal)
            cost += abs(ref[0] - state[0])

            # velocity cost
            speed_mph = state[3] * 3.6
            if speed_mph > 10:
                cost += 10 * abs(speed_mph - 30) ** 2

        return cost


sim_run(options, ModelPredictiveControl)
