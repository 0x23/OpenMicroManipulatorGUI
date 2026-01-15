# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import time
from scipy.optimize import minimize
from skopt import gp_minimize
from skopt.space import Real

from hardware.open_micro_stage_api import OpenMicroStageInterface
from hardware.camera_basler import BaslerCamera


class OpticalAlignment:
    def __init__(self, oms: OpenMicroStageInterface, camera: BaslerCamera):
        self.oms = oms
        self.feed_rate = 50
        self.camera = camera
        self.search_box_size=np.array([1,1,1])
        self.eval_count = 0
        self.best_eval = 0
        self.best_position = None

    def evaluate(self, parameter):
        # parameter = np.clip(parameter, -self.search_box_size, self.search_box_size)

        self.oms.move_to(parameter[0], parameter[1], parameter[2], self.feed_rate, move_immediately=True)
        self.oms.wait_for_stop(polling_interval_ms=1)
        time.sleep(0.1)

        image = self.camera.grab_single_triggered()

        cost = -np.average(image)
       # print(f"brightness at [{parameter[0]} {parameter[1]} {parameter[2]}]: {cost:.3f}")
        return cost  # We negate because we want to maximize brightness

    def optimize(self, search_box_size=(0.01, 0.01, 0.01), n_calls=100):
        self.oms.set_max_acceleration(100, 5000)
        self.search_box_size = search_box_size
        self.camera.start_grabbing(single_grab=True)

        initial_params = [0, 0, 0]
        initial_cost = self.evaluate(initial_params)
        self.eval_count = 0
        self.best_eval = initial_cost
        self.best_position = None

        # Define the parameter search space using Real dimensions
        space = [
            Real(-search_box_size[0], search_box_size[0], name='x'),
            Real(-search_box_size[1], search_box_size[1], name='y'),
            Real(-search_box_size[2], search_box_size[2], name='z'),
        ]

        # Define wrapper to track best result
        def objective(params):
            return self.evaluate(params)

        print(f"\n🔍 Starting Bayesian optimization with {n_calls} evaluations...")

        result = gp_minimize(
            func=objective,
            dimensions=space,
            n_calls=n_calls,
            x0=[initial_params],  # Provide initial guess here
            y0=[initial_cost],  # Provide corresponding objective value
            n_initial_points=10,  # Random initial points
            acq_func="EI",  # Expected improvement
            acq_optimizer="auto",  # Internal optimizer
            noise=1e-7,  # Assume small noise
            verbose=True,
            random_state=1
        )

        self.camera.stop_grabbing()

        # Best found position and brightness
        final_pos = result.x
        final_brightness = -result.fun

        print(f"\n✅ Best position found: {result}, Brightness: {final_brightness:.2f}")
        self.oms.move_to(*final_pos, self.feed_rate, move_immediately=True)

        return final_pos, final_brightness

    def optimize1(self, search_box_size=(0.03, 0.2, 0.03)):
        self.oms.set_max_acceleration(1000, 5000)
        self.camera.start_grabbing(single_grab=True)

        initial_guess = np.array([0, 0, 0])
        self.search_box_size = np.array(search_box_size)

        # Define bounds centered around 0 with width of search_box_size
        bounds = [(-s, s) for s in search_box_size]

        # Use Powell's method for efficient derivative-free optimization
        optimizer_type = 0

        if optimizer_type == 0:
            result = minimize(
                self.evaluate,
                initial_guess,
                method='Powell',
                bounds=bounds,
                options={
                    'xtol': 1e-4,   # Stop when x changes are below this threshold
                    'ftol': 1e-2,   # Stop when f changes are below this threshold
                    'disp': True    # Show optimization progress
                }
            )
        else:
            result = minimize(
                self.evaluate,
                initial_guess,
                method='L-BFGS-B',
                bounds=bounds,
                options={
                    'maxiter': 100,
                    'disp': True,
                   'eps': 0.05  # small step size
               }
            )

  #      result = minimize_lrs(
  #          fun=self.evaluate,
  #          x0=np.array([0, 0, 0]),
  #          bounds=[(-s, s) for s in search_box_size],
  #          search_box_size=search_box_size,
  #          num_iterations=15,
  #          samples_per_iteration=10,
  #          shrink_factor=0.5,
  #          disp=True
  #      )

        self.camera.stop_grabbing()

        best_position = result.x
        best_brightness = -result.fun  # Negate again to get max brightness

        # Optionally move to the best found position
        self.oms.move_to(*best_position, self.feed_rate, move_immediately=True)

        return best_position, best_brightness


import numpy as np
from types import SimpleNamespace

def minimize_lrs(
    fun,
    x0,
    bounds=None,
    search_box_size=None,
    num_iterations=100,
    samples_per_iteration=10,
    shrink_factor=0.5,
    tol=1e-4,
    disp=False
):
    x0 = np.array(x0, dtype=float)
    dim = len(x0)

    if bounds is None:
        # Default to +/-inf if no bounds are given
        bounds = [(-np.inf, np.inf)] * dim

    if search_box_size is None:
        # If not provided, compute as half the bounds width
        search_box_size = np.array([(hi - lo) / 2 for lo, hi in bounds])
    else:
        search_box_size = np.array(search_box_size)

    lower_bounds = np.array([b[0] for b in bounds])
    upper_bounds = np.array([b[1] for b in bounds])

    current_x = x0
    current_fun = fun(current_x)

    for iteration in range(num_iterations):
        improved = False

        for _ in range(samples_per_iteration):
            # Random direction and scale
            offset = (np.random.rand(dim) - 0.5) * 2 * search_box_size
            candidate = current_x + offset
            candidate = np.clip(candidate, lower_bounds, upper_bounds)

            candidate_fun = fun(candidate)

            if candidate_fun < current_fun - tol:
                current_x = candidate
                current_fun = candidate_fun
                improved = True

        if disp:
            print(f"[{iteration+1}/{num_iterations}] f(x) = {current_fun:.6f}, x = {current_x}")

        if not improved:
            search_box_size *= shrink_factor

    result = SimpleNamespace(
        x=current_x,
        fun=current_fun,
        success=True,
        message="Optimization finished",
        nfev=num_iterations * samples_per_iteration,
        nit=num_iterations
    )
    return result

