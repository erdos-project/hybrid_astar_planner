import ctypes
import numpy as np
import os

from ctypes import c_double, c_int, POINTER, Structure, CDLL, byref

try:
    from py_cpp_struct import HybridAStarInitialConditions, \
        HybridAStarHyperparameters, HybridAStarReturnValues, MAX_PATH_LENGTH

except:
    from hybrid_astar_planner.HybridAStar.py_cpp_struct \
        import HybridAStarInitialConditions, HybridAStarHyperparameters, \
        HybridAStarReturnValues, MAX_PATH_LENGTH

try:
    cdll = CDLL("build/libHybridAStar.so")
except:
    cdll = CDLL("{}/dependencies/hybrid_astar_planner/"
                "build/libHybridAStar.so".format(os.getenv("PYLOT_HOME")))
_c_double_p = POINTER(c_double)

# func / return type declarations for C++ ApplyHybridAStar
_apply_hybrid_astar = cdll.ApplyHybridAStar
_apply_hybrid_astar.argtypes = (POINTER(HybridAStarInitialConditions),
                                POINTER(HybridAStarHyperparameters),
                                POINTER(HybridAStarReturnValues))
_apply_hybrid_astar.restype = None


def apply_hybrid_astar(initial_conditions, hyperparameters):
    """ Run HybridAStar given initial conditions and hyperparameters.

    Args:
        initial_conditions (dict): dict containing the following items
            start (np.ndarray): [x, y, yaw]
            end (np.ndarray): [x, y, yaw]
            obs (np.ndarray): top-down obstacles formatted as follows
                [lower left x, lower left y, upper right x, upper right y]

        hyperparameters (dict): a dict of optional hyperparameters
            step_size (float): sampling distance [m]
            max_iterations (float): maximum number of iterations
            completion_threshold (float): threshold to end position [m]
            angle_completion_threshold (float): threshold to end yaw [rad]
            rad_step (float): turning sampling discretization [rad]
            rad_upper_range (float): maximum turning angle to the right [rad]
            rad_lower_range (float): maximum turning angle to the left [rad]
            obstacle_clearance (float): obstacle clearance threshold [m]
            lane_width (float): road width [m]
            radius (float): minimum turning radius of the car [m]
            car_length (float): length of car [m]
            car_width (float): width of car [m]
    Returns:
        x_path (np.ndarray(float)): x positions of Hybrid A*, if it exists
        y_path (np.ndarray(float)): y positions of Hybrid A*, if it exists
        success (bool): whether Hybrid A* was successful
    """
    # convert initial conditions to hybrid a* format
    hastar_ic = to_hastar_initial_conditions(initial_conditions)

    # parse hyperparameters
    hastar_hp = _parse_hyperparameters(hyperparameters)

    # initialize return values
    hastar_rv = HybridAStarReturnValues(0)

    _apply_hybrid_astar(hastar_ic, hastar_hp, hastar_rv)

    success = hastar_rv.success
    x_path = np.array([hastar_rv.x_path[i] for i in range(MAX_PATH_LENGTH)])
    y_path = np.array([hastar_rv.y_path[i] for i in range(MAX_PATH_LENGTH)])
    yaw_path = np.array(
        [hastar_rv.yaw_path[i] for i in range(MAX_PATH_LENGTH)])

    ind = -1
    if success and np.any(np.isnan(x_path)):
        ind = np.where(np.isnan(x_path))[0][0]

    return x_path[:ind], y_path[:ind], yaw_path[:ind], success


def to_hastar_initial_conditions(initial_conditions):
    x_start = initial_conditions['start'][0]
    y_start = initial_conditions['start'][1]
    yaw_start = initial_conditions['start'][2]
    x_end = initial_conditions['end'][0]
    y_end = initial_conditions['end'][1]
    yaw_end = initial_conditions['end'][2]
    obs = initial_conditions['obs']
    o_llx = np.copy(obs[:, 0]).astype(np.float64)
    o_lly = np.copy(obs[:, 1]).astype(np.float64)
    o_urx = np.copy(obs[:, 2]).astype(np.float64)
    o_ury = np.copy(obs[:, 3]).astype(np.float64)
    return HybridAStarInitialConditions(
        x_start,
        y_start,
        yaw_start,
        x_end,
        y_end,
        yaw_end,
        o_llx.ctypes.data_as(_c_double_p),  # obstacles lower left x
        o_lly.ctypes.data_as(_c_double_p),  # obstacles lower left y
        o_urx.ctypes.data_as(_c_double_p),  # obstacles upper right x
        o_ury.ctypes.data_as(_c_double_p),  # obstacles upper right y
        len(obs))


def _parse_hyperparameters(hp):
    return HybridAStarHyperparameters(
        hp['step_size'],
        hp['max_iterations'],
        hp['completion_threshold'],
        hp['angle_completion_threshold'],
        hp['rad_step'],
        hp['rad_upper_range'],
        hp['rad_lower_range'],
        hp['obstacle_clearance'],
        hp['lane_width'],
        hp['radius'],
        hp['car_length'],
        hp['car_width'],
    )
