import ctypes
import numpy as np
import os

from ctypes import c_double, c_int

try:
    cdll = ctypes.CDLL("build/libHybridAStar.so")
except:
    cdll = ctypes.CDLL(
        "{}/pylot/planning/hybrid_astar/hybrid_astar_planner/"
        "build/libHybridAStar.so".format(os.getcwd())
    )
_c_double_p = ctypes.POINTER(c_double)
_apply_hybrid_astar = cdll.ApplyHybridAStar
_apply_hybrid_astar.argtypes = (
    c_double, c_double, c_double, # starting x, y, yaw
    c_double, c_double, c_double, # ending x, y, yaw
    _c_double_p, _c_double_p, # obstacle lower left x, y
    _c_double_p, _c_double_p, # obstacle upper right x, y
    c_int, # num obstacles
    _c_double_p, _c_double_p, _c_double_p # x, y, yaw return path
)
_apply_hybrid_astar.restype = c_int

def apply_hybrid_astar(start, end, obs):
    result_x = np.zeros(100)
    result_y = np.zeros(100)
    result_yaw = np.zeros(100)

    llx = np.copy(obs[:, 0]).astype(np.float64)
    lly = np.copy(obs[:, 1]).astype(np.float64)
    urx = np.copy(obs[:, 2]).astype(np.float64)
    ury = np.copy(obs[:, 3]).astype(np.float64)

    success = _apply_hybrid_astar(
        c_double(start[0]), c_double(start[1]), c_double(start[2]),
        c_double(end[0]), c_double(end[1]), c_double(end[2]),
        llx.ctypes.data_as(_c_double_p), lly.ctypes.data_as(_c_double_p),
        urx.ctypes.data_as(_c_double_p), ury.ctypes.data_as(_c_double_p),
        c_int(obs.shape[0]),
        result_x.ctypes.data_as(_c_double_p),
        result_y.ctypes.data_as(_c_double_p),
        result_yaw.ctypes.data_as(_c_double_p),
    )

    ind = -1
    if success and np.any(np.isnan(result_x)):
        ind = np.where(np.isnan(result_x))[0][0]

    return success, (result_x[:ind], result_y[:ind], result_yaw[:ind])
