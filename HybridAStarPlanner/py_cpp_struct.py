from ctypes import c_double, c_int, POINTER, Structure, CDLL

_c_double_p = POINTER(c_double)

MAX_PATH_LENGTH = 100

class HybridAStarInitialConditions(Structure):
    _fields_ = [
        ("x_start", c_double),
        ("y_start", c_double),
        ("yaw_start", c_double),
        ("x_end", c_double),
        ("y_end", c_double),
        ("yaw_end", c_double),
        ("o_llx", _c_double_p),
        ("o_lly", _c_double_p),
        ("o_urx", _c_double_p),
        ("o_ury", _c_double_p),
        ("no", c_int)
    ]

class HybridAStarReturnValues(Structure):
    _fields_ = [
        ("success", c_int),
        ("x_path", c_double * MAX_PATH_LENGTH),
        ("y_path", c_double * MAX_PATH_LENGTH),
        ("yaw_path", c_double * MAX_PATH_LENGTH)
    ]

class HybridAStarHyperparameters(Structure):
    _fields_ = [
        ("step_size", c_double),
        ("max_iterations", c_int),
        ("completion_threshold", c_double),
        ("angle_completion_threshold", c_double),
        ("rad_step", c_double),
        ("rad_upper_range", c_double),
        ("rad_lower_range", c_double),
        ("obstacle_clearance", c_double),
        ("lane_width", c_double),
        ("radius", c_double),
        ("car_length", c_double),
        ("car_width", c_double),
    ]