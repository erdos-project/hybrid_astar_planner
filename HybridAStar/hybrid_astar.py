import hybrid_astar_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch


def main():
    """A debug script for the hybrid astar planner.

    This script will solve the hybrid astar problem in a
    standalone simulation and visualize the results or raise an error if a
    path is not found.
    """
    print(__file__ + " start!!")
    sim_loop = 100
    area = 20.0  # animation area length [m]
    show_animation = True

    initial_conditions = {
        'start': np.array([10, 15, 0]),
        'end': np.array([50, 15, 0]),
        'obs': np.array([
            [26.0, 10.0, 34.0, 17.0],
            [26.0, 21.0, 34.0, 28.0],
        ]),
    }  # paste output from debug log

    hyperparameters = {
        "step_size": 3.0,
        "max_iterations": 10000,
        "completion_threshold": 1.0,
        "angle_completion_threshold": 3.0,
        "rad_step": 0.5,
        "rad_upper_range": 4.0,
        "rad_lower_range": 4.0,
        "obstacle_clearance": 0.5,
        "lane_width": 6.0,
        "radius": 6.0,
        "car_length": 4.8,
        "car_width": 1.8,
    }
    start_time = time.time()
    result_x, result_y, result_yaw, success = \
        hybrid_astar_wrapper.apply_hybrid_astar(initial_conditions,
                                                hyperparameters)
    end_time = time.time() - start_time
    print("Time taken: {}s".format(end_time))
    print(success)
    if not success:
        print("FAILED")
        return
    start = initial_conditions['start']
    end = initial_conditions['end']
    obs = initial_conditions['obs']
    for i in range(sim_loop):
        print("Iteration: {}".format(i))
        x = result_x[0]
        y = result_y[0]
        result_x = result_x[1:]
        result_y = result_y[1:]

        if np.hypot(x - end[0], y - end[1]) <= 2:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]),
                                       o[2] - o[0],
                                       o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(start[0], start[1], "og")
            plt.plot(end[0], end[1], "or")
            if success:
                plt.plot(result_x[1:], result_y[1:], ".r")
                plt.plot(result_x[1], result_y[1], "vc")
                plt.xlim(result_x[1] - area, result_x[1] + area)
                plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.grid(True)
            plt.pause(0.1)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(1)
        plt.show()


if __name__ == '__main__':
    main()
