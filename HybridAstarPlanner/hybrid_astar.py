import hybrid_astar_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch


def main():
    """A debug script for the rrt star planner.

    This script will solve the rrt star problem in a
    standalone simulation and visualize the results or raise an error if a
    path is not found.
    """
    print(__file__ + " start!!")
    sim_loop = 100
    area = 20.0  # animation area length [m]
    show_animation = True

    conds = {
        'start': [10, 15, 0],
        'end': [50, 15, 0],
        'obstacles': [
            [26.0, 10.0,
             34.0, 17.0],
            [26.0, 21.0,
             34.0, 28.0],
        ],
    }  # paste output from debug log

    start = conds['start']
    end = conds['end']
    if len(conds['obstacles']) == 0:
        obs = np.empty((0, 4))
    else:
        obs = np.array(conds['obstacles'])

    total_time_taken = 0
    x, y, yaw = start
    start_time = time.time()
    success, (result_x, result_y, result_yaw) = \
        hybrid_astar_wrapper.apply_hybrid_astar([x, y, yaw], end, obs)
    end_time = time.time() - start_time
    print("Time taken: {}s".format(end_time))
    print(success)
    for i in range(sim_loop):
        print("Iteration: {}".format(i))
        x = result_x[0]
        y = result_y[0]
        result_x = result_x[1:]
        result_y = result_y[1:]

        if np.hypot(x - end[0], y - end[1]) <= 1.5:
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
