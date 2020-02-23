#!/usr/bin/env python3
"""
File:          siv_test.py
Author:        Binit Shah
Last Modified: Binit on 2/22
"""

import math
from localization import particle_filter, utilities

if __name__ == "__main__":
    viz = utilities.Visualization()
    grid = particle_filter.Grid()
    particles = grid.initialize_particles(10)
    robot = particle_filter.Particle(utilities.inch_to_meters(9.5), utilities.inch_to_meters(22.5), math.radians(0.0))
    pfilter = particle_filter.ParticleFilter(particles, robot, grid)
    # print(f"pre ({robot.x_m, robot.y_m, robot.theta_rad})")
    # print("pre is_free: ", grid.is_free(robot.x_m, robot.y_m, robot.theta_rad))
    # pfilter.motion_update((3, 3), 4.5)
    # pfilter.motion_update((1, 2), 2)
    # print(f"post ({robot.x_m, robot.y_m, robot.theta_rad})")
    # print("post is_free: ", grid.is_free(robot.x_m, robot.y_m, robot.theta_rad))

    while True:
        viz.draw_background(grid)
        viz.draw_particles(particles)
        viz.draw_robot(robot)
        print(f"robot_pose: ({robot.x_m, robot.y_m, robot.theta_rad})")
        viz.update()
        x = int(input("x = "))
        y = int(input("y = "))
        pfilter.motion_update((x, y), 1)
