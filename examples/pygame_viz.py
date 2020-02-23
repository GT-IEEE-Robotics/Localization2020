#!/usr/bin/env python3
"""
File:          pygame_viz.py
Author:        Binit Shah
Last Modified: Binit on 2/18
"""

import math
import random
import pygame as pg
from localization import particle_filter

WINDOW_DIM = (472, 280)

def inch_to_pixels(val_in, offset=True):
    # 100in to 400 pixels
    if offset:
        return (val_in * 4) + 50
    else:
        return val_in * 4

def meters_to_inch(val_m):
    return val_m * 39.37007874

def inch_to_meters(val_in):
    return val_in * 0.0254

def meters_to_pixels(val_m, offset=True):
    return inch_to_pixels(meters_to_inch(val_m), offset)

if __name__ == "__main__":
    pg.init()
    grid = particle_filter.Grid()

    display = pg.display.set_mode(WINDOW_DIM)
    display.fill((255, 255, 255))

    # Draw board
    pg.draw.rect(display, (0, 0, 0), (inch_to_pixels(0), inch_to_pixels(0),
                                      inch_to_pixels(grid.width_in, offset=False), inch_to_pixels(grid.height_in, offset=False)))

    # Draw walls
    for w in grid.walls_in:
        pg.draw.rect(display, (255, 255, 0), (inch_to_pixels(w[0]), inch_to_pixels(w[1]),
                                              inch_to_pixels(w[2] - w[0], offset=False), inch_to_pixels(w[3] - w[1], offset=False)))

    # Draw particles
    particles = grid.initialize_particles(500)
    for particle in particles:
        robot_pts = particle.vertices()
        robot_pts_pix = [(meters_to_pixels(robot_pts[0][0]), meters_to_pixels(robot_pts[0][1])),
                         (meters_to_pixels(robot_pts[1][0]), meters_to_pixels(robot_pts[1][1])),
                         (meters_to_pixels(robot_pts[2][0]), meters_to_pixels(robot_pts[2][1])),
                         (meters_to_pixels(robot_pts[3][0]), meters_to_pixels(robot_pts[3][1]))]
        s = pg.Surface(WINDOW_DIM, pg.SRCALPHA)
        pg.draw.polygon(s, (random.randint(0,255), random.randint(0,255), random.randint(0,255), 20), robot_pts_pix)
        display.blit(s, dest=(0, 0))

    # Draw robot
    robot = particle_filter.Particle(inch_to_meters(9.5), inch_to_meters(22.5), math.radians(0.0))
    robot_pts = robot.vertices()
    robot_pts_pix = [(meters_to_pixels(robot_pts[0][0]), meters_to_pixels(robot_pts[0][1])),
                     (meters_to_pixels(robot_pts[1][0]), meters_to_pixels(robot_pts[1][1])),
                     (meters_to_pixels(robot_pts[2][0]), meters_to_pixels(robot_pts[2][1])),
                     (meters_to_pixels(robot_pts[3][0]), meters_to_pixels(robot_pts[3][1]))]
    pg.draw.polygon(display, (181, 101, 29), robot_pts_pix)

    # Particle Filter
    pfilter = particle_filter.ParticleFilter(particles, robot, grid)
    print(f"pre ({robot.x_m, robot.y_m, robot.theta_rad})")
    print("pre is_free: ", grid.is_free(robot.x_m, robot.y_m, robot.theta_rad))
    pfilter.motion_update((3, 3), 4.5)
    pfilter.motion_update((1, 2), 2)
    print(f"post ({robot.x_m, robot.y_m, robot.theta_rad})")
    print("post is_free: ", grid.is_free(robot.x_m, robot.y_m, robot.theta_rad))

    # Draw robot
    robot1_pts = robot.vertices()
    robot1_pts_pix = [(meters_to_pixels(robot1_pts[0][0]), meters_to_pixels(robot1_pts[0][1])),
                     (meters_to_pixels(robot1_pts[1][0]), meters_to_pixels(robot1_pts[1][1])),
                     (meters_to_pixels(robot1_pts[2][0]), meters_to_pixels(robot1_pts[2][1])),
                     (meters_to_pixels(robot1_pts[3][0]), meters_to_pixels(robot1_pts[3][1]))]
    pg.draw.polygon(display, (0, 101, 29), robot1_pts_pix)

    # Flip display to render correctly
    display.blit(pg.transform.flip(display, False, True), dest=(0, 0))

    while True:
        pg.display.update()

        event = pg.event.poll()
        if event.type == pg.KEYDOWN:
            pg.quit()
            print("")
            exit()
