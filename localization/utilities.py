#!/usr/bin/env python3
"""
File:          utilities.py
Author:        Binit Shah
Last Modified: Binit on 2/22
"""

import pygame as pg

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

class Visualization:
    """pygame visualization of the field
    """

    def __init__(self):
        self.WINDOW_DIM = (472, 280)

        pg.init()
        self.display = pg.display.set_mode(self.WINDOW_DIM)

    def draw_background(self, grid):
        self.display.fill((255, 255, 255))

        # Draw board
        pg.draw.rect(self.display, (0, 0, 0), (inch_to_pixels(0), inch_to_pixels(0),
                                               inch_to_pixels(grid.width_in, offset=False),
                                               inch_to_pixels(grid.height_in, offset=False)))

        # Draw walls
        for w in grid.walls_in:
            pg.draw.rect(self.display, (255, 255, 0), (inch_to_pixels(w[0]), inch_to_pixels(w[1]),
                                                       inch_to_pixels(w[2] - w[0], offset=False),
                                                       inch_to_pixels(w[3] - w[1], offset=False)))

    def draw_particles(self, particles):
        for particle in particles:
            particle_pts = particle.vertices()
            particle_pts_pix = [(meters_to_pixels(particle_pts[0][0]), meters_to_pixels(particle_pts[0][1])),
                                (meters_to_pixels(particle_pts[1][0]), meters_to_pixels(particle_pts[1][1])),
                                (meters_to_pixels(particle_pts[2][0]), meters_to_pixels(particle_pts[2][1])),
                                (meters_to_pixels(particle_pts[3][0]), meters_to_pixels(particle_pts[3][1]))]
            s = pg.Surface(self.WINDOW_DIM, pg.SRCALPHA)
            pg.draw.polygon(s, (255, 0, 0, 50), particle_pts_pix)
            self.display.blit(s, dest=(0, 0))

    def draw_robot(self, robot):
        robot_pts = robot.vertices()
        robot_pts_pix = [(meters_to_pixels(robot_pts[0][0]), meters_to_pixels(robot_pts[0][1])),
                         (meters_to_pixels(robot_pts[1][0]), meters_to_pixels(robot_pts[1][1])),
                         (meters_to_pixels(robot_pts[2][0]), meters_to_pixels(robot_pts[2][1])),
                         (meters_to_pixels(robot_pts[3][0]), meters_to_pixels(robot_pts[3][1]))]
        pg.draw.polygon(self.display, (181, 101, 29), robot_pts_pix)

    def update(self):
        # Flip display
        self.display.blit(pg.transform.flip(self.display, False, True), dest=(0, 0))

        pg.display.update()

        # event = pg.event.poll()
        # if event.type == pg.KEYDOWN:
        #     pg.quit()
        #     print("")
        #     exit()
