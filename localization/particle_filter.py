import numpy as np
import random
import math

class Particle:
    def __init__(self, x, y, theta):
        self.x_m = x
        self.y_m = y
        self.theta_rad = theta
        self.robot_width_m = 0.2286  # 9 inches
        self.robot_height_m = 0.2667 # 10.5 inches

    def vertices(self):
        robot_pts = [[self.x_m + (0.5 * self.robot_width_m * math.cos(self.theta_rad) - 0.5 * self.robot_height_m * math.sin(self.theta_rad)),
                      self.y_m + (0.5 * self.robot_width_m * math.sin(self.theta_rad) + 0.5 * self.robot_height_m * math.cos(self.theta_rad))],
                     [self.x_m + (-0.5 * self.robot_width_m * math.cos(self.theta_rad) - 0.5 * self.robot_height_m * math.sin(self.theta_rad)),
                      self.y_m + (-0.5 * self.robot_width_m * math.sin(self.theta_rad) + 0.5 * self.robot_height_m * math.cos(self.theta_rad))],
                     [self.x_m + (-0.5 * self.robot_width_m * math.cos(self.theta_rad) + 0.5 * self.robot_height_m * math.sin(self.theta_rad)),
                      self.y_m + (-0.5 * self.robot_width_m * math.sin(self.theta_rad) - 0.5 * self.robot_height_m * math.cos(self.theta_rad))],
                     [self.x_m + (0.5 * self.robot_width_m * math.cos(self.theta_rad) + 0.5 * self.robot_height_m * math.sin(self.theta_rad)),
                      self.y_m + (0.5 * self.robot_width_m * math.sin(self.theta_rad) - 0.5 * self.robot_height_m * math.cos(self.theta_rad))]]
        return robot_pts

class Grid:
    """Defines the Southeastcon grid. Origin at bottomleft."""
    def __init__(self):
        self.width_in = 93.0
        self.height_in = 45.0
        self.width_m = 2.3622 # 93.0 inches
        self.height_m = 1.143 # 45.0 inches

        self.walls_in = [(19.255, 0, 20.005, 12), (30.005, 0, 30.755, 12), (40.775, 0, 41.505, 12), (51.505, 0, 52.255, 12), (62.255, 0, 63.005, 12), (73.005, 0, 73.755, 12),
                         (19.255, 33.0, 20.005, 45.0), (30.005, 33.0, 30.755, 45.0), (40.775, 33.0, 41.505, 45.0), (51.505, 33.0, 52.255, 45.0), (62.255, 33.0, 63.005, 45.0), (73.005, 33.0, 73.755, 45.0),
                         (-1.5, -1.5, 94.5, 0), (-1.5, 0, 0, 45.0), (93.0, 0, 94.5, 45.0), (-1.5, 45.0, 94.5, 46.5)]

        # (bottomleft_corner x_m, y_m, topright_corner x_m, y_m)
        self.walls_m = [(0.48907699999999993, 0.0, 0.508127, 0.30479999999999996),
                        (0.762127, 0.0, 0.7811769999999999, 0.30479999999999996),
                        (1.035685, 0.0, 1.054227, 0.30479999999999996),
                        (1.308227, 0.0, 1.327277, 0.30479999999999996),
                        (1.581277, 0.0, 1.600327, 0.30479999999999996),
                        (1.8543269999999998, 0.0, 1.8733769999999998, 0.30479999999999996),
                        (0.48907699999999993, 0.8382, 0.508127, 1.143),
                        (0.762127, 0.8382, 0.7811769999999999, 1.143),
                        (1.035685, 0.8382, 1.054227, 1.143),
                        (1.308227, 0.8382, 1.327277, 1.143),
                        (1.581277, 0.8382, 1.600327, 1.143),
                        (1.8543269999999998, 0.8382, 1.8733769999999998, 1.143),
                        (-0.038099999999999995, -0.038099999999999995, 2.4003, 0.0),
                        (-0.038099999999999995, 0.0, 0.0, 1.143),
                        (2.3622, 0.0, 2.4003, 1.143),
                        (-0.038099999999999995, 1.143, 2.4003, 1.1811)]

    def is_overlapping(self, robot, wall):
        """
        * Helper function to determine whether there is an intersection between the two polygons described
        * by the lists of vertices. Uses the Separating Axis Theorem
        * https://stackoverflow.com/a/56962827
        *
        * @return true if there is any intersection between the 2 polygons, false otherwise
        """
        a = robot.vertices()
        b = [[wall[0], wall[1]], [wall[0], wall[3]], [wall[2], wall[3]], [wall[2], wall[1]]]
        polygons = [a, b];
        minA, maxA, projected, i, i1, j, minB, maxB = None, None, None, None, None, None, None, None

        for i in range(len(polygons)):

            # for each polygon, look at each edge of the polygon, and determine if it separates
            # the two shapes
            polygon = polygons[i];
            for i1 in range(len(polygon)):

                # grab 2 vertices to create an edge
                i2 = (i1 + 1) % len(polygon);
                p1 = polygon[i1];
                p2 = polygon[i2];

                # find the line perpendicular to this edge
                normal = { 'x': p2[1] - p1[1], 'y': p1[0] - p2[0] };

                minA, maxA = None, None
                # for each vertex in the first shape, project it onto the line perpendicular to the edge
                # and keep track of the min and max of these values
                for j in range(len(a)):
                    projected = normal['x'] * a[j][0] + normal['y'] * a[j][1];
                    if (minA is None) or (projected < minA): 
                        minA = projected

                    if (maxA is None) or (projected > maxA):
                        maxA = projected

                # for each vertex in the second shape, project it onto the line perpendicular to the edge
                # and keep track of the min and max of these values
                minB, maxB = None, None
                for j in range(len(b)): 
                    projected = normal['x'] * b[j][0] + normal['y'] * b[j][1]
                    if (minB is None) or (projected < minB):
                        minB = projected

                    if (maxB is None) or (projected > maxB):
                        maxB = projected

                # if there is no overlap between the projects, the edge we are looking at separates the two
                # polygons, and we know there is no overlap
                if (maxA < minB) or (maxB < minA):
                    return False;

        return True

    def is_free(self, x_m, y_m, theta_rad):
        free = True
        for w in self.walls_m:
            if self.is_overlapping(Particle(x_m, y_m, theta_rad), w):
                free = False
        return free

    def random_free_pose(self):
        while True:
            x = random.uniform(0, self.width_m)
            y = random.uniform(0, self.height_m)
            theta = random.uniform(0, 360)
            if self.is_free(x, y, theta):
                return (x, y, theta)

    def initialize_particles(self, num_particles):
        return [Particle(*self.random_free_pose()) for _ in range(num_particles)]

class ParticleFilter:
    def __init__(self, particles, robot, grid):
        self.particles = particles
        self.robot = robot
        self.grid = grid
        self.R = 0.045  # radius of wheel in meters
        self.L = 0.1    # distance to single integrator point in meters
        self.D = 0.2427 # wheel base in meters
        self.translation_sigma = 0
        self.heading_sigma = 0

    def motion_update(self, odom, dt):
        """
        Applies a motion update to all particles based on wheel odometry.

        @param odom wheel odometry tuple in form (lmotor_vel, rmotor_vel)
        """
        omega_left, omega_right = odom

        for particle in self.particles:
            xdot = omega_right * (self.R/2 * math.cos(particle.theta_rad) - self.R * self.L/self.D * math.sin(particle.theta_rad)) + \
                    omega_left * (self.R/2 * math.cos(particle.theta_rad) + self.R * self.L/self.D * math.sin(particle.theta_rad)) + \
                    random.gauss(0.0, self.translation_sigma)
            ydot = omega_right * (self.R/2 * math.sin(particle.theta_rad) + self.R * self.L/self.D * math.cos(particle.theta_rad)) + \
                    omega_left * (self.R/2 * math.sin(particle.theta_rad) - self.R * self.L/self.D * math.cos(particle.theta_rad)) + \
                    random.gauss(0.0, self.translation_sigma)
            thetadot = omega_right * self.R/self.D - omega_left * self.R/self.D + random.gauss(0.0, self.heading_sigma)

            particle.x_m += xdot * dt
            particle.y_m += ydot * dt
            particle.theta_rad += thetadot * dt
            particle.theta_rad = particle.theta_rad % (2 * np.pi)

        # robot
        xdot = omega_right * (self.R/2 * math.cos(self.robot.theta_rad) - self.R * self.L/self.D * math.sin(self.robot.theta_rad)) + \
                omega_left * (self.R/2 * math.cos(self.robot.theta_rad) + self.R * self.L/self.D * math.sin(self.robot.theta_rad)) + \
                random.gauss(0.0, self.translation_sigma)
        ydot = omega_right * (self.R/2 * math.sin(self.robot.theta_rad) + self.R * self.L/self.D * math.cos(self.robot.theta_rad)) + \
                omega_left * (self.R/2 * math.sin(self.robot.theta_rad) - self.R * self.L/self.D * math.cos(self.robot.theta_rad)) + \
                random.gauss(0.0, self.translation_sigma)
        thetadot = omega_right * self.R/self.D - omega_left * self.R/self.D + random.gauss(0.0, self.heading_sigma)

        self.robot.x_m += xdot * dt
        self.robot.y_m += ydot * dt
        self.robot.theta_rad += thetadot * dt
        self.robot.theta_rad = self.robot.theta_rad % (2 * np.pi)

    def measurement_update(self, markers_list):
        pass

    def compute_mean_pose(self):
        """ Averages the particles.

        :return: (x, y, theta), confidence
        """
        pass
