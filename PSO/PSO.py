import pandas as pd
import numpy as np
import random
import seaborn as sns

# Single particles class
class Particle:
    def __init__(self, x0:list, max_iter:int, boundary:list):
        self.position_i = [] # Paritcle position
        self.velocity_i = [] # Particle velocity
        self.pos_best_i = [] # Best position of individual particle
        self.err_best_i = -1 # Best error of individual (Starting at -1)
        self.err_i = -1      # Error of individual
        self.max_iter = max_iter
        self.boundary = boundary
        self.v_max = []
        self.v_min = []

        for i in range(0, num_dimensions):
            self.velocity_i.append(random.uniform(-1, 1))
            self.position_i.append(x0[i])
            
            v_max = (self.boundary[i][-1] - self.boundary[i][0])*0.2
            self.v_max.append(v_max)
            self.v_min.append(-v_max)

    def evaluate(self, costFunc):
        self.err_i = costFunc(self.position_i)

        # Comparing current error with error at current best
        if self.err_i < self.err_best_i or self.err_best_i == -1:
            self.pos_best_i = self.position_i
            self.err_best_i = self.err_i

    def update_velocity(self, pos_best_g, action_rounds):
        w_max = 0.9     # Max inertia weight
        w_min = 0.2        # Min intertia weight
        c1 = 1          # Cognitive weight (individual particle)
        c2 = 2          # Social weight
        w = w_max - action_rounds*(w_max - w_min)/self.max_iter

        for i in range(0, num_dimensions):
            r1 = random.random()
            r2 = random.random()

            vel_cognitive = c1*r1*(self.pos_best_i[i] - self.position_i[i])
            vel_social = c2*r2*(pos_best_g[i] - self.position_i[i])

            self.velocity_i[i] = w*self.velocity_i[i] + vel_cognitive + vel_social
            
            # Check velocity with velocity boundary
            if self.velocity_i[i] > self.v_max[i]:
                self.velocity_i[i] = self.v_max[i]
            if self.velocity_i[i] < self.v_min[i]:
                self.velocity_i[i] = self.v_min[i]
    
    def update_position(self, bounds):
        for i in range(0, num_dimensions):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

            # Check position with boundary in search space
            # Upper bound, if exceed then assign maximum boundary
            if self.position_i[i] > bounds[i][1]:
                self.position_i[i] = bounds[i][1]

            if self.position_i[i] < bounds[i][0]:
                self.position_i[i] = bounds[i][0]


# Swarm class
class PSO():
    def __init__(self, costFunc, x0:list, bounds:list, num_particles:int, max_iter:int):
        self.max_iter = max_iter
        self.num_particles = num_particles
        self.bounds = bounds
        self.costFunc = costFunc

        global num_dimensions
        num_dimensions = len(x0)

        self.err_best_g = -1     # Initial best error for particles group
        self.pos_best_g = []     # Collect best global best positions
        
        # Initiate swarm with particle base given num_particles
        self.swarm = []
        for i in range(0, num_particles):
            self.swarm.append(Particle(x0, self.max_iter, self.bounds))


    def optimize(self):
        # Iterate over max_iter rounds
        i = 0
        while i < self.max_iter:
            # Perform optimize over swarm by looping all particles
            for j in range(0, self.num_particles):
                self.swarm[j].evaluate(self.costFunc)

                if self.swarm[j].err_i < self.err_best_g or self.err_best_g == -1:
                    pos_best_g = list(self.swarm[j].position_i)
                    err_best_g = float(self.swarm[j].err_i)

            for j in range(0, self.num_particles):
                self.swarm[j].update_velocity(pos_best_g, i)
                self.swarm[j].update_position(self.bounds)

            i += 1

        print("FINAL !")
        print(f"Best position : {pos_best_g}")
        print(f"Error at best position : {err_best_g}")
