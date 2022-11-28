"""
This is a collection of classes. Each represents a different type of driver onf different types of cars.

Types of drivers:
- Normal Human
- Tailgater (drive aggressively behind the car in front)
- Slowpoke (drive slowly)
- Self-driving (optimal speed, no collisions)

Types of cars:
- Normal Car
- Truck
- Bus
- Self-driving Car

"""

import numpy as np
import utils

class Driver:
    def __init__(self, car, speed=0, lane=0, position=np.zeros(2)):
        self.car = car
        self.speed = 0
        self.lane = lane
        self.position = np.zeros(2)
        self.front = None

    def update(self):
        self.position += self.speed * utils.dt
