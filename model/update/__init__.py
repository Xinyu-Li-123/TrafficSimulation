"""
There are two ways to update the location of vehicles:
1. update only based on current time step
    The order of update is
        update location based on current distance, velocity and acceleration
        update distance based on current location
        update velocity based on current distance, velocity and acceleration
        update acceleration based on current distance, velocity and acceleration
2. update based on current and previous time step
"""
