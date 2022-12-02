"""
Parameters of a single vehicle
"""

from utils.utils import KMPH_to_MPS

# Volkswagen Santana 
car_width = 1.7     
car_length = 4.5

vmax = 120
vmax *= KMPH_to_MPS

MIN_FOLLOWING_DISTANCE = 1