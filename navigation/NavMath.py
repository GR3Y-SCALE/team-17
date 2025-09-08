import numpy as np
import math

class NavMath:

    def clip_deg_fov(angle, FOV):
        if angle <= 0:
            angle = 0
        elif angle >= FOV:
            angle = FOV
        return angle