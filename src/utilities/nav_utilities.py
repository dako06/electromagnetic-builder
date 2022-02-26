
from math import pi

# correct theta to account for discontinuties 
def theta_correction(self, theta):

    # adjust theta if discontinuity occurs
    if theta <= 0 and self.pose.theta >= 0 and self.pose.theta - theta >= pi:
        theta += 2*pi
    elif theta > 0 and self.pose.theta < 0 and theta - self.pose.theta > pi:  
        theta -= 2*pi

    return theta