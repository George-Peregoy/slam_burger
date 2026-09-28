import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from math import radians, sin, cos

class Ellipse2():
    """
    Class for individual ellipses to sample from.

    Attributes
    ----------
    f1 : tuple
        First focal point.
    f2 : tuple
        Second focal point.
    s : float
        Path length.
    h : float
        x coord of center.
    k : float
        y coord of center.
    e : float
        Eccentricity of ellipse.
    area : float
        Area of ellipse.
    c : float
        Norm of center.
    b : float
        Semi minor axis.
    a : float
        Semi major axis.
    tilt : float
        Angle of ellipse in degrees.

    Methods
    -------
    in_ellipse(point)
        Returns True if point is in ellipse else False.
    """
    
    def __init__(self, f1: tuple, f2: tuple, s: float):
        # Init ellipse 
        self.f1 = tuple(f1)     # foci 1
        self.f2 = tuple(f2)     # foci 2
        self.s = s             # minor axis set as path length
        self._def_params()
        

    def _def_params(self):
        """
        Derives h, k, e, area, c, b, a, tilt.
        """
        # derive center coords
        h,k = (0.5*(self.f1[0]+self.f2[0]),0.5*(self.f1[1]+self.f2[1]))
        self.h = h
        self.k = k
        
        # derive c,b,a,theta
        cx,cy = (self.f2[0]-h, self.f2[1]-k)
        c = np.sqrt(cx*cx + cy*cy)
        a = self.s/2 + 0.01     # adding small delta since using straight lines
        b = np.sqrt(abs(a*a - c*c))
        tilt = np.arctan2(self.f2[1] - self.f1[1], self.f2[0] - self.f1[0])*180/np.pi  # rad -> deg
        
        # calculate eccentricity & area
        e = np.sqrt(abs(a*a - b*b))/a
        area = (np.pi/4) * self.s**2 * np.sqrt(1-e**2)
        self.e = e
        self.area = area
        
        # reassign the rest 
        self.c = c
        self.b = b
        self.a = a
        self.tilt = tilt
        
    def _new_params(self, fnew: int, flag: int):
        """
        Updates focus point and re-derives attributes of ellipse.

        Parameters
        ----------
        fnew : tuple
            New foci. Example (x, y).
        flag : int
            1 if changing foci 1 elif 2 change foci 2.
        """
        
        # flag = 1 or 2; defines which foci is being replaced
        if flag == 1:
            self.f1 = fnew
        elif flag == 2:
            self.f2 = fnew
        
        self._def_params()
    
    def in_ellipse(self, point: tuple):
            """
            Checks if point is in ellipse.

            Parameters
            ----------
            point : tuple
                Point being assessed. Example (x, y).

            Returns
            -------
            True if in ellipse else False.
            """
            x = point[0]
            y = point[1]
        
            # Convert angle to radians
            angle_rad = radians(self.tilt)

            # Calculate the coordinates of the point relative to the ellipse's center
            x_rel = x - self.h
            y_rel = y - self.k

            # Rotate the point to align with the ellipse's axes
            x_rot = x_rel * cos(angle_rad) + y_rel * sin(angle_rad)
            y_rot = -x_rel * sin(angle_rad) + y_rel * cos(angle_rad)

            # Check if the point is within the ellipse
            if (x_rot/self.a)**2 + (y_rot/self.b)**2 <= 1:
                return True
            else:
                return False
        
# -------------------------------------------------------------------------------------------------------------------
# Extra functions:

def samp_ellipse(ell: Ellipse2, num_points: int):
    """
    Take in an Ellipse object and num_points.

    Parameters
    ----------
    ell : Ellipse2
        Ellipse being sampled from.
    num_points : int
        Num points being sampled.

    Returns
    -------
    List of sampled points.
    """

    # Returns tuple of the sampled point
    theta_rotate = np.radians(ell.tilt) # convert to rad

    # Generate random polar coordinates
    r = np.sqrt(np.random.uniform(0, 1, num_points))
    theta = np.random.uniform(0, 2 * np.pi, num_points)

    # Convert polar coordinates to Cartesian coordinates with rotation
    x = ell.h + ell.a * r * np.cos(theta) * np.cos(theta_rotate) - ell.b * r * np.sin(theta) * np.sin(theta_rotate)
    y = ell.k + ell.a * r * np.cos(theta) * np.sin(theta_rotate) + ell.b * r * np.sin(theta) * np.cos(theta_rotate)

    ellipse_samples = zip(x, y)
    sampled_points = [(pt) for pt in ellipse_samples]

    return sampled_points

# ...existing code...

def samp_ellipse_cholesky(ell: Ellipse2, num_points: int):
    """
    Sample points uniformly from inside the ellipse using Cholesky and eigen decomposition.

    Parameters
    ----------
    ell : Ellipse2
        Ellipse being sampled from.
    num_points : int
        Num points being sampled.

    Returns
    -------
    List of sampled points.
    """
    # Ellipse parameters
    a, b = ell.a, ell.b
    theta = np.radians(ell.tilt)
    center = np.array([ell.h, ell.k])

    # Covariance matrix for the ellipse
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    D = np.diag([a**2, b**2])
    cov = R @ D @ R.T

    # Cholesky decomposition
    L = np.linalg.cholesky(cov)

    # Sample points uniformly from the unit circle
    u = np.random.uniform(0, 1, num_points)
    r = np.sqrt(u)
    angles = np.random.uniform(0, 2 * np.pi, num_points)
    circle_samples = np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)

    # Transform to ellipse
    ellipse_samples = center + circle_samples @ L.T

    if num_points == 1:
        return tuple(ellipse_samples[0])
    else:
        return [tuple(pt) for pt in ellipse_samples]

# ...existing code...

def plot_ellipse_with_samples(ell: Ellipse2, num_points: int=100, show: bool=True):
    """
    Plot ellipse with sampled using uniform sampling.

    Parameters
    ----------
    ell : Ellipse2
        Ellipse being sampled from.
    num_points : int
        Num points to sample. default=100
    show : bool
        True if show plot else False. default=True
    """
    # Plot the ellipse and random samples inside it
    fig, ax = plt.subplots()
    
    # Plot the ellipse using matplotlib.patches.Ellipse
    ellipse_patch = Ellipse(
        (ell.h, ell.k), 
        width=2*ell.a, 
        height=2*ell.b, 
        angle=ell.tilt, 
        edgecolor='black', 
        facecolor='none', 
        lw=2
    )
    ax.add_patch(ellipse_patch)
    ax.scatter(*ell.f1, color='green', marker='x', s=100)
    ax.scatter(*ell.f2, color='green', marker='x', s=100)
    
    # Sample points and plot them
    x, y = zip(*samp_ellipse(ell, num_points))
    ax.scatter(x, y, color='red', s=10, label='Samples')
    
    ax.set_aspect('equal')
    # ax.legend()
    plt.title("Transformation Samples")
    if show:
        plt.show()

def plot_ellipse_with_cholesky_samples(ell, num_points=100, show=True):
    """
    Plot the ellipse and random samples inside it using Cholesky sampling

    Parameters
    ----------
    ell : Ellipse2
        Ellipse being sampled from.
    num_points : int
        Num point to sample. default=100
    show : bool
        True if show plot else False. default=True
    """
    fig, ax = plt.subplots()
    
    ellipse_patch = Ellipse(
        (ell.h, ell.k), 
        width=2*ell.a, 
        height=2*ell.b, 
        angle=ell.tilt, 
        edgecolor='black', 
        facecolor='none', 
        lw=2
    )
    ax.add_patch(ellipse_patch)
    ax.scatter(*ell.f1, color='green', marker='x', s=20)
    ax.scatter(*ell.f2, color='green', marker='x', s=20)
    
    samples = samp_ellipse_cholesky(ell, num_points)
    xs, ys = zip(*samples)
    ax.scatter(xs, ys, color='red', s=10, label='Cholesky Samples')
    
    ax.set_aspect('equal')
    plt.title("Cholesky Samples")
    if show:
        plt.show()

def main():
    # Example ellipse: foci at (0,0) and (4,4), minor axis length 5
    
    ell = Ellipse2(f1=(0, 0), f2=(4, 4), s=6)
    #samp_ellipse(ell, 10)
    #samp_ellipse_cholesky(ell, 10)
    #plot_ellipse_with_samples(ell)
    #plot_ellipse_with_cholesky_samples(ell)

if __name__ == "__main__":
    main()