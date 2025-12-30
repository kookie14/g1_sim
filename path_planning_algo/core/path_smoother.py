import numpy as np
from scipy import interpolate

def smooth_path_spline(rx, ry, smoothing_factor=0.5, resolution=0.05):
    """
    Smooths a path using B-spline interpolation.
    
    Inputs:
        rx, ry: Raw waypoints from a planner (e.g., A*).
        smoothing_factor: Spline smoothing parameter (0 = exact fit, higher = smoother/more approximation).
        resolution: Step size for the output interpolation [m].
    
    Outputs:
        smooth_x, smooth_y: Lists of smoothed path coordinates.
    """

    # Cannot interpolate a path with fewer than 2 points
    if len(rx) < 2:
        return rx, ry
    
    points = np.array(list(zip(rx, ry)))
    
    # Remove consecutive duplicate points to avoid singularities in spline fitting
    _, unique_indices = np.unique(points, axis=0, return_index=True)
    points = points[np.sort(unique_indices)]
    
    # Re-verify point count after removing duplicates
    if len(points) < 2:
        return list(points[:, 0]), list(points[:, 1])
    
    x = points[:, 0]
    y = points[:, 1]
    
    # B-spline interpolation: 
    # tck represents the knot vector, coefficients, and degree of the spline
    # u represents the parameter values of the points
    tck, u = interpolate.splprep([x, y], s=smoothing_factor, k=min(3, len(x)-1))
    
    # Calculate the number of output points based on the total cumulative path distance
    total_distance = 0
    for i in range(len(x)-1):
        total_distance += np.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
    
    # Ensure sufficient density by taking the maximum of (dist/res) or 10x the original point count
    num_points = max(int(total_distance / resolution), len(x) * 10)
    
    # Create a uniform parameter vector from 0.0 to 1.0
    unew = np.linspace(0, 1.0, num_points)
    
    # Evaluate the spline at the new parameter points to obtain smooth coordinates
    out = interpolate.splev(unew, tck)
    
    return list(out[0]), list(out[1])