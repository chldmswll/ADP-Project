"""
Clothoid spline generation for minimum curvature race line optimization.

Created for:
Race line generation using clothoid curves to minimize curvature for vehicle drivability.
"""

import numpy as np
import trajectory_planning_helpers as tph


def optimize_clothoid_spline(raceline_points, kappa_profile, max_curvature=None, debug=False):
    """
    Optimize raceline using clothoid splines to minimize curvature.
    
    Clothoid curves have linear curvature change: κ(s) = κ₀ + κ'·s
    This ensures smooth transitions and minimizes maximum curvature for better drivability.
    
    Parameters:
    raceline_points: Array of raceline points [N x 2]
    kappa_profile: Current curvature profile [N]
    max_curvature: Maximum allowed curvature (rad/m)
    debug: Print debug messages
    
    Returns:
    raceline_clothoid: Optimized raceline points [N x 2]
    kappa_clothoid: Optimized curvature profile [N]
    """
    if len(raceline_points) < 3:
        return raceline_points, kappa_profile
    
    n_points = len(raceline_points)
    raceline_clothoid = np.zeros_like(raceline_points)
    kappa_clothoid = np.zeros_like(kappa_profile)
    
    # Calculate cumulative arc length
    diffs = np.diff(raceline_points, axis=0)
    el_lengths = np.linalg.norm(diffs, axis=1)
    s_cumulative = np.cumsum(np.insert(el_lengths, 0, 0.0))
    
    # Calculate initial heading from first two points
    if len(raceline_points) > 1:
        initial_direction = raceline_points[1] - raceline_points[0]
        current_heading = np.arctan2(initial_direction[1], initial_direction[0])
    else:
        current_heading = 0.0
    
    # Smooth curvature profile to ensure linear transitions (clothoid property)
    # Apply weighted moving average to make curvature changes more linear
    window_size = min(7, max(3, n_points // 15))
    if window_size > 1 and window_size % 2 == 1:
        # Use Gaussian-like weights for smoother transitions
        weights = np.exp(-0.5 * ((np.arange(window_size) - window_size//2) / (window_size/4))**2)
        weights = weights / np.sum(weights)
        
        # Pad curvature profile for convolution
        kappa_padded = np.pad(kappa_profile, (window_size//2, window_size//2), mode='wrap')
        kappa_smooth = np.convolve(kappa_padded, weights, mode='valid')
    else:
        kappa_smooth = kappa_profile.copy()
    
    # Ensure curvature is within limits and minimize maximum curvature
    if max_curvature is not None:
        kappa_smooth = np.clip(kappa_smooth, -max_curvature, max_curvature)
    
    # Apply additional smoothing to reduce curvature peaks
    # This helps achieve minimum curvature for drivability
    for _ in range(2):
        kappa_smooth = 0.7 * kappa_smooth + 0.15 * np.roll(kappa_smooth, 1) + 0.15 * np.roll(kappa_smooth, -1)
        if max_curvature is not None:
            kappa_smooth = np.clip(kappa_smooth, -max_curvature, max_curvature)
    
    # Reconstruct path using clothoid-based smoothing
    # For each segment, ensure curvature changes linearly (clothoid property)
    raceline_clothoid[0] = raceline_points[0]
    kappa_clothoid[0] = kappa_smooth[0]
    
    for i in range(1, n_points):
        # Calculate desired curvature change (linear interpolation for clothoid)
        if i < n_points - 1:
            kappa_0 = kappa_smooth[i-1]
            kappa_1 = kappa_smooth[i]
        else:
            # For last point, transition to first point (closed track)
            kappa_0 = kappa_smooth[i-1]
            kappa_1 = kappa_smooth[0]
        
        # Calculate curvature change rate (κ' for clothoid: κ(s) = κ₀ + κ'·s)
        ds = max(el_lengths[i-1], 0.01)
        kappa_dot = (kappa_1 - kappa_0) / ds
        
        # Clothoid heading change: θ(s) = θ₀ + κ₀·s + (1/2)·κ'·s²
        dtheta = kappa_0 * ds + 0.5 * kappa_dot * ds * ds
        
        # Update heading
        current_heading += dtheta
        
        # Calculate new position using clothoid integration
        # For small angles, use approximation; for larger, use numerical integration
        if abs(dtheta) < 0.1:
            # Small angle approximation
            dx = ds * (np.cos(current_heading - dtheta/2) - dtheta**2/24 * np.sin(current_heading - dtheta/2))
            dy = ds * (np.sin(current_heading - dtheta/2) + dtheta**2/24 * np.cos(current_heading - dtheta/2))
        else:
            # Numerical integration for clothoid segment
            n_segments = max(3, int(abs(dtheta) * 10))
            dx_sum = 0.0
            dy_sum = 0.0
            heading_local = current_heading - dtheta
            
            for j in range(n_segments):
                s_local = (j + 0.5) * ds / n_segments
                kappa_local = kappa_0 + kappa_dot * s_local
                dtheta_local = kappa_local * (ds / n_segments)
                heading_local += dtheta_local
                dx_sum += (ds / n_segments) * np.cos(heading_local)
                dy_sum += (ds / n_segments) * np.sin(heading_local)
            
            dx = dx_sum
            dy = dy_sum
        
        raceline_clothoid[i] = raceline_clothoid[i-1] + np.array([dx, dy])
        
        # Update curvature (linear interpolation for clothoid)
        kappa_clothoid[i] = kappa_0 + kappa_dot * ds
    
    # For closed tracks, ensure continuity
    if np.linalg.norm(raceline_points[0] - raceline_points[-1]) < 0.1:
        # Adjust last point to match first point
        raceline_clothoid[-1] = raceline_clothoid[0]
        kappa_clothoid[-1] = kappa_clothoid[0]
    
    if debug:
        max_kappa_original = np.max(np.abs(kappa_profile))
        max_kappa_optimized = np.max(np.abs(kappa_clothoid))
        reduction = max_kappa_original - max_kappa_optimized
        print(f"INFO: Clothoid optimization applied. Max curvature: {max_kappa_optimized:.6f} rad/m "
              f"(reduction: {reduction:.6f} rad/m)")
    
    return raceline_clothoid, kappa_clothoid


def apply_clothoid_smoothing(coeffs_x, coeffs_y, spline_inds, t_vals, 
                             raceline_points, kappa_profile, max_curvature=None, debug=False):
    """
    Apply clothoid-based smoothing to spline coefficients to minimize curvature.
    
    This function modifies the spline coefficients to ensure the resulting path
    follows clothoid curves (linear curvature change) for better drivability.
    
    Parameters:
    coeffs_x: Spline coefficients for x-component
    coeffs_y: Spline coefficients for y-component
    spline_inds: Spline indices
    t_vals: Parameter values
    raceline_points: Current raceline points [N x 2]
    kappa_profile: Current curvature profile [N]
    max_curvature: Maximum allowed curvature (rad/m)
    debug: Print debug messages
    
    Returns:
    coeffs_x_opt: Optimized spline coefficients for x-component
    coeffs_y_opt: Optimized spline coefficients for y-component
    raceline_opt: Optimized raceline points [N x 2]
    kappa_opt: Optimized curvature profile [N]
    """
    # First, optimize the raceline using clothoid curves
    raceline_opt, kappa_opt = optimize_clothoid_spline(
        raceline_points, kappa_profile, max_curvature=max_curvature, debug=debug
    )
    
    # Recalculate spline coefficients from optimized raceline
    # Close the path for spline calculation
    raceline_closed = np.vstack((raceline_opt, raceline_opt[0]))
    
    try:
        coeffs_x_opt, coeffs_y_opt, _, _ = tph.calc_splines.calc_splines(path=raceline_closed)
        
        if debug:
            print(f"INFO: Recalculated spline coefficients from clothoid-optimized raceline")
    except Exception as e:
        if debug:
            print(f"WARNING: Failed to recalculate splines, using original coefficients: {e}")
        coeffs_x_opt = coeffs_x
        coeffs_y_opt = coeffs_y
    
    return coeffs_x_opt, coeffs_y_opt, raceline_opt, kappa_opt


def minimize_curvature_with_clothoid(raceline_points, kappa_profile, 
                                     max_iterations=10, tolerance=1e-4, 
                                     max_curvature=None, debug=False):
    """
    Iteratively minimize curvature using clothoid curve optimization.
    
    Parameters:
    raceline_points: Initial raceline points [N x 2]
    kappa_profile: Initial curvature profile [N]
    max_iterations: Maximum number of iterations
    tolerance: Convergence tolerance
    max_curvature: Maximum allowed curvature (rad/m)
    debug: Print debug messages
    
    Returns:
    raceline_opt: Optimized raceline points [N x 2]
    kappa_opt: Optimized curvature profile [N]
    """
    raceline_current = raceline_points.copy()
    kappa_current = kappa_profile.copy()
    
    prev_max_curvature = np.max(np.abs(kappa_current))
    
    for iteration in range(max_iterations):
        # Apply clothoid smoothing
        raceline_current, kappa_current = optimize_clothoid_spline(
            raceline_current, kappa_current, max_curvature=max_curvature, debug=False
        )
        
        # Check convergence
        current_max_curvature = np.max(np.abs(kappa_current))
        curvature_reduction = prev_max_curvature - current_max_curvature
        
        if debug and iteration % 2 == 0:
            print(f"Iteration {iteration}: Max curvature = {current_max_curvature:.6f} rad/m "
                  f"(reduction: {curvature_reduction:.6f})")
        
        if curvature_reduction < tolerance:
            if debug:
                print(f"Converged after {iteration} iterations")
            break
        
        prev_max_curvature = current_max_curvature
    
    if debug:
        print(f"Final max curvature: {np.max(np.abs(kappa_current)):.6f} rad/m")
    
    return raceline_current, kappa_current

