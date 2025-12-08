"""
Hybrid Trajectory Optimization Strategy

Combines mintime (for speed), mincurv (for drivability), and clothoid smoothing.

Strategy:
1. First: mintime optimization → fastest trajectory with optimal speed profile
2. Then: mincurv reoptimization → smooth out sharp curves
3. Finally: clothoid smoothing → ensure linear curvature changes
"""

import numpy as np
import trajectory_planning_helpers as tph
from . import clothoid_spline


def hybrid_optimize_trajectory(reftrack_interp,
                               coeffs_x_interp,
                               coeffs_y_interp,
                               normvec_normalized_interp,
                               a_interp,
                               pars,
                               v_opt,
                               opt_mintime_traj_module,
                               file_paths,
                               plot_opts=None,
                               debug=True):
    """
    Hybrid optimization approach: mintime → mincurv → clothoid
    
    Parameters:
    -----------
    reftrack_interp : np.ndarray
        Interpolated reference track [N x 4]
    coeffs_x_interp, coeffs_y_interp : spline coefficients
    normvec_normalized_interp : np.ndarray
        Normalized normal vectors [N x 2]
    a_interp : np.ndarray
        Spline arc lengths [N]
    pars : dict
        Vehicle and optimization parameters
    v_opt : np.ndarray
        Speed profile from mintime optimization
    opt_mintime_traj_module : module
        Reference to opt_mintime_traj module
    file_paths : dict
        File paths for optimization
    plot_opts : dict, optional
        Plotting options
    debug : bool
        Print debug messages
    
    Returns:
    --------
    alpha_opt : np.ndarray
        Optimized lateral displacement [N]
    v_opt_final : np.ndarray
        Final speed profile [N]
    raceline_interp : np.ndarray
        Final interpolated raceline [M x 2]
    kappa_opt : np.ndarray
        Final curvature profile [M]
    psi_vel_opt : np.ndarray
        Final heading profile [M]
    """
    
    if debug:
        print("=" * 80)
        print("HYBRID OPTIMIZATION: MINTIME → MINCURV → CLOTHOID")
        print("=" * 80)
    
    # ======== STAGE 1: MINTIME OPTIMIZATION ========================================
    if debug:
        print("\n[STAGE 1] Running mintime optimization...")
    
    from . import opt_mintime
    
    alpha_opt, v_opt_from_mintime, reftrack_interp_mintime, a_interp_tmp, normvec_normalized_interp_mintime = \
        opt_mintime.opt_mintime(
            reftrack=reftrack_interp,
            coeffs_x=coeffs_x_interp,
            coeffs_y=coeffs_y_interp,
            normvectors=normvec_normalized_interp,
            pars=pars,
            tpamap_path=file_paths.get("tpamap"),
            tpadata_path=file_paths.get("tpadata"),
            export_path=file_paths.get("mintime_export", ""),
            print_debug=debug,
            plot_debug=plot_opts.get("mintime_plots", False) if plot_opts else False
        )
    
    # Update states if necessary
    if a_interp_tmp is not None:
        a_interp = a_interp_tmp
    if reftrack_interp_mintime is not None:
        reftrack_interp = reftrack_interp_mintime
    if normvec_normalized_interp_mintime is not None:
        normvec_normalized_interp = normvec_normalized_interp_mintime
    
    v_opt = v_opt_from_mintime.copy()
    
    if debug:
        print(f"  ✓ Mintime result: max speed = {np.max(v_opt):.3f} m/s")
    
    # ======== STAGE 2: MINCURV REOPTIMIZATION ======================================
    if debug:
        print("\n[STAGE 2] Running mincurv reoptimization for smoothness...")
    
    # Get raceline from mintime solution
    raceline_mintime = reftrack_interp[:, :2] + np.expand_dims(alpha_opt, 1) * normvec_normalized_interp
    
    # Calculate new track boundaries around mintime raceline
    w_tr_right_mintime = reftrack_interp[:, 2] - alpha_opt
    w_tr_left_mintime = reftrack_interp[:, 3] + alpha_opt
    
    # Create new reference track around the raceline
    racetrack_mintime = np.column_stack((raceline_mintime, w_tr_right_mintime, w_tr_left_mintime))
    
    # Re-prepare track for mincurv
    from . import prep_track
    reftrack_interp_mincurv, normvec_normalized_interp_mincurv, a_interp_mincurv = \
        prep_track.prep_track(
            reftrack_imp=racetrack_mintime,
            reg_smooth_opts=pars["reg_smooth_opts"],
            stepsize_opts=pars["stepsize_opts"],
            debug=False,
            min_width=None
        )[:3]
    
    # Set artificial narrow track widths for mincurv reoptimization
    w_tr_tmp = 0.5 * pars["optim_opts"]["w_tr_reopt"] * np.ones(reftrack_interp_mincurv.shape[0])
    racetrack_mintime_reopt = np.column_stack((reftrack_interp_mincurv[:, :2], w_tr_tmp, w_tr_tmp))
    
    # Run mincurv reoptimization
    alpha_opt_reopt = tph.opt_min_curv.opt_min_curv(
        reftrack=racetrack_mintime_reopt,
        normvectors=normvec_normalized_interp_mincurv,
        A=a_interp_mincurv,
        kappa_bound=pars["veh_params"]["curvlim"],
        w_veh=pars["optim_opts"]["w_veh_reopt"],
        print_debug=debug,
        plot_debug=plot_opts.get("mincurv_curv_lin", False) if plot_opts else False
    )[0]
    
    # Use reoptimized values
    alpha_opt = alpha_opt_reopt
    reftrack_interp = reftrack_interp_mincurv
    normvec_normalized_interp = normvec_normalized_interp_mincurv
    a_interp = a_interp_mincurv
    
    if debug:
        print(f"  ✓ Mincurv reoptimization complete")
    
    # ======== STAGE 3: INTERPOLATE RACELINE ========================================
    if debug:
        print("\n[STAGE 3] Creating fine-grained raceline...")
    
    raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, \
        t_vals_opt_interp, s_points_opt_interp, spline_lengths_opt, el_lengths_opt_interp = \
        tph.create_raceline.create_raceline(
            refline=reftrack_interp[:, :2],
            normvectors=normvec_normalized_interp,
            alpha=alpha_opt,
            stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"]
        )
    
    # Calculate heading and curvature
    psi_vel_opt, kappa_opt = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x_opt,
        coeffs_y=coeffs_y_opt,
        ind_spls=spline_inds_opt_interp,
        t_spls=t_vals_opt_interp
    )
    
    if debug:
        print(f"  ✓ Raceline interpolated: {len(raceline_interp)} points")
        print(f"    Max curvature (before clothoid): {np.max(np.abs(kappa_opt)):.6f} rad/m")
    
    # ======== STAGE 4: CLOTHOID SMOOTHING ==========================================
    if debug:
        print("\n[STAGE 4] Applying clothoid smoothing...")
    
    kappa_opt_original = kappa_opt.copy()
    raceline_interp_original = raceline_interp.copy()
    
    # Apply clothoid optimization
    raceline_interp_clothoid, kappa_opt_clothoid = \
        clothoid_spline.minimize_curvature_with_clothoid(
            raceline_points=raceline_interp,
            kappa_profile=kappa_opt,
            max_iterations=5,
            tolerance=1e-5,
            max_curvature=pars["veh_params"]["curvlim"],
            debug=debug
        )
    
    # Recalculate splines from clothoid-optimized raceline
    raceline_closed = np.vstack((raceline_interp_clothoid, raceline_interp_clothoid[0]))
    try:
        coeffs_x_clothoid, coeffs_y_clothoid, a_clothoid, _ = \
            tph.calc_splines.calc_splines(path=raceline_closed)
        
        # Re-create raceline with clothoid-optimized path
        raceline_interp_new, a_opt_new, coeffs_x_opt_new, coeffs_y_opt_new, \
            spline_inds_opt_interp_new, t_vals_opt_interp_new, s_points_opt_interp_new, \
            spline_lengths_opt_new, el_lengths_opt_interp_new = \
            tph.create_raceline.create_raceline(
                refline=raceline_interp_clothoid,
                normvectors=normvec_normalized_interp,
                alpha=np.zeros(len(raceline_interp_clothoid)),
                stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"]
            )
        
        # Recalculate heading and curvature
        psi_vel_opt_new, kappa_opt_new = tph.calc_head_curv_an.calc_head_curv_an(
            coeffs_x=coeffs_x_opt_new,
            coeffs_y=coeffs_y_opt_new,
            ind_spls=spline_inds_opt_interp_new,
            t_spls=t_vals_opt_interp_new
        )
        
        # Check if clothoid optimization improved curvature
        max_kappa_original = np.max(np.abs(kappa_opt_original))
        max_kappa_new = np.max(np.abs(kappa_opt_new))
        
        if max_kappa_new < max_kappa_original:
            # Accept clothoid optimization
            raceline_interp = raceline_interp_new
            coeffs_x_opt = coeffs_x_opt_new
            coeffs_y_opt = coeffs_y_opt_new
            psi_vel_opt = psi_vel_opt_new
            kappa_opt = kappa_opt_new
            
            if debug:
                print(f"  ✓ Clothoid optimization successful")
                print(f"    Max curvature reduced: {max_kappa_original:.6f} → {max_kappa_new:.6f} rad/m")
        else:
            if debug:
                print(f"  ⚠ Clothoid optimization did not improve curvature (kept original)")
    
    except Exception as e:
        if debug:
            print(f"  ⚠ Clothoid smoothing failed: {e}. Using original splines.")
    
    # ======== FINAL SUMMARY ========================================================
    if debug:
        print("\n" + "=" * 80)
        print("HYBRID OPTIMIZATION COMPLETE")
        print("=" * 80)
        print(f"Final raceline points: {len(raceline_interp)}")
        print(f"Max speed: {np.max(v_opt):.3f} m/s")
        print(f"Max curvature: {np.max(np.abs(kappa_opt)):.6f} rad/m")
        print(f"Min curvature: {np.min(np.abs(kappa_opt)):.6f} rad/m")
        print("=" * 80 + "\n")
    
    return alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt


def sequential_multi_pass_optimize(reftrack_interp,
                                   coeffs_x_interp,
                                   coeffs_y_interp,
                                   normvec_normalized_interp,
                                   a_interp,
                                   pars,
                                   opt_mintime_traj_module,
                                   file_paths,
                                   num_iterations=3,
                                   debug=True):
    """
    Advanced strategy: Run multiple passes of [mintime → mincurv → clothoid]
    
    Each iteration uses the previous result as input for further refinement.
    
    Parameters:
    -----------
    num_iterations : int
        Number of optimization passes (typically 2-3)
    
    Returns:
    --------
    Same as hybrid_optimize_trajectory
    """
    
    if debug:
        print("=" * 80)
        print(f"MULTI-PASS HYBRID OPTIMIZATION ({num_iterations} iterations)")
        print("=" * 80)
    
    alpha_opt = None
    v_opt = None
    raceline_interp = None
    kappa_opt = None
    psi_vel_opt = None
    
    for iteration in range(num_iterations):
        if debug:
            print(f"\n\n{'#' * 80}")
            print(f"# PASS {iteration + 1}/{num_iterations}")
            print(f"{'#' * 80}\n")
        
        # Run hybrid optimization
        alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
            hybrid_optimize_trajectory(
                reftrack_interp=reftrack_interp,
                coeffs_x_interp=coeffs_x_interp,
                coeffs_y_interp=coeffs_y_interp,
                normvec_normalized_interp=normvec_normalized_interp,
                a_interp=a_interp,
                pars=pars,
                v_opt=v_opt if v_opt is not None else np.zeros(len(reftrack_interp)),
                opt_mintime_traj_module=opt_mintime_traj_module,
                file_paths=file_paths,
                debug=debug
            )
        
        if debug:
            print(f"✓ Pass {iteration + 1} complete. Max curvature: {np.max(np.abs(kappa_opt)):.6f} rad/m\n")
    
    if debug:
        print("\n" + "=" * 80)
        print("MULTI-PASS OPTIMIZATION COMPLETE")
        print("=" * 80 + "\n")
    
    return alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt
