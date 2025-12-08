"""
Integration guide: How to use hybrid_optimizer in trajectory_optimizer.py

This document explains how to integrate the hybrid optimization strategy
into the existing trajectory optimization pipeline.
"""

# ============================================================================
# METHOD 1: Simple Integration (Replace default mintime)
# ============================================================================
# Location: trajectory_optimizer.py, line ~600
#
# Replace the single mintime call with hybrid optimization:
#
#    # OLD CODE:
#    elif curv_opt_type == 'mintime':
#        alpha_opt, v_opt, reftrack_interp, a_interp_tmp, normvec_normalized_interp = \
#            opt_mintime_traj.src.opt_mintime.opt_mintime(...)
#
#    # NEW CODE:
#    elif curv_opt_type == 'mintime':
#        from helper_funcs_glob.src import hybrid_optimizer
#        alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
#            hybrid_optimizer.hybrid_optimize_trajectory(
#                reftrack_interp=reftrack_interp,
#                coeffs_x_interp=coeffs_x_interp,
#                coeffs_y_interp=coeffs_y_interp,
#                normvec_normalized_interp=normvec_normalized_interp,
#                a_interp=a_interp,
#                pars=pars_tmp,
#                v_opt=np.zeros(reftrack_interp.shape[0]),  # Initial speed profile
#                opt_mintime_traj_module=opt_mintime_traj,
#                file_paths=file_paths,
#                plot_opts=plot_opts,
#                debug=debug
#            )


# ============================================================================
# METHOD 2: Configuration-based Selection
# ============================================================================
# Add a new optimization type option:
#
#    # In user input section, change:
#    # opt_type = 'mintime'
#
#    # To:
#    opt_type = 'mintime_hybrid'  # or 'mintime_multipass'
#
#    # Then update the optimization type check:
#    if opt_type not in ["shortest_path", "mincurv", "mincurv_iqp", "mintime",
#                        "mintime_hybrid", "mintime_multipass"]:
#        raise IOError("Unknown optimization type!")
#
#    # Then in the optimization call section:
#    elif opt_type == 'mintime_hybrid':
#        from helper_funcs_glob.src import hybrid_optimizer
#        alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
#            hybrid_optimizer.hybrid_optimize_trajectory(...)
#
#    elif opt_type == 'mintime_multipass':
#        from helper_funcs_glob.src import hybrid_optimizer
#        alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
#            hybrid_optimizer.sequential_multi_pass_optimize(
#                num_iterations=3,
#                ...
#            )


# ============================================================================
# METHOD 3: Parameter-based Control
# ============================================================================
# Use existing parameters to enable hybrid optimization:
#
#    mintime_opts = {
#        "tpadata": None,
#        "warm_start": False,
#        "var_friction": None,
#        "reopt_mintime_solution": False,  # OLD: just mincurv reopt
#        "recalc_vel_profile_by_tph": False,
#        "use_hybrid_optimization": True,   # NEW: enable full hybrid
#        "use_multipass": False,            # NEW: enable multi-pass
#        "multipass_iterations": 2          # NEW: number of passes
#    }
#
#    # Then in code:
#    if mintime_opts.get("use_hybrid_optimization", False):
#        # Use hybrid optimizer
#        from helper_funcs_glob.src import hybrid_optimizer
#        if mintime_opts.get("use_multipass", False):
#            num_iters = mintime_opts.get("multipass_iterations", 2)
#            alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
#                hybrid_optimizer.sequential_multi_pass_optimize(
#                    num_iterations=num_iters,
#                    ...
#                )
#        else:
#            alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
#                hybrid_optimizer.hybrid_optimize_trajectory(...)
#    else:
#        # Use original mintime + optional mincurv reopt
#        alpha_opt, v_opt, reftrack_interp, a_interp_tmp, normvec_normalized_interp = \
#            opt_mintime_traj.src.opt_mintime.opt_mintime(...)


# ============================================================================
# COMPARISON OF STRATEGIES
# ============================================================================
#
# Strategy          | Speed | Drivability | Smoothness | Computation Time
# ================================================================================
# mintime alone     | HIGH  | MEDIUM      | MEDIUM     | FAST (1x)
# mintime + mincurv | HIGH  | MEDIUM      | HIGH       | MEDIUM (2x)
# mintime_hybrid    | HIGH  | HIGH        | HIGH       | SLOW (3x)
# multipass_hybrid  | HIGH  | HIGHEST     | HIGHEST    | VERY SLOW (6-9x)
#
# Recommendation:
# - Fast tracks (wide) → mintime alone
# - Normal tracks     → mintime + mincurv
# - Tight tracks      → mintime_hybrid (with 1 pass)
# - Technical tracks  → mintime_multipass (with 2-3 passes)


# ============================================================================
# RESULT INTERPRETATION
# ============================================================================
#
# hybrid_optimize_trajectory returns:
# 1. alpha_opt       : Lateral displacement from reference track [N]
# 2. v_opt           : Speed profile with time-optimal values [N]
# 3. raceline_interp : Final interpolated raceline [M x 2]
# 4. kappa_opt       : Curvature profile (smoothed with clothoid) [M]
# 5. psi_vel_opt     : Heading profile [M]
#
# These can be directly used in the post-processing steps without modification.


# ============================================================================
# VALIDATION CHECKLIST
# ============================================================================
#
# Before using in production:
# □ Check max curvature is within vehicle limits (kappa_opt vs curvlim)
# □ Verify raceline stays within track boundaries (d_right, d_left)
# □ Confirm speed profile is physically feasible (v vs vehicle limits)
# □ Test clothoid smoothing improves drivability (compare original vs optimized)
# □ Benchmark computation time on target hardware
# □ Compare lap times: original vs hybrid optimization


# ============================================================================
# DEBUGGING TIPS
# ============================================================================
#
# If clothoid optimization fails:
# 1. Check if raceline_interp has sufficient points (>50)
# 2. Verify kappa_opt is not all zeros
# 3. Check max_curvature parameter is not too restrictive
# 4. Increase max_iterations in minimize_curvature_with_clothoid
#
# If optimization is too slow:
# 1. Reduce num_iterations in multipass optimization
# 2. Skip clothoid smoothing if not critical
# 3. Increase stepsize_interp_after_opt to reduce raceline points
#
# If results are poor:
# 1. Check vehicle parameters (curvlim, width, wheelbase)
# 2. Verify track boundaries are valid
# 3. Compare with single-pass mintime optimization


print(__doc__)
