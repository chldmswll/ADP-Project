import numpy as np
import time
import json
import os
from typing import Tuple
import trajectory_planning_helpers as tph
import matplotlib.pyplot as plt
import configparser
from global_racetrajectory_optimization import opt_mintime_traj, helper_funcs_glob

"""
Created by:
Luca Schwarzenbach

Documentation:
This is an adaption of the main_globaltraj.py file which allows to call a function to optimize a trajectory and returns
the optimized trajectory, the track boundaries, and the lap time.
"""        
        
def trajectory_optimizer(input_path: str,
                         track_name: str,
                         curv_opt_type: str,
                         safety_width: float = 0.8,
                         plot: bool = False,
                         veh_params_file: str = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Optimizes a trajectory based on the given input parameters.

    Parameters:
    - input_path (str): The path to the directory with the input files.
    - track_name (str): The name of the track.
    - curv_opt_type (str): The type of curvature optimization to perform. Valid options are "shortest_path", "mincurv",
                                                    "mincurv_iqp", and "mintime".
    - safety_width (float): The safety width for the trajectory. Default is 0.8.
    - plot (bool): Whether to plot the optimized trajectory. Default is False.

    Returns:
    - Tuple[np.ndarray, np.ndarray, np.ndarray, float]: A tuple containing the optimized trajectory, the track boundaries, and the lap time.

    Raises:
    - IOError: If the optimization type is unknown.
    - ValueError: If the vehicle parameter file does not exist or is empty.

    """
    # debug and plot options -------------------------------------------------------------------------------------------
    debug = True                                    # print console messages
    plot_opts = {"mincurv_curv_lin": False,         # plot curv. linearization (original and solution based, mincurv only)
                 "raceline": True,                  # plot optimized path
                 "imported_bounds": False,          # plot imported bounds (analyze difference to interpolated bounds)
                 "raceline_curv": True,             # plot curvature profile of optimized path
                 "racetraj_vel": True,              # plot velocity profile
                 "racetraj_vel_3d": True,          # plot 3D velocity profile above raceline
                 "racetraj_vel_3d_stepsize": 0.2,   # [m] vertical lines stepsize in 3D velocity profile plot
                 "spline_normals": False,           # plot spline normals to check for crossings
                 "mintime_plots": False}            # plot states, controls, friction coeffs etc. (mintime only)
    
    # vehicle parameter file -------------------------------------------------------------------------------------------
    if veh_params_file is None:
        veh_params_file = "racecar_f110.ini"
    file_paths = {"veh_params_file": veh_params_file}

    # track file (including centerline coordinates + track widths) -----------------------------------------------------
    file_paths["track_name"] = track_name

    # set import options -----------------------------------------------------------------------------------------------
    imp_opts = {"flip_imp_track": False,                # flip imported track to reverse direction
                "set_new_start": False,                 # set new starting point (changes order, not coordinates)
                "new_start": np.array([-2.4, 0.0]),    # [x_m, y_m]
                "min_track_width": None,                # [m] minimum enforced track width (set None to deactivate)
                "num_laps": 1}                          # number of laps to be driven (significant with powertrain-option),
                                                        # only relevant in mintime-optimization

    # set mintime specific options (mintime only) ----------------------------------------------------------------------
    # tpadata:                      set individual friction map data file if desired (e.g. for varmue maps),
    #                               else set None, e.g. "berlin_2018_varmue08-12_tpadata.json"
    # warm_start:                   [True/False] warm start IPOPT if previous result is available for current track
    # var_friction:                 [-] None, "linear", "gauss" -> set if variable friction coefficients should be used
    #                               either with linear regression or with gaussian basis functions
    #                               (requires friction map)
    # reopt_mintime_solution:       reoptimization of the mintime solution by min. curv. for improved curv. smoothness
    # recalc_vel_profile_by_tph:    override mintime velocity profile by ggv based calculation (see TPH package)

    mintime_opts = {"tpadata": None,
                    "warm_start": False,
                    "var_friction": None,
                    "reopt_mintime_solution": False,
                    "recalc_vel_profile_by_tph": False}
    
    # lap time calculation table ---------------------------------------------------------------------------------------
    lap_time_mat_opts = {"use_lap_time_mat": False,             # calculate a lap time matrix (top speeds and scales)
                         "gg_scale_range": [0.3, 1.0],          # range of gg scales to be covered
                         "gg_scale_stepsize": 0.05,             # step size to be applied
                         "top_speed_range": [100.0, 150.0],     # range of top speeds to be simulated [in km/h]
                         "top_speed_stepsize": 5.0,             # step size to be applied
                         "file": "lap_time_matrix.csv"}         # file name of the lap time matrix (stored in "outputs")

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK USER INPUT -------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if curv_opt_type not in ["shortest_path", "mincurv", "mincurv_iqp", "mintime"]:
        raise IOError("Unknown optimization type!")

    # ------------------------------------------------------------------------------------------------------------------
    # INITIALIZATION OF PATHS ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    
    # input file path --------------------------------------------------------------------------------------------------
    file_paths["inputs"] = input_path
    
    # get current path
    file_paths["module"] = os.path.dirname(os.path.abspath(__file__))

    # assemble track import path
    # Check if track_name is already a full path
    if os.path.isabs(file_paths["track_name"]):
        # Full path provided, check if it exists
        if os.path.exists(file_paths["track_name"]):
            file_paths["track_file"] = file_paths["track_name"]
        elif os.path.exists(file_paths["track_name"] + ".csv"):
            # Try adding .csv extension
            file_paths["track_file"] = file_paths["track_name"] + ".csv"
        else:
            raise IOError(f"Track file not found: {file_paths['track_name']} or {file_paths['track_name']}.csv")
    elif os.path.exists(file_paths["track_name"]):
        # Relative path that exists, use as-is
        file_paths["track_file"] = os.path.abspath(file_paths["track_name"])
    elif os.path.exists(file_paths["track_name"] + ".csv"):
        # Relative path with .csv extension
        file_paths["track_file"] = os.path.abspath(file_paths["track_name"] + ".csv")
    else:
        # Try input_path/tracks/track_name.csv first, then fall back to module/inputs/tracks/track_name.csv
        track_file_candidate1 = os.path.join(file_paths["inputs"], "tracks", file_paths["track_name"] + ".csv")
        track_file_candidate2 = os.path.join(file_paths["module"], "inputs", "tracks", file_paths["track_name"] + ".csv")
        if os.path.exists(track_file_candidate1):
            file_paths["track_file"] = track_file_candidate1
        elif os.path.exists(track_file_candidate2):
            file_paths["track_file"] = track_file_candidate2
        else:
            # Fallback to just track_name.csv (for backward compatibility)
            file_paths["track_file"] = file_paths["track_name"] + ".csv"

    # assemble friction map import paths
    file_paths["tpamap"] = os.path.join(file_paths["inputs"], "frictionmaps",
                                        file_paths["track_name"] + "_tpamap.csv")

    if mintime_opts["tpadata"] is None:
        file_paths["tpadata"] = os.path.join(file_paths["inputs"], "frictionmaps",
                                             file_paths["track_name"] + "_tpadata.json")
    else:
        file_paths["tpadata"] = os.path.join(file_paths["inputs"], "frictionmaps", mintime_opts["tpadata"])

    # check if friction map files are existing if the var_friction option was set
    if curv_opt_type == 'mintime' \
            and mintime_opts["var_friction"] is not None \
            and not (os.path.exists(file_paths["tpadata"]) and os.path.exists(file_paths["tpamap"])):

        mintime_opts["var_friction"] = None
        print("WARNING: var_friction option is not None but friction map data is missing for current track -> Setting"
              " var_friction to None!")
        
    # create outputs folder(s)
    os.makedirs(file_paths["module"] + "/outputs", exist_ok=True)

    if curv_opt_type == 'mintime':
        os.makedirs(file_paths["module"] + "/outputs/mintime", exist_ok=True)

    # assemble export paths
    file_paths["traj_race_export"] = os.path.join(file_paths["module"], "outputs", "traj_race_cl.csv")
    file_paths["mintime_export"] = os.path.join(file_paths["module"], "outputs", "mintime")

    # file_paths["traj_ltpl_export"] = os.path.join(file_paths["module"], "outputs", "traj_ltpl_cl.csv")
    file_paths["lap_time_mat_export"] = os.path.join(file_paths["module"], "outputs", lap_time_mat_opts["file"])

    # ------------------------------------------------------------------------------------------------------------------
    # IMPORT VEHICLE DEPENDENT PARAMETERS ------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # load vehicle parameter file into a "pars" dict
    parser = configparser.ConfigParser()
    pars = {}

    # Determine config file path
    veh_params_file = file_paths["veh_params_file"]
    if os.path.isabs(veh_params_file):
        # Absolute path provided, use as-is
        config_file_path = veh_params_file
    else:
        # Relative path: try params/ directory first, then inputs/ directory
        config_file_path = os.path.join(file_paths["module"], "params", veh_params_file)
        if not os.path.exists(config_file_path):
            config_file_path = os.path.join(file_paths["inputs"], veh_params_file)

    if not parser.read(config_file_path):
        raise ValueError(f'Specified config file does not exist or is empty! Tried: {config_file_path}')

    pars["ggv_file"] = json.loads(parser.get('GENERAL_OPTIONS', 'ggv_file'))
    pars["ax_max_machines_file"] = json.loads(parser.get('GENERAL_OPTIONS', 'ax_max_machines_file'))
    pars["stepsize_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'stepsize_opts'))
    pars["reg_smooth_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'reg_smooth_opts'))
    pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
    pars["vel_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts'))

    if curv_opt_type == 'shortest_path':
        pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_shortest_path'))

    elif curv_opt_type in ['mincurv', 'mincurv_iqp']:
        pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mincurv'))

    elif curv_opt_type == 'mintime':
        pars["curv_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'curv_calc_opts'))
        pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))
        pars["vehicle_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
        pars["tire_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'tire_params_mintime'))
        pars["pwr_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'pwr_params_mintime'))

        # modification of mintime options/parameters
        pars["optim_opts"]["var_friction"] = mintime_opts["var_friction"]
        pars["optim_opts"]["warm_start"] = mintime_opts["warm_start"]
        pars["vehicle_params_mintime"]["wheelbase"] = (pars["vehicle_params_mintime"]["wheelbase_front"]
                                                       + pars["vehicle_params_mintime"]["wheelbase_rear"])

    # set import path for ggv diagram and ax_max_machines (if required)
    if not (curv_opt_type == 'mintime' and not mintime_opts["recalc_vel_profile_by_tph"]):
        file_paths["ggv_file"] = os.path.join(file_paths["inputs"], "veh_dyn_info", pars["ggv_file"])
        file_paths["ax_max_machines_file"] = os.path.join(file_paths["inputs"], "veh_dyn_info",
                                                          pars["ax_max_machines_file"])

    # ------------------------------------------------------------------------------------------------------------------
    # IMPORT TRACK AND VEHICLE DYNAMICS INFORMATION --------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # save start time
    t_start = time.perf_counter()

    # import track
    reftrack_imp = helper_funcs_glob.src.import_track.import_track(imp_opts=imp_opts,
                                                                   file_path=file_paths["track_file"],
                                                                   width_veh=pars["veh_params"]["width"])
    
    # Validate imported track
    if reftrack_imp is None or reftrack_imp.size == 0 or len(reftrack_imp.shape) < 2 or reftrack_imp.shape[0] == 0:
        raise ValueError(f"Imported track is empty or invalid. Track file: {file_paths['track_file']}")
    
    # Check if track has valid width data (columns 2 and 3 should contain track widths)
    if reftrack_imp.shape[1] < 4:
        raise ValueError(f"Track data has insufficient columns. Expected at least 4 columns (x, y, width_right, width_left), got {reftrack_imp.shape[1]}")
    
    # Ensure reftrack_imp is 2D array with correct shape
    if len(reftrack_imp.shape) != 2:
        reftrack_imp = reftrack_imp.reshape(-1, 4)
    
    # Check for minimum track width
    if np.any(reftrack_imp[:, 2] <= 0) or np.any(reftrack_imp[:, 3] <= 0):
        min_width_right = np.min(reftrack_imp[:, 2])
        min_width_left = np.min(reftrack_imp[:, 3])
        print(f"WARNING: Some track widths are zero or negative. Min width_right: {min_width_right}, Min width_left: {min_width_left}")
        # Set minimum width to safety_width
        reftrack_imp[:, 2] = np.maximum(reftrack_imp[:, 2], safety_width)
        reftrack_imp[:, 3] = np.maximum(reftrack_imp[:, 3], safety_width)
    
    # Ensure all values are finite (no NaN or Inf)
    if not np.all(np.isfinite(reftrack_imp)):
        raise ValueError(f"Track data contains NaN or Inf values. Track file: {file_paths['track_file']}")
    
    print(f"DEBUG: Imported track shape: {reftrack_imp.shape}, first point: {reftrack_imp[0]}, last point: {reftrack_imp[-1]}")

    # import ggv and ax_max_machines (if required)
    if not (curv_opt_type == 'mintime' and not mintime_opts["recalc_vel_profile_by_tph"]):
        ggv, ax_max_machines = tph.import_veh_dyn_info.\
            import_veh_dyn_info(ggv_import_path=file_paths["ggv_file"],
                                ax_max_machines_import_path=file_paths["ax_max_machines_file"])
    else:
        ggv = None
        ax_max_machines = None

    # set ax_pos_safe / ax_neg_safe / ay_safe if required and not set in parameters file
    if curv_opt_type == 'mintime' and pars["optim_opts"]["safe_traj"] \
            and (pars["optim_opts"]["ax_pos_safe"] is None
                 or pars["optim_opts"]["ax_neg_safe"] is None
                 or pars["optim_opts"]["ay_safe"] is None):

        # get ggv if not available
        if ggv is None:
            ggv = tph.import_veh_dyn_info. \
                import_veh_dyn_info(ggv_import_path=file_paths["ggv_file"],
                                    ax_max_machines_import_path=file_paths["ax_max_machines_file"])[0]

        # limit accelerations
        if pars["optim_opts"]["ax_pos_safe"] is None:
            pars["optim_opts"]["ax_pos_safe"] = np.amin(ggv[:, 1])
        if pars["optim_opts"]["ax_neg_safe"] is None:
            pars["optim_opts"]["ax_neg_safe"] = -np.amin(ggv[:, 1])
        if pars["optim_opts"]["ay_safe"] is None:
            pars["optim_opts"]["ay_safe"] = np.amin(ggv[:, 2])

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARE REFTRACK -------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # Basic validation - detailed validation is done in prep_track
    if reftrack_imp is None or reftrack_imp.size == 0:
        raise ValueError(f"reftrack_imp is empty. Track file: {file_paths['track_file']}")
    
    print(f"DEBUG: Calling prep_track with reftrack_imp shape: {reftrack_imp.shape}, dtype: {reftrack_imp.dtype}")
    
    try:
        reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = \
            helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=reftrack_imp,
                                                        reg_smooth_opts=pars["reg_smooth_opts"],
                                                        stepsize_opts=pars["stepsize_opts"],
                                                        debug=debug,
                                                        min_width=imp_opts["min_track_width"])
    except Exception as e:
        raise RuntimeError(f"prep_track failed: {e}") from e
    
    # Validate prep_track results
    if reftrack_interp is None or reftrack_interp.size == 0 or reftrack_interp.shape[0] == 0:
        raise ValueError(f"prep_track returned empty track. Input track had {reftrack_imp.shape[0]} points. This may indicate track widths are too small or track data is invalid.")
    if normvec_normalized_interp is None or normvec_normalized_interp.size == 0 or normvec_normalized_interp.shape[0] == 0:
        raise ValueError(f"prep_track returned empty normvec_normalized_interp.")
    if a_interp is None or a_interp.size == 0 or a_interp.shape[0] == 0:
        raise ValueError(f"prep_track returned empty a_interp.")
    
    print(f"DEBUG: prep_track successful. reftrack_interp shape: {reftrack_interp.shape}, normvec shape: {normvec_normalized_interp.shape}")

    # ------------------------------------------------------------------------------------------------------------------
    # CALL OPTIMIZATION ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # if reoptimization of mintime solution is used afterwards we have to consider some additional deviation in the first
    # optimization
    # if curv_opt_type == 'mintime' and mintime_opts["reopt_mintime_solution"]:
    #     w_veh_tmp = pars["optim_opts"]["width_opt"] + (pars["optim_opts"]["w_tr_reopt"] - pars["optim_opts"]["w_veh_reopt"])
    #     w_veh_tmp += pars["optim_opts"]["w_add_spl_regr"]
    #     pars_tmp = copy.deepcopy(pars)
    #     pars_tmp["optim_opts"]["width_opt"] = w_veh_tmp
    # else:
    pars_tmp = pars

    # call optimization
    if curv_opt_type == 'mincurv':
        alpha_opt = tph.opt_min_curv.opt_min_curv(reftrack=reftrack_interp,
                                                  normvectors=normvec_normalized_interp,
                                                  A=a_interp,
                                                  kappa_bound=pars["veh_params"]["curvlim"],
                                                  w_veh=safety_width,
                                                  print_debug=debug,
                                                  plot_debug=False)[0]
        # Ensure alpha_opt is 1D
        if len(alpha_opt.shape) > 1:
            alpha_opt = alpha_opt.flatten()
        print(f"DEBUG: mincurv successful. alpha_opt shape: {alpha_opt.shape}, reftrack_interp shape: {reftrack_interp.shape}")

    elif curv_opt_type == 'mincurv_iqp':
        alpha_opt, reftrack_interp, normvec_normalized_interp, _, _, _, _ = tph.iqp_handler.\
            iqp_handler(reftrack=reftrack_interp,
                        normvectors=normvec_normalized_interp,
                        A=a_interp,
                        spline_len=np.zeros(reftrack_interp.shape[0]),
                        psi=np.zeros(reftrack_interp.shape[0]),
                        kappa=np.zeros(reftrack_interp.shape[0]),
                        dkappa=np.zeros(reftrack_interp.shape[0]),
                        kappa_bound=pars["veh_params"]["curvlim"],
                        w_veh=safety_width,
                        print_debug=debug,
                        plot_debug=plot_opts["mincurv_curv_lin"],
                        stepsize_interp=pars["stepsize_opts"]["stepsize_reg"],
                        iters_min=pars["optim_opts"]["iqp_iters_min"],
                        curv_error_allowed=pars["optim_opts"]["iqp_curverror_allowed"])
        
        # Validate iqp_handler results
        if alpha_opt is None or alpha_opt.size == 0:
            raise ValueError(f"iqp_handler returned empty alpha_opt. This may indicate optimization failed due to invalid track data.")
        if reftrack_interp is None or reftrack_interp.size == 0 or reftrack_interp.shape[0] == 0:
            raise ValueError(f"iqp_handler returned empty reftrack_interp.")
        # Ensure alpha_opt is 1D
        if len(alpha_opt.shape) > 1:
            alpha_opt = alpha_opt.flatten()
        print(f"DEBUG: iqp_handler successful. alpha_opt shape: {alpha_opt.shape}, reftrack_interp shape: {reftrack_interp.shape}")

    elif curv_opt_type == 'shortest_path':
        alpha_opt = tph.opt_shortest_path.opt_shortest_path(reftrack=reftrack_interp,
                                                            normvectors=normvec_normalized_interp,
                                                            w_veh=safety_width,
                                                            print_debug=debug)
        # Ensure alpha_opt is 1D
        if len(alpha_opt.shape) > 1:
            alpha_opt = alpha_opt.flatten()
        half_window = 3
        window = 2*half_window + 1
        print(f"DEBUG: shortest_path alpha_opt shape before smoothing: {alpha_opt.shape}")
        alpha_opt = np.concatenate((alpha_opt[-half_window:], alpha_opt, alpha_opt[:half_window]), axis = None)
        alpha_opt = np.convolve(alpha_opt, np.ones(window), 'valid')/window
        print(f"DEBUG: shortest_path alpha_opt shape after smoothing: {alpha_opt.shape}")

    elif curv_opt_type == 'mintime':
        # reftrack_interp, a_interp and normvec_normalized_interp are returned for the case that non-regular sampling
        # was applied
        alpha_opt, v_opt, reftrack_interp, a_interp_tmp, normvec_normalized_interp = opt_mintime_traj.src.opt_mintime.\
            opt_mintime(reftrack=reftrack_interp,
                        coeffs_x=coeffs_x_interp,
                        coeffs_y=coeffs_y_interp,
                        normvectors=normvec_normalized_interp,
                        pars=pars_tmp,
                        tpamap_path=file_paths["tpamap"],
                        tpadata_path=file_paths["tpadata"],
                        export_path=file_paths["mintime_export"],
                        print_debug=debug,
                        plot_debug=plot_opts["mintime_plots"])

        # replace a_interp if necessary
        if a_interp_tmp is not None:
            a_interp = a_interp_tmp

    else:
        raise IOError('Unknown optimization type!')
        # alpha_opt = np.zeros(reftrack_interp.shape[0])

    # ------------------------------------------------------------------------------------------------------------------
    # REOPTIMIZATION OF THE MINTIME SOLUTION ---------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if curv_opt_type == 'mintime' and mintime_opts["reopt_mintime_solution"]:

        # get raceline solution of the time-optimal trajectory
        raceline_mintime = reftrack_interp[:, :2] + np.expand_dims(alpha_opt, 1) * normvec_normalized_interp

        # calculate new track boundaries around raceline solution depending on alpha_opt values
        w_tr_right_mintime = reftrack_interp[:, 2] - alpha_opt
        w_tr_left_mintime = reftrack_interp[:, 3] + alpha_opt

        # create new reference track around the raceline
        racetrack_mintime = np.column_stack((raceline_mintime, w_tr_right_mintime, w_tr_left_mintime))

        # use spline approximation a second time
        reftrack_interp, normvec_normalized_interp, a_interp = \
            helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=racetrack_mintime,
                                                        reg_smooth_opts=pars["reg_smooth_opts"],
                                                        stepsize_opts=pars["stepsize_opts"],
                                                        debug=False,
                                                        min_width=imp_opts["min_track_width"])[:3]

        # set artificial track widths for reoptimization
        w_tr_tmp = 0.5 * pars["optim_opts"]["w_tr_reopt"] * np.ones(reftrack_interp.shape[0])
        racetrack_mintime_reopt = np.column_stack((reftrack_interp[:, :2], w_tr_tmp, w_tr_tmp))

        # call mincurv reoptimization
        alpha_opt = tph.opt_min_curv.opt_min_curv(reftrack=racetrack_mintime_reopt,
                                                  normvectors=normvec_normalized_interp,
                                                  A=a_interp,
                                                  kappa_bound=pars["veh_params"]["curvlim"],
                                                  w_veh=pars["optim_opts"]["w_veh_reopt"],
                                                  print_debug=debug,
                                                  plot_debug=plot_opts["mincurv_curv_lin"])[0]

        # calculate minimum distance from raceline to bounds and print it
        if debug:
            raceline_reopt = reftrack_interp[:, :2] + np.expand_dims(alpha_opt, 1) * normvec_normalized_interp
            bound_r_reopt = (reftrack_interp[:, :2]
                             + np.expand_dims(reftrack_interp[:, 2], axis=1) * normvec_normalized_interp)
            bound_l_reopt = (reftrack_interp[:, :2]
                             - np.expand_dims(reftrack_interp[:, 3], axis=1) * normvec_normalized_interp)

            d_r_reopt = np.hypot(raceline_reopt[:, 0] - bound_r_reopt[:, 0], raceline_reopt[:, 1] - bound_r_reopt[:, 1])
            d_l_reopt = np.hypot(raceline_reopt[:, 0] - bound_l_reopt[:, 0], raceline_reopt[:, 1] - bound_l_reopt[:, 1])

            print("INFO: Mintime reoptimization: minimum distance to right/left bound: %.2fm / %.2fm"
                  % (np.amin(d_r_reopt) - pars["veh_params"]["width"] / 2,
                     np.amin(d_l_reopt) - pars["veh_params"]["width"] / 2))

    # ------------------------------------------------------------------------------------------------------------------
    # INTERPOLATE SPLINES TO SMALL DISTANCES BETWEEN RACELINE POINTS ---------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # Ensure alpha_opt is 1D before calling create_raceline
    if len(alpha_opt.shape) > 1:
        alpha_opt = alpha_opt.flatten()
    if alpha_opt.shape[0] != reftrack_interp.shape[0]:
        raise ValueError(f"alpha_opt length ({alpha_opt.shape[0]}) does not match reftrack_interp length ({reftrack_interp.shape[0]})")
    
    # Validate inputs to create_raceline
    if len(reftrack_interp.shape) != 2 or reftrack_interp.shape[1] < 2:
        raise ValueError(f"reftrack_interp has invalid shape: {reftrack_interp.shape}. Expected (N, >=2)")
    if len(normvec_normalized_interp.shape) != 2 or normvec_normalized_interp.shape[1] != 2:
        raise ValueError(f"normvec_normalized_interp has invalid shape: {normvec_normalized_interp.shape}. Expected (N, 2)")
    if len(alpha_opt.shape) != 1:
        raise ValueError(f"alpha_opt has invalid shape: {alpha_opt.shape}. Expected 1D array")
    
    print(f"DEBUG: Calling create_raceline with refline shape: {reftrack_interp[:, :2].shape}, normvectors shape: {normvec_normalized_interp.shape}, alpha shape: {alpha_opt.shape}")
    
    try:
        raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, t_vals_opt_interp, s_points_opt_interp,\
            spline_lengths_opt, el_lengths_opt_interp = tph.create_raceline.\
            create_raceline(refline=reftrack_interp[:, :2],
                            normvectors=normvec_normalized_interp,
                            alpha=alpha_opt,
                            stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"])
        print(f"DEBUG: create_raceline successful. raceline_interp shape: {raceline_interp.shape}")
    except Exception as e:
        raise RuntimeError(f"create_raceline failed: {e}. Input shapes - refline: {reftrack_interp[:, :2].shape}, normvectors: {normvec_normalized_interp.shape}, alpha: {alpha_opt.shape}") from e

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE HEADING AND CURVATURE ----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate heading and curvature (analytically)
    # Validate inputs
    if coeffs_x_opt is None or coeffs_x_opt.size == 0:
        raise ValueError(f"coeffs_x_opt is empty. shape: {coeffs_x_opt.shape if coeffs_x_opt is not None else 'None'}")
    if coeffs_y_opt is None or coeffs_y_opt.size == 0:
        raise ValueError(f"coeffs_y_opt is empty. shape: {coeffs_y_opt.shape if coeffs_y_opt is not None else 'None'}")
    
    try:
        psi_vel_opt, kappa_opt = tph.calc_head_curv_an.\
            calc_head_curv_an(coeffs_x=coeffs_x_opt,
                              coeffs_y=coeffs_y_opt,
                              ind_spls=spline_inds_opt_interp,
                              t_spls=t_vals_opt_interp)
        print(f"DEBUG: calc_head_curv_an successful. psi_vel_opt shape: {psi_vel_opt.shape}, kappa_opt shape: {kappa_opt.shape}")
    except Exception as e:
        raise RuntimeError(f"calc_head_curv_an failed: {e}") from e

    # ------------------------------------------------------------------------------------------------------------------
    # APPLY CLOTHOID CURVE OPTIMIZATION FOR MINIMUM CURVATURE ----------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    
    # Apply clothoid curve smoothing to minimize curvature for better vehicle drivability
    print("INFO: Applying clothoid curve optimization to minimize curvature...")
    
    try:
        from global_racetrajectory_optimization.helper_funcs_glob.src import clothoid_spline
        minimize_curvature_with_clothoid = clothoid_spline.minimize_curvature_with_clothoid
        
        # Store original values for comparison
        kappa_opt_original = kappa_opt.copy()
        
        # Minimize curvature using clothoid curves
        raceline_interp_clothoid, kappa_opt_clothoid = minimize_curvature_with_clothoid(
            raceline_points=raceline_interp,
            kappa_profile=kappa_opt,
            max_iterations=5,
            tolerance=1e-5,
            max_curvature=pars["veh_params"]["curvlim"],
            debug=True
        )
        
        # Recalculate spline coefficients from clothoid-optimized raceline
        raceline_closed = np.vstack((raceline_interp_clothoid, raceline_interp_clothoid[0]))
        coeffs_x_opt_clothoid, coeffs_y_opt_clothoid, a_opt_clothoid, _ = tph.calc_splines.calc_splines(path=raceline_closed)
        
        # Regenerate raceline with clothoid-optimized path using create_raceline
        raceline_interp_new, a_opt_new, coeffs_x_opt_new, coeffs_y_opt_new, spline_inds_opt_interp_new, \
            t_vals_opt_interp_new, s_points_opt_interp_new, spline_lengths_opt_new, el_lengths_opt_interp_new = \
            tph.create_raceline.create_raceline(refline=raceline_interp_clothoid,
                                               normvectors=normvec_normalized_interp,
                                               alpha=np.zeros(len(raceline_interp_clothoid)),  # No lateral offset needed
                                               stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"])
        
        # Recalculate heading and curvature with optimized splines
        psi_vel_opt_new, kappa_opt_new = tph.calc_head_curv_an.\
            calc_head_curv_an(coeffs_x=coeffs_x_opt_new,
                              coeffs_y=coeffs_y_opt_new,
                              ind_spls=spline_inds_opt_interp_new,
                              t_spls=t_vals_opt_interp_new)
        
        # Use optimized values if curvature is reduced
        if np.max(np.abs(kappa_opt_new)) < np.max(np.abs(kappa_opt_original)):
            raceline_interp = raceline_interp_new
            coeffs_x_opt = coeffs_x_opt_new
            coeffs_y_opt = coeffs_y_opt_new
            a_opt = a_opt_new
            spline_inds_opt_interp = spline_inds_opt_interp_new
            t_vals_opt_interp = t_vals_opt_interp_new
            s_points_opt_interp = s_points_opt_interp_new
            spline_lengths_opt = spline_lengths_opt_new
            el_lengths_opt_interp = el_lengths_opt_interp_new
            psi_vel_opt = psi_vel_opt_new
            kappa_opt = kappa_opt_new
            
            print(f"INFO: Clothoid optimization successful. Max curvature reduced from "
                  f"{np.max(np.abs(kappa_opt_original)):.6f} to {np.max(np.abs(kappa_opt)):.6f} rad/m")
        else:
            print(f"INFO: Clothoid optimization did not reduce curvature. Using original spline.")
    
    except ImportError as e:
        print(f"WARNING: Could not import clothoid_spline module: {e}")
        print("WARNING: Continuing without clothoid optimization")
    except Exception as e:
        print(f"WARNING: Clothoid optimization failed: {e}")
        print("WARNING: Continuing with original spline")

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE VELOCITY AND ACCELERATION PROFILE ----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if curv_opt_type == 'mintime' and not mintime_opts["recalc_vel_profile_by_tph"]:
        # interpolation
        s_splines = np.cumsum(spline_lengths_opt)
        s_splines = np.insert(s_splines, 0, 0.0)
        if len(s_splines[:-1]) != len(v_opt):
            n_steps = s_splines[-1] / len(v_opt)
            s_splines_copy = np.ones(len(v_opt) + 1)
            for i, v in enumerate(v_opt):
                s_splines_copy[i] = i * n_steps
            s_splines = s_splines_copy

        print(f"Length s_splines: {len(s_splines[:-1])}, v_opt: {len(v_opt)}")
        print(f"s_splines: {s_splines[:-1]}, v_opt: {v_opt}")
        vx_profile_opt = np.interp(s_points_opt_interp, s_splines[:-1], v_opt)

    else:
        vx_profile_opt = tph.calc_vel_profile.\
            calc_vel_profile(ggv=ggv,
                             ax_max_machines=ax_max_machines,
                             v_max=pars["veh_params"]["v_max"],
                             kappa=kappa_opt,
                             el_lengths=el_lengths_opt_interp,
                             closed=True,
                             filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
                             dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
                             drag_coeff=pars["veh_params"]["dragcoeff"],
                             m_veh=pars["veh_params"]["mass"])

    # calculate longitudinal acceleration profile
    vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
    ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                         el_lengths=el_lengths_opt_interp,
                                                         eq_length_output=False)

    # calculate laptime
    t_profile_cl = tph.calc_t_profile.calc_t_profile(vx_profile=vx_profile_opt,
                                                     ax_profile=ax_profile_opt,
                                                     el_lengths=el_lengths_opt_interp)
    print("INFO: Estimated laptime: %.2fs" % t_profile_cl[-1])

    if plot_opts["racetraj_vel"] and plot:
        s_points = np.cumsum(el_lengths_opt_interp[:-1])
        s_points = np.insert(s_points, 0, 0.0)

        plt.plot(s_points, vx_profile_opt)
        plt.plot(s_points, ax_profile_opt)
        plt.plot(s_points, t_profile_cl[:-1])

        plt.grid()
        plt.xlabel("distance in m")
        plt.legend(["vx in m/s", "ax in m/s2", "t in s"])

        plt.show()

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE LAP TIMES (AT DIFFERENT SCALES AND TOP SPEEDS) ---------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if lap_time_mat_opts["use_lap_time_mat"]:
        # simulate lap times
        ggv_scales = np.linspace(lap_time_mat_opts['gg_scale_range'][0],
                                 lap_time_mat_opts['gg_scale_range'][1],
                                 int((lap_time_mat_opts['gg_scale_range'][1] - lap_time_mat_opts['gg_scale_range'][0])
                                     / lap_time_mat_opts['gg_scale_stepsize']) + 1)
        top_speeds = np.linspace(lap_time_mat_opts['top_speed_range'][0] / 3.6,
                                 lap_time_mat_opts['top_speed_range'][1] / 3.6,
                                 int((lap_time_mat_opts['top_speed_range'][1] - lap_time_mat_opts['top_speed_range'][0])
                                     / lap_time_mat_opts['top_speed_stepsize']) + 1)

        # setup results matrix
        lap_time_matrix = np.zeros((top_speeds.shape[0] + 1, ggv_scales.shape[0] + 1))

        # write parameters in first column and row
        lap_time_matrix[1:, 0] = top_speeds * 3.6
        lap_time_matrix[0, 1:] = ggv_scales

        for i, top_speed in enumerate(top_speeds):
            for j, ggv_scale in enumerate(ggv_scales):
                tph.progressbar.progressbar(i*ggv_scales.shape[0] + j,
                                            top_speeds.shape[0] * ggv_scales.shape[0],
                                            prefix="Simulating laptimes ")

                ggv_mod = np.copy(ggv)
                ggv_mod[:, 1:] *= ggv_scale

                vx_profile_opt = tph.calc_vel_profile.\
                    calc_vel_profile(ggv=ggv_mod,
                                     ax_max_machines=ax_max_machines,
                                     v_max=top_speed,
                                     kappa=kappa_opt,
                                     el_lengths=el_lengths_opt_interp,
                                     dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
                                     filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
                                     closed=True,
                                     drag_coeff=pars["veh_params"]["dragcoeff"],
                                     m_veh=pars["veh_params"]["mass"])

                # calculate longitudinal acceleration profile
                vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
                ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                                     el_lengths=el_lengths_opt_interp,
                                                                     eq_length_output=False)

                # calculate lap time
                t_profile_cl = tph.calc_t_profile.calc_t_profile(vx_profile=vx_profile_opt,
                                                                 ax_profile=ax_profile_opt,
                                                                 el_lengths=el_lengths_opt_interp)

                # store entry in lap time matrix
                lap_time_matrix[i + 1, j + 1] = t_profile_cl[-1]
                
        # store lap time matrix to file
        np.savetxt(file_paths["lap_time_mat_export"], lap_time_matrix, delimiter=",", fmt="%.3f")

    # ------------------------------------------------------------------------------------------------------------------
    # DATA POSTPROCESSING ----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # arrange data into one trajectory
    trajectory_opt = np.column_stack((s_points_opt_interp,
                                      raceline_interp,
                                      psi_vel_opt,
                                      kappa_opt,
                                      vx_profile_opt,
                                      ax_profile_opt))
    spline_data_opt = np.column_stack((spline_lengths_opt, coeffs_x_opt, coeffs_y_opt))

    # create a closed race trajectory array
    traj_race_cl = np.vstack((trajectory_opt, trajectory_opt[0, :]))
    traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

    # print end time
    print("INFO: Runtime from import to final trajectory was %.2fs" % (time.perf_counter() - t_start))

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK TRAJECTORY -------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    bound1, bound2 = helper_funcs_glob.src.check_traj.\
        check_traj(reftrack=reftrack_interp,
                   reftrack_normvec_normalized=normvec_normalized_interp,
                   length_veh=pars["veh_params"]["length"],
                   width_veh=pars["veh_params"]["width"],
                   debug=debug,
                   trajectory=trajectory_opt,
                   ggv=ggv,
                   ax_max_machines=ax_max_machines,
                   v_max=pars["veh_params"]["v_max"],
                   curvlim=pars["veh_params"]["curvlim"],
                   mass_veh=pars["veh_params"]["mass"],
                   dragcoeff=pars["veh_params"]["dragcoeff"])

    # ------------------------------------------------------------------------------------------------------------------
    # PLOT RESULTS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if plot:
        # get bound of imported map (for reference in final plot)
        bound1_imp = None
        bound2_imp = None

        if plot_opts["imported_bounds"]:
            # try to extract four times as many points as in the interpolated version (in order to hold more details)
            n_skip = max(int(reftrack_imp.shape[0] / (bound1.shape[0] * 4)), 1)

            _, _, _, normvec_imp = tph.calc_splines.calc_splines(path=np.vstack((reftrack_imp[::n_skip, 0:2],
                                                                                 reftrack_imp[0, 0:2])))

            bound1_imp = reftrack_imp[::n_skip, :2] + normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 2], 1)
            bound2_imp = reftrack_imp[::n_skip, :2] - normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 3], 1)

        # plot results
        helper_funcs_glob.src.result_plots.result_plots(plot_opts=plot_opts,
                                                        width_veh_opt=pars["optim_opts"]["width_opt"],
                                                        width_veh_real=pars["veh_params"]["width"],
                                                        refline=reftrack_interp[:, :2],
                                                        bound1_imp=bound1_imp,
                                                        bound2_imp=bound2_imp,
                                                        bound1_interp=bound1,
                                                        bound2_interp=bound2,
                                                        trajectory=trajectory_opt)

    return traj_race_cl, bound1, bound2, t_profile_cl[-1]  # also return estimated lap time
