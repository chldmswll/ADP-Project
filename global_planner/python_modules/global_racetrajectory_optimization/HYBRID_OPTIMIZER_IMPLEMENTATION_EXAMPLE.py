"""
실제 구현 예제: main_globaltraj.py에 hybrid optimizer 적용

이 파일은 main_globaltraj.py의 기존 코드를 수정하는 방법을 보여줍니다.
"""

# ============================================================================
# LOCATION: main_globaltraj.py, 사용자 입력 부분 (라인 ~60)
# ============================================================================

# 기존 코드:
# opt_type = 'mintime'

# 수정된 코드 - 옵션 1: 하이브리드 옵션 추가
opt_type = 'mintime'

# 하이브리드 최적화 활성화 여부
use_hybrid_optimization = True
use_multipass = False  # 여러 패스 반복 여부
multipass_iterations = 2  # 반복 횟수


# ============================================================================
# LOCATION: main_globaltraj.py, 최적화 호출 부분 (라인 ~278)
# ============================================================================

# 기존 코드 (라인 278-310):
# elif opt_type == 'mintime':
#     alpha_opt, v_opt, reftrack_interp, a_interp_tmp, normvec_normalized_interp = opt_mintime_traj.src.opt_mintime.\
#         opt_mintime(reftrack=reftrack_interp,
#                     coeffs_x=coeffs_x_interp,
#                     coeffs_y=coeffs_y_interp,
#                     normvectors=normvec_normalized_interp,
#                     pars=pars_tmp,
#                     tpamap_path=file_paths["tpamap"],
#                     tpadata_path=file_paths["tpadata"],
#                     export_path=file_paths["mintime_export"],
#                     print_debug=debug,
#                     plot_debug=plot_opts["mintime_plots"])


# 수정된 코드:
elif opt_type == 'mintime':
    if use_hybrid_optimization:
        # ========== 하이브리드 최적화 ==========
        from helper_funcs_glob.src import hybrid_optimizer
        
        if debug:
            print("\n" + "="*80)
            print("USING HYBRID OPTIMIZATION: mintime → mincurv → clothoid")
            print("="*80 + "\n")
        
        if use_multipass:
            # 방법 1: 다중 패스 최적화
            alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
                hybrid_optimizer.sequential_multi_pass_optimize(
                    reftrack_interp=reftrack_interp,
                    coeffs_x_interp=coeffs_x_interp,
                    coeffs_y_interp=coeffs_y_interp,
                    normvec_normalized_interp=normvec_normalized_interp,
                    a_interp=a_interp,
                    pars=pars_tmp,
                    opt_mintime_traj_module=opt_mintime_traj,
                    file_paths=file_paths,
                    num_iterations=multipass_iterations,
                    debug=debug
                )
        else:
            # 방법 2: 단일 패스 하이브리드 최적화
            alpha_opt, v_opt, raceline_interp, kappa_opt, psi_vel_opt = \
                hybrid_optimizer.hybrid_optimize_trajectory(
                    reftrack_interp=reftrack_interp,
                    coeffs_x_interp=coeffs_x_interp,
                    coeffs_y_interp=coeffs_y_interp,
                    normvec_normalized_interp=normvec_normalized_interp,
                    a_interp=a_interp,
                    pars=pars_tmp,
                    v_opt=np.zeros(reftrack_interp.shape[0]),  # 초기 속도 프로필
                    opt_mintime_traj_module=opt_mintime_traj,
                    file_paths=file_paths,
                    plot_opts=plot_opts,
                    debug=debug
                )
        
        # 필요하면 a_interp 업데이트
        # (hybrid_optimizer에서 reftrack_interp가 변경되었을 수 있음)
        
    else:
        # ========== 기존 mintime 최적화 ==========
        if debug:
            print("\n" + "="*80)
            print("USING STANDARD MINTIME OPTIMIZATION")
            print("="*80 + "\n")
        
        alpha_opt, v_opt, reftrack_interp, a_interp_tmp, normvec_normalized_interp = \
            opt_mintime_traj.src.opt_mintime.\
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
        
        # a_interp 업데이트
        if a_interp_tmp is not None:
            a_interp = a_interp_tmp


# ============================================================================
# LOCATION: main_globaltraj.py, REOPTIMIZATION 섹션 (라인 ~328)
# ============================================================================

# 기존 코드: mintime_opts["reopt_mintime_solution"] 체크
# 하이브리드 옵션을 사용할 때는 이 섹션을 스킵할 수 있음 (이미 clothoid 적용됨)

if opt_type == 'mintime' and mintime_opts["reopt_mintime_solution"] and not use_hybrid_optimization:
    # 기존 mincurv reoptimization 로직
    # ... (기존 코드 유지)
elif use_hybrid_optimization and opt_type == 'mintime':
    if debug:
        print("\nINFO: Skipping separate reoptimization (already done in hybrid optimization)")


# ============================================================================
# LOCATION: main_globaltraj.py, INTERPOLATION 섹션 (라인 ~370)
# ============================================================================

# 하이브리드 최적화 사용 시 처리 방식

if use_hybrid_optimization and opt_type == 'mintime':
    # 하이브리드 옵티마이저에서 이미 raceline_interp, kappa_opt, psi_vel_opt를 반환함
    # 따라서 다음 단계는 스킵하거나 검증 목적으로만 실행
    
    if debug:
        print("\nINFO: Raceline already interpolated and optimized by hybrid_optimizer")
        print(f"      Raceline points: {len(raceline_interp)}")
        print(f"      Max curvature: {np.max(np.abs(kappa_opt)):.6f} rad/m")
else:
    # 기존 interpolation 로직
    raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, \
        t_vals_opt_interp, s_points_opt_interp, spline_lengths_opt, el_lengths_opt_interp = \
        tph.create_raceline.create_raceline(
            refline=reftrack_interp[:, :2],
            normvectors=normvec_normalized_interp,
            alpha=alpha_opt,
            stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"]
        )
    
    # 기존 heading/curvature 계산
    psi_vel_opt, kappa_opt = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x_opt,
        coeffs_y=coeffs_y_opt,
        ind_spls=spline_inds_opt_interp,
        t_spls=t_vals_opt_interp
    )


# ============================================================================
# CLOTHOID 최적화 섹션 (라인 ~400)
# ============================================================================

# 기존 코드: clothoid 별도 적용
# 하이브리드 사용 시: 이미 포함되어 있으므로 스킵

if use_hybrid_optimization and opt_type == 'mintime':
    if debug:
        print("\nINFO: Clothoid optimization already applied in hybrid_optimizer")
        print("      Skipping separate clothoid smoothing step")
else:
    # 기존 clothoid 적용 코드 (라인 391-450)
    # ... (기존 코드 유지)
    try:
        from helper_funcs_glob.src import clothoid_spline
        # ... rest of clothoid code
    except Exception as e:
        if debug:
            print(f"WARNING: Clothoid optimization failed: {e}")


# ============================================================================
# 실행 예제
# ============================================================================

# 터미널에서:
# python main_globaltraj.py

# 또는 trajectory_optimizer.py 사용:
# from trajectory_optimizer import trajectory_optimizer
# traj, bound_r, bound_l, lap_time = trajectory_optimizer(
#     input_path="...",
#     track_name="berlin_2018",
#     curv_opt_type="mintime",  # 여전히 "mintime"을 사용
#     safety_width=0.8,
#     plot=True
# )
# (trajectory_optimizer.py 내부에서 use_hybrid_optimization 옵션을 확인)
