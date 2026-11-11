"""2026-11-11 (Onbeom, #192 E1): 두 프로그램 (P1 평가 · chain45) 의 물리 설정을 같은 형식으로 한 줄씩 찍음 (RL1 조건 2: 물리 맞춤 표).

    from phys_report import report; report("E1a", cfg)
"""


def _g(o, *names):
    for nm in names:
        if o is None:
            return None
        o = getattr(o, nm, None)
    return o


def _mat(m):
    if m is None:
        return "기본"
    return (f"μs {_g(m, 'static_friction')} μd {_g(m, 'dynamic_friction')} 반발 {_g(m, 'restitution')} "
            f"마찰결합 {_g(m, 'friction_combine_mode')} 반발결합 {_g(m, 'restitution_combine_mode')}")


def _col(c):
    if c is None:
        return "기본"
    return f"contact_offset {_g(c, 'contact_offset')} rest_offset {_g(c, 'rest_offset')}"


def _rig(rp):
    if rp is None:
        return "기본"
    return (f"위치 반복 {_g(rp, 'solver_position_iteration_count')} 속도 반복 {_g(rp, 'solver_velocity_iteration_count')} "
            f"max_depen {_g(rp, 'max_depenetration_velocity')} kinematic {_g(rp, 'kinematic_enabled')}")


def report(tag, cfg):
    sim = cfg.sim
    ph = _g(sim, "physics")
    lines = [
        f"sim dt {sim.dt} decimation {cfg.decimation} 바닥 재질 {_mat(_g(sim, 'physics_material'))}",
        "physx " + (" ".join(f"{k} {getattr(ph, k)}" for k in ("solver_type", "min_position_iteration_count", "max_position_iteration_count",
                                                                 "min_velocity_iteration_count", "max_velocity_iteration_count",
                                                                 "bounce_threshold_velocity", "enable_stabilization", "gpu_max_rigid_contact_count")
                              if hasattr(ph, k)) if ph is not None else "없음"),
    ]
    rs = cfg.scene.robot.spawn
    lines.append(f"로봇 관절체 위치 반복 {_g(rs, 'articulation_props', 'solver_position_iteration_count')} 속도 반복 {_g(rs, 'articulation_props', 'solver_velocity_iteration_count')}"
                 f" | 충돌 {_col(_g(rs, 'collision_props'))} | 재질 {_mat(_g(rs, 'physics_material'))}")
    for k in ("boxA", "puckT_box", "puckT_cyl", "puckN"):
        o = getattr(cfg.scene, k, None)
        if o is None:
            continue
        sp = o.spawn
        lines.append(f"{k}: 질량 {_g(sp, 'mass_props', 'mass')} | {_rig(_g(sp, 'rigid_props'))} | 충돌 {_col(_g(sp, 'collision_props'))} | 재질 {_mat(_g(sp, 'physics_material'))}")
    for ln in lines:
        print(f"[Phys {tag}] {ln}", flush=True)
