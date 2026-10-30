"""넣기 전용 `Waffle-Insert-Direct` (Direct RL): 통 위에 맞춘 퍽 (ST3 가 끝나는 상태) 을 **통 안으로 내려 넣고 놓고 집게를 빼기**.

정책_학습기록 4.12 절. 매니퓰레이터 모델 3 번째 단계 (4.11 절). 양옆 통 P·N.
- 시작: sim/isaac/insert_starts.json (Isaac, ST3 맞춤 + 손 IK 내리기 중 기록, `store_hybrid.py --collect`).
  역방향 커리큘럼: 통 위 (입구 위) / 모따기 (입구 −2 ~ +6 mm) / 통 안 (6 mm 넘게) 를 섞어 뽑음 -> 넣은 뒤 놓고 빼기를 먼저 경험.
- 행동 4 = 잡는 점 위치 변화 (base_link x, y, z) x 1 mm + 집게 (> 0 벌림). 위치 변화 -> 감쇠 최소제곱 IK (집게 자세는 시작 자세 유지).
  ST1 ~ ST5 (관절 공간 0.04 rad) 는 흔들림 ≈ 5 mm > 통 틈 1 mm 로 넣기 0 %.
- 관측 (정책, 실제 로봇에서 얻을 수 있는 것만) 35 = 팔 위치 (기준 자세 기준) 5 · 속도 5 · 토크 5 (= 다이나믹셀 전류), 손가락 1, 잡는 점 3,
  통 입구 (축 xy·입구 z, 에피소드마다 ±0.5 mm 보정 오차) 3, 추정 퍽 바닥 (= 잡는 점 − (0, 0, 0.006 + 0.03)) − 통 입구 3, 집게 방향 3, 통 one-hot 3, 이전 행동 4.
- 관측 (평가 critic 만, 시뮬레이션 참값) 12 = 퍽 중심 − 잡는 점 3, 퍽 축 3, 참 퍽 바닥 − 통 입구 3, 손가락·팔 링크·퍽 -몸체 접촉 힘 3.
"""

from __future__ import annotations

from isaaclab.envs import ViewerCfg
from isaaclab.utils import configclass

from .store_env_cfg import StoreSceneCfg, WaffleStoreEnvCfg

INSERT_POSE = {"joint1": 2.80, "joint2": -0.49, "joint3": -0.52, "joint4": 1.01, "joint5": 0.0}  # 261028: 관측 기준값 (store_starts 의 통 위 자세로 갱신)
"""관측의 기준 자세 (통 위 자세 근처)."""


@configclass
class WaffleInsertEnvCfg(WaffleStoreEnvCfg):
    episode_length_s = 6.0  # IN2: 4 -> 6 (끼었다 다시 맞출 시간)
    action_space = 4
    observation_space = 35
    state_space = 0  # 평가 (critic) 참값 12 개는 관측 사전의 'critic' 으로 직접 줌 (리셋 때도 나오게)
    viewer: ViewerCfg = ViewerCfg(eye=(0.25, -0.45, 0.65), lookat=(-0.09, 0.0, 0.42), origin_type="env", env_index=0)
    scene: StoreSceneCfg = StoreSceneCfg(num_envs=256, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)

    cart_scale = 0.001  # [m / 스텝] 잡는 점 위치 변화 최대
    rot_hold_weight = 0.3  # IK 에서 집게 자세 유지 가중치
    ik_damping = 1e-4
    tube_calib_noise = 0.0005  # [m] 관측 통 위치 오차 (에피소드마다)
    start_weight = (0.5, 0.25, 0.25)  # 통 위 / 모따기 / 통 안
    tubes_used = (1, 2)
    # 261028 IN-N2: 새 집게는 손가락 끝이 퍽 바닥 위 16 mm -> 퍽이 13 mm 들어가면 손가락이 통 윗면에 닿아 누르면 퍽이 기울어 끼임 (store_hybrid 15 mm: 0/15, 10 mm: 8/11)
    inserted_depth = 0.008  # '통 안' (이전 0.015)
    depth_goal = 0.010  # 깊이 보상이 다 차는 깊이 (이전 0.020)
    release_depth = 0.006  # '잘 놓음' 깊이 (이전 0.010)

    # 보상 (store 의 접촉 판정·벌점 값은 상속: 팔 링크 −15, 세게 −50 + 종료, 넣는 자리 손가락 −1)
    rew_xy = 2.0  # (1 − tanh(d/0.002)) 퍽 바닥-축 (참값)
    rew_depth = 4.0  # 깊이 (입구 + 5 mm -> 입구 − 20 mm) x (1 − tanh(d/0.004))
    rew_inserted = 1.0  # 통 안 (쥔 채). 261028 IN-N3: 5 -> 1 (N2 는 쥔 채 통 안에 머물며 +5 / 스텝을 받고 안 놓음, 넣기 끝 20 %)
    rew_released = 5.0  # 통 안에서 놓음
    rew_stored = 40.0  # 통 안 깊이 60 mm + 빈손 (IN-N3: 20 -> 40)
    rew_retract = 3.0  # 놓은 뒤 잡는 점을 **위로** 뺌 (입구 + 3 cm -> + 8 cm). IN1 은 아무 방향 6 cm 라 뒤로 빼다 팔이 몸체에 15 N (2 대)
    rew_release_ok = 6.0  # IN-N3: 3 -> 6.  # IN2: 통 안 10 mm 넘게·축 3 mm·직립 0.99 에서 놓음 (IN1 실패 = 10 ~ 23 mm 에서 기울어 끼인 채 안 놓음)
    rew_jam = -2.0  # 손가락이 넣는 자리에서 15 N 넘게 누름 (끼임)
    rew_time = -0.05
    rew_puck_tilt = -5.0  # (1 − 퍽 축 z), 쥐고 있을 때
    rew_action_rate = -0.01
