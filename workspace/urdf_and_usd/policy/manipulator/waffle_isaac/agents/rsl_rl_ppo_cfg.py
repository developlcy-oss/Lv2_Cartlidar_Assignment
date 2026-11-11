"""Waffle 주행 과제 RSL-RL PPO 설정 (작은 MLP, 8 GB GPU 용)."""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class WaffleDrivePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 500
    save_interval = 50
    experiment_name = "waffle_drive"
    actor = RslRlMLPModelCfg(
        hidden_dims=[64, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.6),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[64, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive", gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class WafflePickPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """주사위 2 집기 P2 (관측 32, 행동 6). 이전 로봇 퍽 잡기 (96 %) 와 같은 크기·학습률."""

    num_steps_per_env = 24
    max_iterations = 300  # G1: 150 ~ 250 반복에서 보상이 평평해짐 (1500 은 과함)
    save_interval = 50
    experiment_name = "waffle_pick"
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.006, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=1.0e-4, schedule="adaptive", gamma=0.98, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class WafflePlacePPORunnerCfg(WafflePickPPORunnerCfg):
    """주사위 2 놓기 P1 (관측 33, 행동 6). 집기와 같은 망·학습률."""

    experiment_name = "waffle_place"


@configclass
class WaffleSidePickPPORunnerCfg(WafflePickPPORunnerCfg):
    """정렬·집기 (옆에서 집기). 261028 N8 부터 직교 행동 (관측 33, 행동 4) 이라 넣기 전용과 같은 탐색 설정:
    시작 표준편차 1.0 -> 0.5, entropy 0.006 -> 0.003, 학습률 1e-4 -> 3e-4 (N7 은 표준편차가 1.34 까지 커져 집게가 흔들리고 열고 닫기가 뒤섞여 한 번도 못 쥠)."""

    experiment_name = "waffle_sidepick"
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.003, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=3.0e-4, schedule="adaptive", gamma=0.98, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class ClampedGaussianCfg(RslRlMLPModelCfg.GaussianDistributionCfg):
    """표준편차 범위를 둔 가우시안 (rsl_rl GaussianDistribution std_range). TP3 학습 (2026-11-04) 은 9000 반복 뒤 표준편차 0.65 -> 3.4 로 폭주."""

    std_range: tuple = (0.05, 0.8)


@configclass
class WaffleTopPickPPORunnerCfg(WaffleSidePickPPORunnerCfg):
    """위에서 집기 (관측 36 = 35 + 기울임 모드, 행동 4 = 잡는 점 변위 + 집게). 옆에서 집기 N8 과 같은 탐색 설정 (직교 행동).
    TP4 (2026-11-05) 부터 표준편차 상한 0.8 (학습 중 행동 잡음이 커지면 정밀 쥐기를 못 배움)."""

    experiment_name = "waffle_toppick"
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=ClampedGaussianCfg(init_std=0.5),
    )


@configclass
class WaffleStorePPORunnerCfg(WafflePickPPORunnerCfg):
    """스토리지 넣기 (관측 41, 행동 6). 에피소드 10 s (300 스텝) 로 길어 gamma 0.99. ST3 부터 entropy 0.006 -> 0.003 (ST2 행동 표준편차 0.93 이 커서 mm 맞춤 방해)."""

    experiment_name = "waffle_store"
    actor = RslRlMLPModelCfg(  # 261028: 시작 표준편차 1.0 -> 0.5 (정렬·집기 N7 -> N8 교훈: 큰 탐색은 새 팔에서 정밀 동작을 못 배움)
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.003, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=3.0e-4, schedule="adaptive", gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class WaffleInsertPPORunnerCfg(WafflePickPPORunnerCfg):
    """넣기 전용 (정책 관측 35 = 실제 로봇에서 얻을 수 있는 것, 평가 critic 은 + 시뮬레이션 참값 12, 행동 4 = 잡는 점 위치 변화 + 집게).
    탐색 시작 표준편차 0.5 (1 mm 틈), 에피소드 4 s."""

    experiment_name = "waffle_insert"
    max_iterations = 300
    obs_groups = {"actor": ["policy"], "critic": ["policy", "critic"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.003, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=3.0e-4, schedule="adaptive", gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class WaffleLieUprightPPORunnerCfg(WaffleStorePPORunnerCfg):
    """2026-11-10 (Onbeom, #192) 누운 직육면체 세우기 P1 (관측 45 + 평가자 참값 16, 행동 6). 설계 4 절: [256, 128, 64] elu, 학습률 3e-4 adaptive,
    gamma 0.99, lam 0.95, entropy 0.003, 시작 표준편차 0.5, 24 스텝/반복, 미니배치 4, 에포크 5 (팀 Store 와 같음)."""

    experiment_name = "waffle_lieupright"
    max_iterations = 1000
    save_interval = 50
    obs_groups = {"actor": ["policy"], "critic": ["policy", "critic"]}
