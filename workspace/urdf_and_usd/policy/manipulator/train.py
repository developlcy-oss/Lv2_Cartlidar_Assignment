"""Waffle 과제 학습·재생 (Isaac Lab 3.0 RSL-RL 진입점을 그대로 쓰고 과제 등록만 먼저). run_isaac.sh train / play 로 부름.

로그·체크포인트: sim/isaac/logs/rsl_rl/<experiment_name>/<시각>/ (실행한 폴더 기준 logs/)
"""

import os
import sys

import warp as wp

wp.config.enable_backward = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import waffle_isaac  # noqa: E402,F401  (gym 등록)

from isaaclab_rl.entrypoints import run_play_cli, run_train_cli  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--rl_library" not in argv:
        argv = ["--rl_library", "rsl_rl"] + argv
    play = os.path.basename(sys.argv[0]).startswith("play")
    raise SystemExit((run_play_cli if play else run_train_cli)(argv))
