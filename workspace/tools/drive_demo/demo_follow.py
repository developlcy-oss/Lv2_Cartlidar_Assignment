#!/usr/bin/env python3
"""규칙 v2 계획 시연: demo_plan.json 을 Nav2 (controller_server FollowPath · behavior_server Spin) 로 따라가며 기록 (run_demo.sh 가 부름).

  python3 demo_follow.py --plan demo_plan.json --out <기록 폴더> [--sim] [--step] [--legs 0-3]

  --sim   노트북 loopback 시뮬 (모의 시각, 시작 자세를 /initialpose 로 맞출 때까지 되풀이)
  실물    벽시계. 시작 전 확인 (11-15 강화): map → base_link 가 출발 자세에서 15 cm · 15° 안 그리고
            위치 감시 (loc_monitor) 가 떠 있으면 /localization/status = OK 만 (SUSPECT · 상태 없음은 안 됨),
            꺼져 있으면 /amcl_pose σ ≤ 5 cm · 5° + RViz 로 라이다 ↔ 벽 겹침을 보고 y 입력.
          60 s 안에 안 맞으면 이유를 알려 주고 끝. --set-initial 이면 출발 자세를 /initialpose 로 한 번 냄 (로봇을 출발 자리에 정확히 놓았을 때만)
  --step  구간마다 Enter 를 눌러야 출발 (q = 끝). Ctrl+C = 지금 동작 취소 → Nav2 가 0 을 냄 → 멈춤 (소프트웨어 정지, 물리 비상정지 아님)
조각: 전진 · 후진 = FollowPath (Forward · Reverse, 끝 뒤 연장선 → 원래 끝을 지나면 취소), 제자리 = Spin, 정지점 방향 2° 넘게 다르면 Spin.
      11-15: 전진 · 후진 조각을 시작할 때 방향이 4° 넘게 틀어져 있으면 먼저 Spin 으로 맞춤 (--no-head-fix 로 끔).
11-15 기본 경로 + 들르기 (계획 파일 "mode": "route"): 기본 경로 (상자 순회) 를 달리다가 들르기 요청이 오면 그 정지점에서
      요청한 곳 (예 L2) 을 먼저 들른 뒤 기본 경로 다음 곳으로 복귀. 길은 길 모음 (legs.json, 90 개) 에서 꺼냄.
      요청 = step 모드에서 L1 · L2 · L3 (또는 상자 이름) 입력 · 토픽 /drive_demo/detour (std_msgs/String) · 계획 파일 auto_detour (시뮬 시험).
      정지점에 도착하면 /drive_demo/arrived (std_msgs/String, 그 이름) 를 냄. 어느 L 이 완성 가능한지는 정하지 않음 (요청을 따를 뿐).
11-19 장애물 막힘 · 우회: 전진 · 후진 조각 중 collision_monitor 가 멈춤 (/collision_monitor_state STOP) 을 걸면 최대 BLOCK_WAIT (5 s) 기다림
      (FollowPath 는 살아 있어 장애물이 사라지면 그대로 이어감, 기다린 시간은 조각 시간 초과에 더함). 5 s 넘으면 조각을 취소하고 전진 조각이면
      먼저 지나온 길로 DETOUR_BACK (0.15 m) 후진 (멈춘 자리는 앞 끝 ↔ 장애물 3 ~ 15 cm 라 반경 0.20 전진만으로는 어깨가 모서리를 못 비킴,
      0.20 m 넘게 떨어져야 앞뒤 약 1.06 m · 옆 0.32 m 로 비킴) → planner_server (compute_path_to_pose, planner_id Detour) 로
      조각 1.1 · 1.3 · 1.5 m 앞 (또는 끝) 까지 우회 경로 (조각에서 0.4 m 안) 를 받아 따라간 뒤 남은 조각을 이어감 (구간당 최대 DETOUR_MAX 번).
      후진 실패 (뒤쪽 띠 막힘 등) 도 기존 되찾기로.
      정지 래치 (HOLD): STOP 을 처음 받으면 FollowPath 를 바로 취소하고, 라이다 (--scan-topic, 기본 /scan_merged) 로 진행 방향 차체 끝
      0.20 m · 폭 ±0.20 m 띠를 직접 봄 → 점 4 개 미만이 1 s 이어지면 진행 인덱스부터 이어감 (멈춘 뒤 느린 명령으로 다가가는 것 막음).
      라이다가 끊기면 안 풂. 시작 방향이 90° 넘게 틀어진 조각은 방향 맞추기 대신 실패로 (되찾기 · 건너뛰기). 후진 조각 · 우회 경로 없음 · planner 없음 (SAFETY=0) 이면 기존 되찾기. --no-detour 또는 DEMO_DETOUR=0 으로 끔.
      시험: DEMO_TEST_BLOCK=1 = 첫 전진 조각 (> 0.6 m) 3 s 뒤 5 s 넘게 막힌 것처럼.
11-20 상자 앞 정지 띠 끄기 (box_quiet.py): 앞 라이다를 1.5 cm 낮춰 70 mm 상자가 보이므로, 목적지가 상자 (box1 ~ 6 · L1 ~ 3) 인 구간에서
      map 자세가 정지점 --box-quiet-m (기본 0.40 m) 안이면 collision_monitor 를 끄고 (/collision_monitor/toggle, nav2_msgs/Toggle)
      HOLD 판단을 건너뜀. 0.42 m 밖으로 나가거나 구간이 끝나면 · Ctrl+C · 예외로 끝날 때 다시 켬. 0 = 기능 끔.
11-21 (사용자 결정, 위 11-20 을 바꿈) 상자 구간: 0.40 m 안 (끝 0.42 m) 에서 collision_monitor 를 끄지 않고 /drive_demo/box_zone
      (상자 이름 · 가운데, 0.1 s 마다) 을 냄 → box_scan_guard.py 가 그 상자 점만 지운 /scan_guard · /scan_front_guard (상자 구간만 앞 라이다
      원본 — /scan_merged 는 차체 앞 3.5 cm 안을 못 봄) 를 collision_monitor · HOLD 에 줌. HOLD 건너뛰기 없음.
      정지점 0.30 m 안 (BOX_CAP_M) 에서는 /drive_demo/speed_cap ZONE_V (0.15 m/s) → v2_filter (max 에서도 상자 앞은 느리게, 다가가기 ≤ 0.05 는 그대로).
11-21 상자 다가가기 (box_face.py, --box-approach on · off · dry, 기본 on): 상자 구간 (box1 ~ 6 · L1 ~ 3, --box-approach-at 으로 줄임) 에서
      마지막 전진 조각을 정지점 AP_MEASURE_D (11-22: 0.038 m → 측정 틈 AP_MEASURE_GAP 0.145 = 정지점 틈 0.107 + 0.038) 앞에서 끊고 멈춰 원본 앞 라이다 (/scan_front, 차체 가림 지우기 전) 로 상자 앞면을 잼
      (마지막 조각이 전진이 아니면 도착 뒤 곧게 후진해서 잼). 11-22: 틈 ≤ 0.20 m 로 재면 S 자 계획 (bf.plan_s_curve_face) 한 번으로 5 cm 자리까지 (옆 ≲ 3 cm 후진 없음, 더 크면 짧은 후진), 다시 재지 않음.
      (아래는 틈 > 0.20 m 로 잰 예전 흐름) 상자 법선 · 중앙선을 따라 앞 끝 ↔ 상자 0.20 m 자리로 기동 → 다시 잼
      → 법선으로 돌고 곧게 (간격 − --box-gap 0.05 m) 전진. 기동은 Nav2 가 쉬는 동안 cmd_vel_raw 에 직접 (v2_filter 를 거침), 회전 · 거리는
      오도메트리 (--odom-topic) 로 닫음. 측정 실패 · 상한 · 시간 초과면 ⚠ 남기고 기존 동작 (남은 조각 · 정지점 방향 맞추기) 으로 — 구간 실패 아님.
      어긋남 판정 (POS_OK) 은 다가가기 전 자리 (끊은 경우 = 끊은 점) 로, 다가간 뒤 값은 course_result approach 키에 따로.
      dry = 조각을 끊지 않고 정지점 0.30 m 앞을 지날 때 잰 값 · 기동 계획만 기록. 앞 라이다 · 오도메트리가 없으면 (시뮬 등) 그 구간은 기존 동작.
기록: 0.1 s 마다 map → base_link + cmd_vel_raw (Nav2 명령) + 필터 뒤 명령 → track.csv · course_result.json · moves.json · summary.txt (주행기록 HTML 형식)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import sys
import threading
import time
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

from box_quiet import STOP_GAP, BOX_CAP_M, BOX_HALF, BOX_QUIET_HYST, BOX_QUIET_M, BOX_STOPS, box_center, box_goal, cap_on, leg_target, quiet_next
import box_quiet   # 11-21 (3) 상자 좌표 BOXES (plan 이 현장 보정하면 그대로 따름)

try:   # 11-21 상자 다가가기 (순수 함수). 없으면 기능만 끄고 시연은 그대로
    import box_face as bf
    BF_ERR = ""
except ImportError as _e:   # noqa: N816
    bf, BF_ERR = None, str(_e)

# 11-21 상자 다가가기 (box approach) — 사용자 결정: 5 cm 기준 = 상판 앞 끝 (base_link x 0.1328), 상자 중앙선 위 · 법선 방향
AP = {"mode": "on", "at": set(), "gap": 0.05, "loose": False}   # main 이 --box-approach · --box-approach-at · --box-gap · --box-loose 로 채움
AP_FRONT, AP_BOX_SIDE = 0.1328, 0.205   # 상판 앞 끝 [m] (robot_geometry base.front_offset) · 상자 한 변 [m]
AP_MEASURE_GAP = 0.145   # 11-22 측정 자리 = 측정할 때 상판 앞 끝 ↔ 상자 틈 [m] (0.13~0.16, 앞면 x 0.26~0.29, --ap-measure-gap cm).
                         #   bag 재생 (face): 앞면 x 0.40~0.50 은 상자 점 8 개 · 0/17, 0.26~0.29 는 점 46 개 · 5/14 (11-21 은 정지점 0.30 m 앞 = 틈 0.41)
AP_MEASURE_D = max(AP_MEASURE_GAP - STOP_GAP, 0.0)   # 끊는 거리: 마지막 전진 조각을 정지점 이만큼 [m] 앞에서 끊음 (정지점 틈 STOP_GAP 0.107 → 0.038). main 이 --ap-measure-gap 으로 다시 계산
                         #   (후진할 때도 정지점 이만큼 앞이 되게). 예전 0.30 근거: 0.25 면 1 차 기동 전진이 10 cm 뿐이라 옆 3 cm + 방향 6° 에서 box_face 회전 상한 25° 에 걸림
AP_CUT_MIN = 0.05        # 끊고 남은 앞부분이 이보다 [m] 짧으면 끊지 않음 (후진해서 잼)
AP_BACK_MAX = 0.35       # 후진해서 잴 때 최대 [m] (box_quiet 다시 켜기 0.42 m 안, v2_filter 이어진 후진 한도 안)
AP_CLOSE_GAP = 0.20      # 11-22 잰 틈이 이 이하 [m] 면 '가까이 잼': 곧바로 S 자 계획 (bf.plan_s_curve_face) 으로 5 cm 자리까지 (0.20 m 자리 다시 재기 없음)
AP_STANDOFF = 0.15       # 1 차 기동 끝 자리: 목표 간격 + 이 값 [m] (= 앞 끝 ↔ 상자 0.20 m, 라이다에서 상자 약 0.20 m → 아직 보임)
AP_SEE_GAP = 0.18        # 다시 잴 때 간격이 이 이상 [m] 으로 보이면 라이다 값, 아니면 오도메트리 예측
AP_N_SCANS, AP_N2_SCANS, AP_MIN_SCANS, AP_DRY_SCANS = 5, 5, 5, 3   # 1 차 · 다시 잴 때 스캔 장수 · 최소 (한 장 lateral σ 0.8 ~ 2 cm → 5 장 평균) · dry 잡는 장수
AP_SCAN_FRESH = 0.2     # 측정에 쓰는 스캔은 받은 지 이 시간 [s] 안 (서로 다른 header.stamp)
AP_SCAN_ALIVE = 1.0     # 구간 시작 때 앞 라이다를 이 시간 [s] 안에 받았어야 조각을 끊음 (아니면 그 구간은 다가가기 안 함)
AP_SCAN_STALE = 0.5     # 다가가는 동안 새 앞 스캔이 이 시간 [s] 넘게 없으면 0 → 그만둠 (마지막 오도메트리 직진은 예외)
AP_TOTAL_T = 30.0       # 측정부터 끝까지 전체 시간 상한 [s] (후진해서 잴 때 후진은 따로 단계 상한)
AP_OVER = 0.01          # 마지막 오도메트리 직진: 누적 전진이 계획 + 이 값 [m] 을 넘으면 그만둠
AP_TURN_GAP = 0.12      # 회전은 상판 앞 끝 ↔ 상자 틈이 이 이상 [m] 일 때만 (+ bf.turn_clear)
AP_SCAN_T = 3.0          # 스캔 모으기 시간 상한 [s]
AP_GAP_RANGE = (0.08, 0.45)   # 1 차 측정 간격을 믿는 범위 [m]
AP_LAT_MAX, AP_YAW_MAX = 0.10, math.radians(25.0)   # 1 차 측정 중앙선 어긋남 · 방향을 믿는 상한
AP_LAT_LEFT = 0.03       # 1 차 기동 뒤 중앙선 어긋남이 이보다 [m] 크면 그만둠 (기존 동작)
AP_MID_MAX = 2           # 11-21 (3) 0.20 m 자리 다시 잰 뒤 S 자 보정 최대 횟수 (0 = 예전처럼 확인만, --ap-mid-fix) — 보정마다 곧게 물러나 다시 잼
AP_MID_TOL = 0.008       # S 자 보정 허용 옆 오차 [m] (--ap-mid-tol mm). 보정이 다 안 돼도 AP_LAT_LEFT 안이면 경고하고 계속
AP_MID_BACK = 0.10       # S 자 보정 계획에 줄 뒤 여유 [m] (후진 S 자를 쓸 때 상한)
AP_MID_T = 12.0          # S 자 보정 한 번마다 전체 시간 상한 AP_TOTAL_T 에 더함 [s]
AP_DRIVE_MIN, AP_DRIVE_MAX = -0.03, 0.30   # 마지막 곧은 전진 [m] 허용 범위
AP_V_MAX, AP_V_FINAL, AP_V_MIN = 0.05, 0.03, 0.015   # [m/s] 1 차 기동 · 마지막 전진 상한 · 끝 가까이 하한
AP_W_MAX, AP_W_MIN = 0.30, 0.10   # 제자리 회전 [rad/s] 상한 · 하한
AP_KP_LIN, AP_KP_YAW = 1.0, 1.5   # 남은 거리 [m] → 속도, 남은 각 [rad] → 각속도
AP_KP_HOLD, AP_HOLD_W = 1.0, 0.15   # 곧게 갈 때 방향 유지 이득 · 상한 [rad/s]
AP_YAW_TOL, AP_POS_TOL = math.radians(1.0), 0.002   # 회전 끝 · 거리 끝 허용
AP_STUCK_T = 3.0         # 이 시간 [s] 동안 나아가지 않으면 그만둠 (v2_filter 가 제자리 예산 · 후진 한도로 0 을 낼 때)
AP_TURN_T = 15.0         # 회전 한 번 시간 상한 [s]
ODOM_STALE = 0.5         # 오도메트리가 이 시간 [s] 넘게 끊기면 그만둠
AP_DT = 0.05             # 기동 명령 주기 [s] (20 Hz, v2_filter 는 0.25 s 끊기면 0)
# 11-21 (2) 다가가기 보강 — plan · route 공통
AP_CUT_YAW = math.radians(20.0)   # 끊는 점 경로 방향과 정지점 방향 차이가 이보다 크면 끊지 않고 도착 뒤 후진해서 잼 (0 = 예전처럼 늘 끊음, --ap-cut-yaw)
                                  #   wide S5 −34° · S6 −50° (AP_YAW_MAX 25° 를 넘어 늘 fallback 이던 곳)
AP_RETURN_STOP = True    # done 뒤 떼기 (ap_leave, --no-ap-return-stop 으로 끔): 곧게 후진해 틈 ≥ AP_LEAVE_GAP 과 정지점 중 더 뒤쪽까지. 상자 쪽 (앞) 으로는 안 감
AP_LEAVE_GAP = 0.12      # done 뒤 떼기 목표 틈 (상판 앞 끝 ↔ 상자) [m] — 5 cm 자리에서 바로 돌면 앞 모서리가 상자에 약 1.1 cm 겹침
                         #   (ICR x −0.0658 · 모서리 반경 0.260, bf.turn_clearance 로 로그). 회전 · 새 길 계산은 안 함 — 다음 구간 첫 후진이 이어받음
AP_LEAVE_MAX = 0.25      # 떼기 후진 최대 [m] (정지점이 그보다 뒤면 여기까지만, 나머지는 다음 구간 첫 후진)
AP_BACK_MIN = 0.10       # 후진해서 잴 때 이어진 후진 한도로 줄인 후진이 이보다 짧으면 [m] 다가가기 안 함
# 이어진 후진 합 (v2_filter 시뮬 0.40 · 실물 0.50 · widen REV_MAX 0.30 은 계획 길만, 0.15 m 전진해야 0 으로):
#   끊기 : 마지막 전진 ≥ 0.30 뒤 → 다가가기 기동은 전진 (AP_DRIVE_MIN −0.03 뿐) → done 뒤 떼기 후진 b + 다음 구간 첫 후진 중 남은 것
#          (leave_trim 이 이미 후진한 만큼 잘라 냄 → 합 ≈ done 자리 ↔ 첫 후진 끝 ≈ 0.30 − 정지점 뒤 거리) ≤ REV_MAX − 0.02 (넘으면 첫 후진 끝을 잘라 냄 · 경고)
#   후진해서 잼 : 앞 조각 끝 후진 (last_rev, S2 0.12) + 후진 b (≤ 0.35) 를 REV_MAX − 0.02 안으로 줄임 (S2 → b 0.26, 합 0.38) · 로그 "이어진 후진 합"
#   다가가기가 앞으로 0.16 ~ 0.29 m 가므로 그 뒤 후진은 새로 셈 (정지점 앞 끝 ↔ 상자 −8.8 ~ +2.5 cm 기준)
# 11-21 (2) 자율 순회 (--auto)
AUTO_MODE = False        # main 이 --auto 로 켬 (조각 자동 건너뛰기 DEMO_AUTO_SKIP 대신 곳 단위 건너뛰기)
AUTO_NEAR = 0.15         # 지금 자리가 구간 출발 정지점에서 이 안 [m] 이면 계획 길, 밖이면 planner 길
AUTO_ALONG = 0.30        # 첫 조각이 후진인 구간: 정지점 방향 선 위 (옆 ≤ AUTO_LAT · 방향 ≤ 20°) 로 이만큼 [m] 뒤까지도 계획 길 (다가가기 done 자리)
AUTO_LAT = 0.05
HOME_FIX_MAX = 0.10      # 출발 자세 곧게 보정 최대 [m]
# 11-21 (3) 상자 앞 Spin "Collision Ahead" 대책 (실물 1824 12 회 · 1809 6 회: 조각 시작 방향 맞추기 ±4 ~ 19° · 다가가기 뒤 정지점 복귀 방향 맞추기 ·
#   계획 제자리 +30 · +60° · 정지점 방향 맞추기 +3° — 모두 상자 몇 cm 앞에서 behavior Spin 이 local costmap 충돌 검사에 걸림)
TURN_DIRECT = math.radians(5.0)    # 이 각 이하 제자리 회전은 Spin 대신 cmd_vel_raw 직접 (오도메트리 되먹임, v2_filter 거침) (--turn-direct DEG, 0 = 끔)
                                   #   5° 면 앞 모서리가 앞으로 최대 1.4 cm 나옴 (ICR −0.0658 · 모서리 반경 0.260)
TURN_DIRECT_NEAR = math.radians(12.0)   # 상자 앞 (발제 상자 좌표, bf.turn_clear 됨) 이면 이 각까지 직접 · Spin 이 Collision Ahead 면 30° 까지 직접으로 한 번 더
TURN_DIRECT_FB = math.radians(30.0)
TURN_ROOM = True         # 정지점 아닌 제자리 (계획 S 조각) 전에 상자 앞이면 틈 확보 후진 (ap_leave 와 같은 기준: 틈 ≥ AP_LEAVE_GAP + bf.turn_clear) (--no-turn-room)
NEAR_BOX_GAP = 0.30      # 상판 앞 끝 ↔ 상자 앞면 이 안 [m] (방향 차 ≤ 60°) 이면 "상자 앞"
AP_LEAVE_ALL = True      # 다가가기 done 이 아니어도 (fallback · 끔 · dry) 상자 정지점 도착 뒤 떼기 — 상자 = 발제 좌표 (--no-ap-leave-all)


class ApFail(Exception):
    """상자 다가가기를 그만두고 기존 동작으로 (status = fallback · skip)"""

    def __init__(self, why, status="fallback"):
        super().__init__(why)
        self.status = status

POS_OK, YAW_OK, YAW_FIX = 0.05, math.radians(10.0), math.radians(2.0)
EXT_F, EXT_R = 0.15, 0.10
STOP_CREEP_D = 0.05   # 11-18 속도 곡선: v2_filter stop_creep_d 와 같게. 구간 끝 (정지점) 만 마지막 이 길이를 천천히, 중간 정지 (후진 · 제자리 앞) 는 이만큼 더해 보내 건너뜀
START_TOL, START_YAW_TOL = 0.15, math.radians(15.0)
AMCL_XY_OK, AMCL_YAW_OK = 0.05, 5.0   # 위치 감시를 껐을 때 출발 허용 AMCL σ [m] · [deg]
OFF_PATH = 0.10   # 11-15 21:4x 사용자 결정: 전진 · 후진 조각 중 AMCL 자리가 조각 경로에서 max(이 값, 시작 때 어긋남 + 3 cm) 넘게
                  #   벗어나면 그 조각을 취소하고 멈춤 (S6 벽 닿음: 경로에서 24 cm). 소프트웨어 정지, 물리 비상정지 아님. 0 = 끔 (--off-path)
RECOVER_TRIES, BACK_MAX = 2, 0.30   # 11-16 사용자 결정: 조각 실패 시 되찾기 (자기 위치 찾기 → 왔던 길 ≤ BACK_MAX m → 다시) 최대 횟수 (0 = 끔)
STEP_MODE = False                  # step 모드면 되찾기도 안 될 때 사람에게 물음
HEAD_FIX = math.radians(4.0)          # 전진 · 후진 조각 시작 때 방향이 이보다 틀어져 있으면 먼저 제자리로 맞춤 (0 = 끔, --no-head-fix)
BLOCK_WAIT, DETOUR_MAX = 5.0, 2   # 11-19 사용자 결정: collision_monitor 멈춤을 기다리는 최대 시간 [s] · 구간당 우회 최대 횟수
DETOUR_BACK = 0.15                  # 11-19 (팀원 D 기하): 우회 계획 전 후진 길이 [m] — 장애물과 0.20 m 넘게 떨어져야 반경 0.20 전진만으로 비킴
START_BACK_N = int(round((DETOUR_BACK + BACK_MAX + 0.15) / 0.01))   # 11-19: 다시 시작할 인덱스를 진행 인덱스 뒤로 찾는 창 [점 수, 1 cm 간격]
                                    #   = 우회 전 후진 + 되찾기 후진 (최대 0.45 m) + 여유 0.15 → 60 개
DETOUR_JOIN = (1.1, 1.3, 1.5)       # 우회 뒤 다시 합류할 점 후보: 후진한 자리 (조각 가장 가까운 점) 부터 경로 길이 [m] 앞 (+ 조각 끝). 1.06 m 기하 + 후진 0.15
DETOUR_DEV_MAX = 0.4                # 우회 경로가 원래 조각에서 벗어나도 되는 최대 거리 [m] (필요한 비킴 0.32 m, 더 크게 돌면 기둥 · 팔레트 위험)
DETOUR_ON = os.environ.get("DEMO_DETOUR", "1") != "0"   # --no-detour 로도 끔
BLOCKED = "BLOCKED"                 # abort_when 이 돌려주는 막힘 표시 (문자열 앞머리)
HOLD = "HOLD"                       # 11-19 정지 래치: STOP 을 받아 goal 을 취소했다는 표시 (follow_piece 안에서만)
BODY_FRONT, BODY_REAR = 0.25, 0.2135   # base_link 에서 차체 앞 끝 · 뒤 끝 [m] (params footprint)
HOLD_DEPTH, HOLD_HALF = 0.20, 0.20     # HOLD 확인 띠: 진행 방향 차체 끝에서 깊이 · 폭 ± [m]
HOLD_PTS, HOLD_CLEAR = 4, 1.0          # 띠 안 점이 이 개수 미만인 채 이 시간 [s] 이어지면 비었다고 봄
SCAN_STALE = 2.0                       # 라이다가 이 시간 [s] 넘게 끊기면 HOLD 를 풀지 않음
CM_STALE = 0.5                         # goal 보내기 전부터 남은 STOP 은 이 시간 [s] 동안 안 믿음 (상태가 바뀔 때만 발행되므로)
HOLD_RUN_OK = 1.0                      # 이어간 뒤 이 시간 [s] 넘게 달렸으면 다음 STOP 은 새 HOLD (기다림 시계 새로)
ZONE_V = 0.15                          # 11-21 상자 정지점 0.30 m (BOX_CAP_M) 안 속도 상한 [m/s] (v2_filter /drive_demo/speed_cap)
TEST_BLOCK = {"done": False}        # 시험용 DEMO_TEST_BLOCK=1 은 전체에서 한 번만
STOP = threading.Event()
LIVE = None   # 11-15 21:3x: track_live.csv — 0.1 s 마다 바로 씀 (Ctrl+C · 강제 종료로 save() 를 못 해도 남음)
NAMES = ["HOME", "box1", "box2", "box3", "box4", "box5", "box6", "L1", "L2", "L3"]
SPIN_MAX_DEG, REV_MAX = 90.0, 0.40   # 규칙 v2 이음매 확인 (길모음/compose.py 와 같음)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_q(y):
    return math.sin(y / 2), math.cos(y / 2)


class Follower(Node):
    def __init__(self, sim, scan_topic="/scan_merged"):
        super().__init__("demo_follower", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, sim)])
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.fp = ActionClient(self, FollowPath, "follow_path")
        self.spin = ActionClient(self, Spin, "spin")
        self.sim = sim
        self.nomotion = self.create_client(Empty, "/request_nomotion_update")   # AMCL: 멈춘 채 위치 다시 계산 (되찾기)
        self.recover = []   # 이 구간 되찾기 기록
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.raw = (0.0, 0.0, -9.0)
        self.out = (0.0, 0.0, -9.0)
        self.create_subscription(TwistStamped, "cmd_vel_raw", lambda m: setattr(self, "raw", (m.twist.linear.x, m.twist.angular.z, self.now_s())), 20)
        self.create_subscription(TwistStamped, "cmd_vel_v2", lambda m: setattr(self, "out", (m.twist.linear.x, m.twist.angular.z, self.now_s())), 20)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.loc = None
        self.create_subscription(String, "/localization/status", lambda m: setattr(self, "loc", m.data), latched)
        # AMCL 자세 확신도 (위치 감시를 끈 실물 시연용). 발행 쪽 durability 를 몰라 두 가지로 받음 (맞는 쪽만 연결됨)
        self.amcl = None   # (σxy [m], σyaw [deg], 받은 벽시계)
        for dur in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=dur))
        self.step = -1
        self.head_fix = []   # 조각 시작 방향 맞추기 각 [deg]
        self.head_skip = []  # 11-16: 거부 · 실패해서 건너뛴 방향 맞추기 각 [deg]
        self.rows = []
        self.lock = threading.Lock()
        self.create_timer(0.1, self._rec)
        # 11-15 들르기: 요청 (언제 와도 다음 정지점에서 씀) · 도착 알림
        self.pending = deque()
        self.plock = threading.Lock()
        self.create_subscription(String, "/drive_demo/detour", self._detour, 10)
        self.arrived = self.create_publisher(String, "/drive_demo/arrived", 10)
        self.stop_pub = self.create_publisher(Float32, "/drive_demo/stop_dist", 10)   # 11-18 속도 곡선: 다음 정지까지 남은 길이 (v2_filter 가 감속)
        # 11-19 장애물 막힘 · 우회: collision_monitor 상태 (바뀔 때만 옴) → STOP 이 시작된 벽시계, planner_server 우회 경로
        self.cm_since = None
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._cm, 10)
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.planner_warned = False
        # 11-19 정지 래치 (HOLD): 라이다 직접 (받은 벽시계, 메시지) · base_link ← 라이다 정적 TF (한 번 조회해 둠)
        self.scan, self.scan_tf = None, None
        self.create_subscription(LaserScan, scan_topic, lambda m: setattr(self, "scan", (time.time(), m)), qos_profile_sensor_data)
        self.hold_n, self.tries, self.leg_t0 = 0, [], 0.0   # run_leg 이 구간마다 맞춤 (follow_piece 가 HOLD 기록)
        # 11-21 상자 구간: 구간 정지점 (상자 구간만) 0.40 m 안이면 box_zone (상자 점 지우기) · 0.30 m 안이면 speed_cap
        #   collision_monitor · HOLD 는 끄지 않음 — box_scan_guard 가 그 상자 점만 지운 스캔을 줌
        self.quiet_m = BOX_QUIET_M
        self.quiet_goal = None    # 지금 구간이 상자 구간이면 정지점 (x, y)
        self.quiet_box = None     # 그 상자 (이름, (cx, cy)) — 좌표 없으면 None (상자 점을 못 지움 → 상자에도 멈출 수 있음, 안전 쪽)
        self.quiet = False        # True = 상자 구간 안
        self.quiet_sent = False   # 한 번이라도 들어갔나 (끝낼 때 끝 알림)
        self.quiet_n = 0          # 이 구간 들어간 횟수 (기록)
        self.zone_pub = self.create_publisher(String, "/drive_demo/box_zone", 10)
        self.cap_pub = self.create_publisher(Float32, "/drive_demo/speed_cap", 10)
        self.qlock = threading.Lock()
        self.create_timer(0.1, self._quiet_tick)
        self.prog = ""            # 11-21 (2) auto: 구간 줄 앞 진행률 "[3/9] "

    def _quiet_tick(self):
        p = self.pose()
        with self.qlock:
            self._apply_quiet(quiet_next(p, self.quiet_goal, self.quiet, self.quiet_m), p)
            if self.quiet:   # 들어가 있는 동안 계속 냄 (box_scan_guard 는 1 s · v2_filter 는 0.3 s 안 오면 끝으로 봄)
                self._zone_msg(p)

    def _zone_msg(self, p):   # self.qlock 안에서만
        b = self.quiet_box
        self.zone_pub.publish(String(data=json.dumps({"name": b[0], "cx": b[1][0], "cy": b[1][1], "half": BOX_HALF}) if b else ""))
        if cap_on(p, self.quiet_goal):
            self.cap_pub.publish(Float32(data=float(ZONE_V)))

    def set_quiet_goal(self, goal, name=""):
        """구간 시작 (상자 정지점 · 상자 이름 또는 None) · 끝 (None) 에 부름. None 이면 바로 끝"""
        p = self.pose()
        with self.qlock:
            self.quiet_goal = goal
            c = box_center(name) if goal else None
            self.quiet_box = (name, c) if c else None
            self._apply_quiet(quiet_next(p, goal, self.quiet, self.quiet_m), p)

    def _apply_quiet(self, q, p):   # self.qlock 안에서만
        if q == self.quiet:
            return
        self.quiet = q
        g = self.quiet_goal
        dist = f"정지점까지 {math.hypot(p[0] - g[0], p[1] - g[1]):.2f} m" if (p and g) else "구간 끝"
        if q:
            self.quiet_n += 1
            self.quiet_sent = True
            print(f"    · 상자 {self.quiet_m:.2f} m 안 ({dist}) — 정지 띠 · HOLD 는 "
                  + ("상자 점 뺀 라이다로 (앞 라이다 가까운 곳 포함)" if self.quiet_box else "그대로 (⚠ 상자 좌표 없음 — 상자에도 멈출 수 있음)")
                  + f" · {BOX_CAP_M:.2f} m 안 속도 ≤ {ZONE_V:.2f} m/s", flush=True)
            self._zone_msg(p)
        else:
            print(f"    · 상자 구간 벗어남 ({dist}) — 상자 점 빼기 · 속도 상한 끝", flush=True)
            self.zone_pub.publish(String(data=""))

    def quiet_close(self):
        """끝낼 때 (정상 · Ctrl+C · 예외): 상자 구간 끝 알림 (box_scan_guard · v2_filter 는 알림이 없어도 1 s · 0.3 s 뒤 스스로 끝냄)"""
        with self.qlock:
            self.quiet_goal = None
            self.quiet = False
            sent = self.quiet_sent
        if sent:
            self.zone_pub.publish(String(data=""))
            print("  · 끝냄: 상자 구간 끝 알림", flush=True)

    def band_count(self, kind):
        """HOLD 확인 띠 안 라이다 점 개수 → (개수, "") 또는 (None, 이유)"""
        sc = self.scan
        if sc is None or time.time() - sc[0] > SCAN_STALE:
            return None, f"라이다 못 받음 ({SCAN_STALE:.0f} s 넘게)"
        m = sc[1]
        if self.scan_tf is None:
            try:
                t = self.tf.lookup_transform("base_link", m.header.frame_id, Time())
            except Exception:  # noqa: BLE001
                return None, f"TF base_link ← {m.header.frame_id} 없음"
            q = t.transform.rotation
            # ⚠ 11-21 알려진 문제 (이번엔 안 고침): yaw 만 뽑으므로 roll π 프레임 (laser_front rpy π 0 π = Ry(π)) 이면 y 부호가 거꾸로.
            #   기본 /scan_merged 는 frame base_link (단위 변환) 라 영향 없음. --scan-topic /scan_front 처럼 라이다 프레임을 직접 주면
            #   띠가 좌우 대칭 (±HOLD_HALF) 이라 점 개수는 같음. 좌우 비대칭 판단에 이 식을 다시 쓰지 말 것 (상자 다가가기는 ap_tf6 = 전체 회전행렬)
            self.scan_tf = (t.transform.translation.x, t.transform.translation.y,
                            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return count_in_band(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, self.scan_tf, kind), ""

    def setup_approach(self, scan_topic, odom_topic):
        """11-21 상자 다가가기: 원본 앞 라이다 · 오도메트리 구독, cmd_vel_raw 발행 (Nav2 가 쉬는 동안만 씀)"""
        self.fscan = None          # (받은 벽시계, LaserScan) — 원본 앞 라이다 (차체 가림 지우기 전)
        self.odom = None           # (x, y, yaw, 받은 벽시계) — odom 프레임
        self.ftf = {}              # 라이다 frame → base_link 2D (tx, ty, r00, r01, r10, r11)
        self.ap_res = None         # 이 구간 상자 다가가기 기록 (run_leg 이 비움)
        self.ap_cap = []           # dry: 정지점 AP_MEASURE_D 앞을 지날 때 잡은 (스캔, map 자세)
        self.ap_dry_goal = None    # dry: 지금 구간 정지점 (x, y, yaw) — 있으면 위 스캔을 잡음
        self.ap_pub = self.create_publisher(TwistStamped, "cmd_vel_raw", 10)
        self.create_subscription(LaserScan, scan_topic, self._fscan, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._odom, 20)

    def _fscan(self, m):
        self.fscan = (time.time(), m)
        g = self.ap_dry_goal
        if g is not None and len(self.ap_cap) < AP_DRY_SCANS:
            p = self.pose()
            if p is not None and math.hypot(g[0] - p[0], g[1] - p[1]) <= AP_MEASURE_D + 0.03:
                self.ap_cap.append((m, p))

    def _odom(self, m):
        q = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), time.time())

    def _cm(self, m):
        if m.action_type == CollisionMonitorState.STOP:
            if self.cm_since is None:
                self.cm_since = time.time()
        else:
            self.cm_since = None

    def _detour(self, m):
        nm = norm(m.data)
        if nm not in NAMES:
            print(f"\n  ⚠ 들르기 요청 '{m.data}' 모름 (쓸 수 있는 이름: {' '.join(NAMES)})", flush=True)
            return
        with self.plock:
            self.pending.append(nm)
        print(f"\n  ↪ 들르기 요청 받음: {nm} (다음 정지점에서)", flush=True)

    def _amcl(self, m):
        c = m.pose.covariance
        self.amcl = (math.sqrt(max(c[0], c[7], 0.0)), math.degrees(math.sqrt(max(c[35], 0.0))), time.time())

    def loc_live(self):
        """위치 감시 (loc_monitor) 가 지금 떠 있나 — 꺼진 뒤 남은 예전 상태는 안 씀"""
        return self.count_publishers("/localization/status") > 0

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _rec(self):
        p = self.pose()
        if p is None:
            return
        t = self.now_s()
        v, w, ts = self.out
        if t - ts > 0.25:
            v = w = 0.0
        rv, rw, rts = self.raw
        if t - rts > 0.25:
            rv = rw = 0.0
        with self.lock:
            self.rows.append((t, p[0], p[1], math.degrees(p[2]), v, w, self.step, rv, rw, self.loc or ""))
        if LIVE is not None:
            try:
                LIVE.write(f"{t:.2f},{p[0]:.4f},{p[1]:.4f},{math.degrees(p[2]):.1f},{v:.3f},{w:.3f},{self.step},{rv:.3f},{rw:.3f},{self.loc or ''}\n")
            except (OSError, ValueError):
                pass

    def set_initial(self, x, y, th):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_q(th)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.02 ** 2
        m.pose.covariance[35] = math.radians(3) ** 2
        self.init_pub.publish(m)


def norm(t):
    t = (t or "").strip()
    if t.upper() in ("L1", "L2", "L3", "HOME"):
        return t.upper()
    return t.lower() if t.lower().startswith("box") else t


def first_rev(pieces):
    """길 처음의 후진 길이 (전진 0.15 m 전까지 이어지는 후진 합) — 길모음/compose.py 와 같음"""
    rev = fw = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            continue
        L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pc["poses"], pc["poses"][1:]))
        if pc["kind"] == "R":
            rev += L
            fw = 0.0
        else:
            fw += L
            if fw >= 0.15:
                break
    return rev


def first_spin(pieces):
    """길 처음 (0.15 m 움직이기 전) 의 제자리 회전 각 합 [deg] — 길모음/compose.py 와 같음"""
    sp = moved = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            sp += abs(math.degrees(pc["dth"]))
            moved = 0.0
            continue
        moved += sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pc["poses"], pc["poses"][1:]))
        if moved >= 0.15:
            break
    return sp


def wait_future(fut, timeout):
    t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.02)
    return True


def sleep_s(node, s):
    t0 = node.now_s()
    while node.now_s() - t0 < s and not STOP.is_set():
        time.sleep(0.02)


def run_action(client, goal, timeout, done_when=None, abort_when=None):
    if not client.wait_for_server(timeout_sec=10.0):
        return False, "action server 없음"
    gf = client.send_goal_async(goal)
    if not wait_future(gf, 10.0):
        return False, "goal 응답 없음"
    gh = gf.result()
    if not gh.accepted:
        return False, "goal 거절"
    rf = gh.get_result_async()
    t0 = time.time()
    while not rf.done():
        if STOP.is_set():
            wait_future(gh.cancel_goal_async(), 5.0)
            return False, "Ctrl+C 로 멈춤"
        why = abort_when() if abort_when is not None else None
        if why:
            wait_future(gh.cancel_goal_async(), 5.0)
            wait_future(rf, 5.0)
            return False, why
        if done_when is not None and done_when():
            wait_future(gh.cancel_goal_async(), 5.0)
            wait_future(rf, 5.0)
            return True, ""
        if time.time() - t0 > timeout:
            wait_future(gh.cancel_goal_async(), 5.0)
            return False, f"{timeout:.0f} s 안에 안 끝남 (취소)"
        time.sleep(0.01)
    res = rf.result()
    ok = res.status == 4
    r = res.result
    return ok, ("" if ok else f"상태 {res.status} · 오류 {getattr(r, 'error_code', 0)} {getattr(r, 'error_msg', '') or ''}".strip())


def merge_pieces(pieces):
    """11-18: 이어진 같은 방향 조각 (전진 → 전진, 끝 · 시작 2 cm · 10° 안) 을 하나로 — 사이에서 멈추지 않게"""
    out = []
    for pc in pieces:
        if out and pc["kind"] in "FR" and out[-1]["kind"] == pc["kind"] and pc.get("poses") and out[-1].get("poses"):
            e, b = out[-1]["poses"][-1], pc["poses"][0]
            if math.hypot(b[0] - e[0], b[1] - e[1]) < 0.02 and abs(wrap(b[2] - e[2])) < math.radians(10):
                out[-1] = dict(out[-1], poses=list(out[-1]["poses"]) + list(pc["poses"][1:] if math.hypot(b[0] - e[0], b[1] - e[1]) < 1e-4 else pc["poses"]))
                continue
        out.append(dict(pc))
    return out


def pub_stop(node, d):
    m = Float32()
    m.data = float(d)
    node.stop_pub.publish(m)


def follow_piece(node, pc, final=True):
    if pc["kind"] == "S" and not pc.get("_raw"):   # 11-21 (3): 작은 각 · 상자 앞은 turn_safe 가 직접 회전 · 틈 확보 후진 → Spin
        return turn_safe(node, pc["dth"], room=bool(pc.get("_room")), rev_budget=pc.get("_rev_budget"))
    if pc["kind"] == "S":
        g = Spin.Goal()
        g.target_yaw = float(pc["dth"])
        g.time_allowance = Duration(sec=int(abs(pc["dth"]) / 0.2) + 10)
        return run_action(node.spin, g, abs(pc["dth"]) / 0.2 + 30)
    poses = pc["poses"]
    cur = node.pose()
    if len(poses) < 2 or (cur and math.hypot(poses[-1][0] - cur[0], poses[-1][1] - cur[1]) < 0.03):
        return True, ""
    # 11-15: 조각 시작 방향 맞추기 — 앞 조각 (특히 짧은 후진) 끝에서 방향이 틀어진 채 출발하면 반경 0.20 한계 때문에 못 돌아와 옆으로 밀림
    #   (11-14 22:57 시뮬 S5: 후진 끝 +8.5° → 전진 중 기둥 쪽으로 2 ~ 3 cm). 멈춘 순간이라 작은 제자리 회전 (규칙 v2 예산 안)
    #   11-19: 목표는 실제로 따라가기 시작할 인덱스 (진행 인덱스 근처, 보내는 Path 의 시작) 의 경로 방향. 처음 시작하는 조각은 0 = 예전과 같음
    #   (전에는 늘 poses[0] → 곡선 조각을 되찾기 · 이어가기로 중간부터 다시 갈 때 -119° · -124° 같은 엉뚱한 각, 팀원 A 시뮬 (b) S0)
    k_start, err = start_heading(poses, cur, pc.get("_near", 0)) if cur is not None else (0, 0.0)
    if HEAD_FIX > 0 and cur is not None:
        if abs(err) > math.radians(SPIN_MAX_DEG):
            # 11-19: 규칙 v2 제자리 한도 (90°) 넘는 방향 맞추기는 안 함 — 되찾기 중 -124° 요구 → Spin 시간 초과 (팀원 A 시뮬).
            #   이 조각 시도를 실패로 돌려 되찾기 · 자동 건너뛰기 · 사람 판단으로
            print(f"    · 조각 시작 방향 {math.degrees(err):+.0f}° > {SPIN_MAX_DEG:.0f}° — 방향 맞추기 안 함 (이 시도 실패)", flush=True)
            return False, f"시작 방향 {math.degrees(err):+.0f}° 가 제자리 한도 {SPIN_MAX_DEG:.0f}° 넘음"
        if abs(err) > HEAD_FIX:
            print(f"    · 조각 시작 방향 맞추기 {math.degrees(err):+.1f}°", flush=True)
            node.head_fix.append(round(math.degrees(err), 1))
            if os.environ.get("DEMO_TEST_HEADFIX_FAIL") == "1":   # 시험용: 방향 맞추기가 모두 거부된 것처럼 (건너뛰기 경로 확인)
                r, msg = False, "시험용 거부 (DEMO_TEST_HEADFIX_FAIL=1)"
            else:
                r, msg = follow_piece(node, {"kind": "S", "dth": err})
            if not r:
                if STOP.is_set():
                    return False, f"시작 방향 맞추기 {math.degrees(err):+.0f}°: {msg}"
                # 11-16 (사용자 결정): Nav2 Spin 이 충돌 예상 (Collision Ahead) · 시간 초과로 거부하면 구간 전체를 멈추지 않고
                #   방향 맞추기만 건너뛰고 조각을 그대로 진행 (RPP 가 따라가며 바로잡음, 경로 이탈 멈춤이 지킴). 시뮬 P2 S2 에서 나옴
                print(f"    · 시작 방향 맞추기 건너뜀 ({msg}) — 조각을 그대로 진행 (경로 이탈 멈춤 {OFF_PATH * 100:.0f} cm 가 지킴)", flush=True)
                node.head_skip.append(round(math.degrees(err), 1))
    d = 1.0 if pc["kind"] == "F" else -1.0
    ex, ey, eth = poses[-1]
    ext = EXT_F if d > 0 else EXT_R
    full = list(poses) + [(ex + d * s * math.cos(eth), ey + d * s * math.sin(eth), eth) for s in [0.01 * i for i in range(1, int(ext / 0.01) + 1)]]
    ctrl = "Forward" if d > 0 else "Reverse"

    def make_goal(k0):   # 11-19 HOLD 뒤 이어갈 때는 진행 인덱스부터 (지나온 부분은 안 보냄)
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = node.get_clock().now().to_msg()
        for x, y, th in full[k0:]:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = yaw_q(th)
            path.poses.append(ps)
        g = FollowPath.Goal()
        g.path = path
        g.controller_id = ctrl
        g.goal_checker_id = "piece_checker"
        if hasattr(g, "progress_checker_id"):
            g.progress_checker_id = "progress_checker"
        return g
    ux, uy = d * math.cos(eth), d * math.sin(eth)

    def passed():
        p = node.pose()
        if p is None:
            return False
        rx, ry = p[0] - ex, p[1] - ey
        return rx * ux + ry * uy >= -0.003 and math.hypot(rx, ry) < 0.2
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))

    def dev():   # 조각 경로 (연장 포함) 에서 가장 가까운 점까지 [m]
        p = node.pose()
        return None if p is None else min(math.hypot(q[0] - p[0], q[1] - p[1]) for q in full)
    d0 = dev() or 0.0
    lim = max(OFF_PATH, d0 + 0.03)
    last = [0.0]
    # 11-18 속도 곡선: 조각 끝 = 정지 (구간 끝이면 마지막 STOP_CREEP_D 천천히, 중간이면 그만큼 더해 보내 그 구간을 건너뜀)
    cum = [0.0]
    for a, b in zip(poses, poses[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    # 11-19: 진행 인덱스 = 조각 dict 의 "_near" (follow_piece 밖 우회 · 되돌아가기 · 합류도 이 근처 앞쪽만 찾음).
    #   node 하나에 두면 우회 조각을 따라가는 동안 원래 조각 값이 덮이므로 조각마다 둠. 다시 따라갈 때는 되돌아간 만큼 (≤ BACK_MAX) 뒤까지 찾음
    near = [k_start]   # 방향 맞추기와 같은 시작 인덱스 (Path 는 make_goal(k_start - 2) 부터)
    pc["_near"] = near[0]
    t_pub = [0.0]

    def send_stop():
        if time.time() - t_pub[0] < 0.05:
            return
        t_pub[0] = time.time()
        p = node.pose()
        if p is None:
            return
        i = nearest_ahead(poses, p, near[0])   # 앞으로만 찾음 (경로가 겹쳐도 엉뚱한 점을 안 고름)
        near[0] = pc["_near"] = i
        pub_stop(node, max(cum[-1] - cum[i], 0.0) + (0.0 if final else STOP_CREEP_D))

    t_start = time.time()
    test_fail = (os.environ.get("DEMO_TEST_PIECE_FAIL") == "1" and pc["kind"] == "F" and length > 0.6 and not pc.get("_tested")
                 and not getattr(node, "test_off", False))   # 11-21 (2) auto 복귀 단계는 시험 실패 안 함 (곳 건너뛰기 · 복귀를 같이 보려고)

    def off_path():
        if test_fail and time.time() - t_start > 3.0:   # 시험용: 되찾기 확인 (DEMO_TEST_PIECE_FAIL=1, 조각마다 처음 한 번)
            pc["_tested"] = True
            return "시험용 실패 (DEMO_TEST_PIECE_FAIL=1)"
        if OFF_PATH <= 0 or time.time() - last[0] < 0.1:
            return None
        last[0] = time.time()
        d = dev()
        if d is not None and d > lim:
            return f"경로에서 {d * 100:.0f} cm 벗어남 (한도 {lim * 100:.0f} cm) — 멈춤 (소프트웨어 정지)"
        return None
    # 11-19 정지 래치 (HOLD): collision_monitor STOP 을 처음 받으면 FollowPath 를 바로 취소 (Nav2 가 0 → 섬).
    #   그대로 두면 RPP 가 더 느린 명령을 내고 collision_monitor 가 그 명령을 더 좁은 띠로만 봐서 다시 통과 → 정지 · 이어감 반복하며 다가감
    #   (11-19 팀원 A 시뮬: 앞 끝 ↔ 장애물 2.1 cm, 한 번 0 cm). HOLD 동안은 라이다를 직접 봐서 진행 방향 띠가 1 s 비면 진행 인덱스부터 이어감,
    #   BLOCK_WAIT 안에 안 비면 BLOCKED (→ run_leg 후진 · 우회). 이어간 뒤 HOLD_RUN_OK 넘게 달리기 전에 또 멈추면 기다림 시계는 이어서 셈
    test_block = (os.environ.get("DEMO_TEST_BLOCK") == "1" and pc["kind"] == "F" and length > 0.6 and not TEST_BLOCK["done"])
    run = {"t": t_start}
    hold_t0 = None

    def stop_seen():
        now = time.time()
        if test_block and now - t_start > 3.0:   # 시험용: 우회 흐름 확인 (DEMO_TEST_BLOCK=1, 전체에서 한 번)
            TEST_BLOCK["done"] = True
            print(f"    ⏸ 장애물 — 시험용 (DEMO_TEST_BLOCK=1): {BLOCK_WAIT:.0f} s 넘게 막힌 것으로 침", flush=True)
            return f"{BLOCKED}: 시험용 막힘 (DEMO_TEST_BLOCK=1)"
        st = node.cm_since
        if st is None:
            return None
        if st < run["t"] and now - run["t"] < CM_STALE:   # 보내기 전부터 남은 STOP (상태는 바뀔 때만 옴) — 잠깐은 새 명령의 판정을 기다림
            return None
        return HOLD

    def abort():
        send_stop()
        return off_path() or stop_seen()
    try:
        while True:
            k0 = max(pc["_near"] - 2, 0)
            run["t"] = time.time()
            r, msg = run_action(node.fp, make_goal(k0), (cum[-1] - cum[k0]) / 0.03 + 30, done_when=passed, abort_when=abort)
            if r or msg != HOLD:
                return r, msg
            now = time.time()
            if hold_t0 is None or now - run["t"] >= HOLD_RUN_OK:   # 새 HOLD
                hold_t0 = now
                node.hold_n += 1
                print(f"    ⏸ 장애물 — 정지 (HOLD, goal 취소), 최대 {BLOCK_WAIT:.0f} s 기다림 · 라이다 {'앞' if d > 0 else '뒤'} 띠 확인", flush=True)
            else:
                print(f"    ⏸ 장애물 — 다시 정지 (HOLD 이어서 {now - hold_t0:.1f} s 째)", flush=True)
            res, info = hold_wait(node, pc["kind"], hold_t0)
            w = time.time() - hold_t0
            node.tries.append({"status": "hold", "msg": f"{'전진' if d > 0 else '후진'} HOLD {w:.1f} s → "
                               + {"clear": "이어감", "blocked": "막힘", "stop": "Ctrl+C"}[res] + (f" ({info})" if info else ""),
                               "t": round(node.now_s() - node.leg_t0, 1)})
            if res == "clear":
                print(f"    ▶ 장애물 사라짐 — 이어감 ({w:.1f} s · {info})", flush=True)
                continue
            if res == "stop":
                return False, "Ctrl+C 로 멈춤"
            return False, f"{BLOCKED}: 장애물로 {w:.1f} s 막힘 (HOLD · {info})"
    finally:
        pub_stop(node, -1.0)   # 상한 끔 (다음 조각이 다시 냄)


def count_in_band(ranges, angle_min, angle_inc, rmin, rmax, tf, kind, front=None, rear=None, depth=None, half=None):
    """11-19 HOLD: 라이다 점 중 진행 방향 확인 띠 (F = 앞 끝 front ~ front + depth, R = 뒤 끝 -rear ~ -rear - depth, 폭 ±half) 안 개수.
    tf = base_link 에서 본 라이다 (x, y, yaw)"""
    front = BODY_FRONT if front is None else front
    rear = BODY_REAR if rear is None else rear
    depth = HOLD_DEPTH if depth is None else depth
    half = HOLD_HALF if half is None else half
    lo, hi = (front, front + depth) if kind == "F" else (-rear - depth, -rear)
    tx, ty, tth = tf
    c, s_ = math.cos(tth), math.sin(tth)
    n = 0
    for i, r in enumerate(ranges):
        if not (rmin <= r <= rmax) or math.isnan(r):   # inf · nan · 범위 밖은 버림
            continue
        a = angle_min + i * angle_inc
        px, py = r * math.cos(a), r * math.sin(a)
        bx, by = tx + c * px - s_ * py, ty + s_ * px + c * py
        if lo <= bx <= hi and abs(by) <= half:
            n += 1
    return n


def hold_wait(node, kind, t0):
    """11-19 HOLD 기다림: 진행 방향 띠 안 점이 HOLD_PTS 개 미만인 채 HOLD_CLEAR s 이어지면 ("clear", 설명).
    t0 부터 BLOCK_WAIT 넘으면 ("blocked", 설명). 라이다 · TF 를 못 받으면 풀지 않음 (안전 쪽). Ctrl+C → ("stop", "")"""
    clear_since, said, last = None, False, "점 모름"
    while True:
        if STOP.is_set():
            return "stop", ""
        now = time.time()
        if now - t0 > BLOCK_WAIT:
            return "blocked", last
        n, why = node.band_count(kind)
        if n is None:
            clear_since, last = None, why
            if not said:
                said = True
                print(f"    · HOLD: {why} — 풀지 않음 (안전 쪽)", flush=True)
        else:
            last = f"띠 안 점 {n} 개"
            if n < HOLD_PTS:
                clear_since = clear_since or now
                if now - clear_since >= HOLD_CLEAR:
                    return "clear", last
            else:
                clear_since = None
        time.sleep(0.05)


def relocalize(node, label):
    """멈춘 채 자기 위치 다시 찾기 (11-16): 실물이면 AMCL nomotion 갱신을 몇 번 요청 (움직이지 않아도 라이다로 다시 맞춤) → σ 를 알림"""
    sleep_s(node, 0.5)
    n = 0
    if not node.sim and node.nomotion.wait_for_service(timeout_sec=1.0):
        for _ in range(5):
            if STOP.is_set():
                break
            wait_future(node.nomotion.call_async(Empty.Request()), 2.0)
            n += 1
            sleep_s(node, 0.3)
    am = node.amcl
    sig = f"AMCL σ {am[0] * 100:.1f} cm · {am[1]:.1f}°" if am and time.time() - am[2] < 10 else ("시뮬 (AMCL 없음)" if node.sim else "AMCL σ 모름")
    print(f"    · {label}: 자기 위치 다시 찾기 (nomotion 갱신 {n} 번) · {sig}", flush=True)


def backtrack_piece(node, pc, max_len=None):
    """왔던 길 되돌아가기 조각 (11-16): 이 조각에서 이미 지나온 부분을 거꾸로, 최대 max_len (기본 BACK_MAX) m. 전진 조각이면 후진으로 (반대도)"""
    max_len = BACK_MAX if max_len is None else max_len
    p = node.pose()
    poses = pc["poses"]
    if p is None or len(poses) < 2 or max_len <= 0:
        return None
    k = nearest_ahead(poses, p, pc.get("_near", 0), ahead=120)   # 11-19: 전체 대신 진행 인덱스 근처 앞쪽만
    back, length = [tuple(poses[k])], 0.0
    for i in range(k - 1, -1, -1):
        step = math.hypot(poses[i][0] - back[-1][0], poses[i][1] - back[-1][1])
        if length + step > max_len:
            break
        back.append(tuple(poses[i])); length += step
    if length < 0.05:
        return None
    return {"kind": "R" if pc["kind"] == "F" else "F", "poses": back, "len": length}


def straight_back_piece(p, dist):
    """11-19: 지금 자세 p 에서 방향 그대로 곧게 dist m 후진하는 조각 (1 cm 간격)"""
    n = max(int(round(dist / 0.01)), 1)
    x, y, th = p
    poses = [(x - dist * i / n * math.cos(th), y - dist * i / n * math.sin(th), th) for i in range(n + 1)]
    return {"kind": "R", "poses": poses, "len": dist}


def detour_back(node, pc):
    """11-19 우회 전 후진 조각: 지나온 길 (조각 위 5 cm 안이면) 로 DETOUR_BACK, 그게 5 cm 보다 짧거나 조각에서 벗어나 있으면 곧게 DETOUR_BACK"""
    p = node.pose()
    if p is None:
        return None
    poses = pc["poses"]
    k = nearest_ahead(poses, p, pc.get("_near", 0), ahead=120)
    on = math.hypot(poses[k][0] - p[0], poses[k][1] - p[1]) < 0.05
    back = backtrack_piece(node, pc, DETOUR_BACK) if on else None
    return back if back is not None else straight_back_piece(p, DETOUR_BACK)


def path_len(poses):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))


def nearest_ahead(poses, p, i0, back=5, ahead=60):
    """11-19: 진행 인덱스 i0 근처 (i0 - back ~ i0 + ahead) 에서 p 에 가장 가까운 인덱스 — 경로가 겹쳐도 엉뚱한 점을 안 고름 (send_stop 과 같은 창)"""
    i0 = max(0, min(int(i0), len(poses) - 1))
    lo, hi = max(i0 - back, 0), min(i0 + ahead, len(poses))
    return min(range(lo, hi), key=lambda j: math.hypot(poses[j][0] - p[0], poses[j][1] - p[1]))


def start_heading(poses, cur, i0):
    """11-19: 조각을 따라가기 시작할 인덱스 (진행 인덱스 i0 근처, 되돌아간 만큼 뒤 START_BACK_N 개까지) 와 그 점 경로 방향 - 지금 방향 [rad]"""
    k = nearest_ahead(poses, cur, i0, back=START_BACK_N, ahead=60)
    return k, wrap(poses[k][2] - cur[2])


def join_candidates(poses, p, dists=DETOUR_JOIN, i0=0):
    """11-19 우회 합류점 후보 인덱스: 진행 인덱스 i0 근처 앞쪽에서 로봇 p 에 가장 가까운 점부터 경로 길이 dists 앞의 점들 + 조각 끝
    (남은 길이 < dists[0] 이면 끝만)"""
    if len(poses) < 2:
        return []
    k = nearest_ahead(poses, p, i0, back=30, ahead=120)   # back 30: 우회 전 0.15 m 후진한 만큼 뒤로도 찾음
    cum = [0.0]
    for a, b in zip(poses[k:], poses[k + 1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    last = len(poses) - 1
    if cum[-1] < dists[0] - 1e-6:
        return [last]
    out = []
    for d in dists:
        j = next((k + n for n, c in enumerate(cum) if c >= d - 1e-6), last)
        if j not in out:
            out.append(j)
    if last not in out:
        out.append(last)
    return out


def seg_dist(q, a, b):
    """점 q 에서 선분 a-b 까지 거리"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / L2))
    return math.hypot(q[0] - a[0] - t * dx, q[1] - a[1] - t * dy)


def detour_ok(path, piece_poses, dev_max=DETOUR_DEV_MAX):
    """11-19 우회 경로 받아들이기 (옆으로 지나갈 공간 검사): 점 2 개 이상 · 길이 ≤ 직선 × 3 + 0.5 m · 원래 조각에서 가장 먼 편차 ≤ dev_max.
    (True, "") 또는 (False, 이유)"""
    if len(path) < 2:
        return False, f"점 {len(path)} 개"
    L, straight = path_len(path), math.hypot(path[-1][0] - path[0][0], path[-1][1] - path[0][1])
    if L > straight * 3 + 0.5:
        return False, f"길이 {L:.2f} m > 직선 {straight:.2f} × 3 + 0.5"
    segs = list(zip(piece_poses, piece_poses[1:])) or [(piece_poses[0], piece_poses[0])]
    dev = max(min(seg_dist(q, a, b) for a, b in segs) for q in path)
    if dev > dev_max:
        return False, f"조각에서 {dev:.2f} m 벗어남 (한도 {dev_max:.1f})"
    return True, ""


def cut_piece(pc, j):
    """11-19 우회 뒤 남은 조각: poses[j:] (남은 길이 < 0.03 m 면 None = 다음 조각으로)"""
    rest = list(pc["poses"][j:])
    if len(rest) < 2 or path_len(rest) < 0.03:
        return None
    out = dict(pc, poses=rest)
    out.pop("_near", None)   # 인덱스가 바뀌므로 진행 인덱스는 새로 (합류점 = 0)
    return out


def request_path(node, goal, timeout=10.0):
    """ComputePathToPose 한 번 → (poses [(x, y, yaw)…], "") 또는 (None, 이유)"""
    gf = node.planner.send_goal_async(goal)
    if not wait_future(gf, 5.0):
        return None, "goal 응답 없음"
    gh = gf.result()
    if not gh.accepted:
        return None, "goal 거절"
    rf = gh.get_result_async()
    if not wait_future(rf, timeout):
        wait_future(gh.cancel_goal_async(), 2.0)
        return None, f"{timeout:.0f} s 안에 계획 못 함"
    res = rf.result()
    if res.status != 4:
        return None, f"상태 {res.status} · 오류 {getattr(res.result, 'error_code', 0)} {getattr(res.result, 'error_msg', '') or ''}".strip()
    out = []
    for ps in res.result.path.poses:
        q = ps.pose.orientation
        out.append((ps.pose.position.x, ps.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    return out, ""


def plan_detour(node, pc):
    """11-19 막힌 전진 조각의 우회: 합류점 후보마다 planner_server (Detour) 에 경로 요청 → 처음 받아들일 수 있는 것.
    (우회 조각, 합류 인덱스, 설명) 또는 (None, None, 이유)"""
    if not node.planner.wait_for_server(timeout_sec=1.0):
        if not node.planner_warned:
            node.planner_warned = True
            print("  ⚠ planner_server (compute_path_to_pose) 없음 — 우회 없이 되찾기 (SAFETY=0 실행?)", flush=True)
        return None, None, "planner_server 없음"
    p = node.pose()
    if p is None:
        return None, None, "map → base_link TF 없음"
    poses = pc["poses"]
    k = nearest_ahead(poses, p, pc.get("_near", 0), back=30, ahead=120)   # 후진한 자리 기준
    why = []
    for j in join_candidates(poses, p, i0=pc.get("_near", 0)):
        ahead = path_len(poses[k:j + 1])
        x, y, th = poses[j]
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.header.stamp = node.get_clock().now().to_msg()
        g.goal.pose.position.x, g.goal.pose.position.y = float(x), float(y)
        g.goal.pose.orientation.z, g.goal.pose.orientation.w = yaw_q(th)
        g.planner_id = "Detour"
        g.use_start = False
        path, msg = request_path(node, g)
        if path is not None:
            ok, msg = detour_ok(path, poses)
            if ok:
                return {"kind": "F", "poses": path, "len": path_len(path)}, j, f"{ahead:.2f} m 앞 합류"
        print(f"    · 우회 후보 {ahead:.2f} m 앞: {msg}", flush=True)
        why.append(f"{ahead:.1f} m: {msg}")
        if STOP.is_set():
            break
    return None, None, " · ".join(why) or "후보 없음"


def ask_choice(prompt):
    """되찾기도 안 될 때 (step 모드): Enter = 한 번 더 · s = 이 조각 건너뛰기 · q = 구간 끝"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            t = sys.stdin.readline().strip().lower()
            return "quit" if t == "q" else ("skip" if t == "s" else "retry")
    return "quit"


def ask(prompt):
    """Enter = 진행, q = 끝. Ctrl+C 도 끝"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            return sys.stdin.readline().strip().lower() != "q"
    return False


def ask_next(prompt):
    """Enter = 그대로 출발 → (True, None) · 이름 (L2 · box3 …) = 그곳 먼저 → (True, 이름) · q = 끝 → (False, None)"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            t = sys.stdin.readline().strip()
            if t.lower() == "q":
                return False, None
            return True, (norm(t) if t else None)
    return False, None


def confirm(prompt):
    """y = 진행, 그 밖 (q · 빈 Enter) = 안 함. Ctrl+C 도 안 함"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            return sys.stdin.readline().strip().lower() == "y"
    return False


def main():
    global TURN_DIRECT, TURN_ROOM, AP_LEAVE_ALL, AP_MID_MAX, AP_MID_TOL, AP_MEASURE_D, AP_MEASURE_GAP   # 11-21 (3) — 아래 help 문자열이 기본값을 읽으므로 맨 위에
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--step", action="store_true")
    ap.add_argument("--set-initial", action="store_true")
    ap.add_argument("--legs", default="")
    ap.add_argument("--no-head-fix", action="store_true", help="조각 시작 방향 맞추기 끔 (11-14 판처럼)")
    ap.add_argument("--off-path", type=float, default=None, help="경로 이탈 멈춤 한도 [m] (기본 0.10, 0 = 끔)")
    ap.add_argument("--no-recover", action="store_true", help="조각 실패 시 되찾기 끔 (11-16 전처럼)")
    ap.add_argument("--no-detour", action="store_true", help="장애물 막힘 우회 끔 (막히면 기존 되찾기, 환경 DEMO_DETOUR=0 과 같음)")
    ap.add_argument("--scan-topic", default="/scan_merged", help="정지 래치 (HOLD) 때 앞 · 뒤 띠를 볼 라이다 (시뮬 /scan)")
    ap.add_argument("--box-quiet-m", type=float, default=BOX_QUIET_M,
                    help=f"11-21: 상자 정지점 이 거리 [m] 안이면 정지 띠 · HOLD 에서 그 상자 점만 뺌 (기본 {BOX_QUIET_M}, 0 = 기능 끔 = 상자에도 멈춤)")
    ap.add_argument("--box-approach", choices=["on", "off", "dry"], default="on",
                    help="11-21: 상자 정지점에서 앞 라이다로 상자 앞면을 재고 상판 앞 끝 ↔ 상자 --box-gap 자리로 다가감 (on 기본 · off · dry = 재기만)")
    ap.add_argument("--box-approach-at", default="", help="상자 다가가기 할 곳 (쉼표 목록, 예 box1,L2 — 비우면 box1 ~ 6 · L1 ~ 3 전부)")
    ap.add_argument("--box-gap", type=float, default=0.05, help="상자 다가가기 목표: 상판 앞 끝 ↔ 상자 앞면 [m] (기본 0.05)")
    ap.add_argument("--box-scan-topic", default="/scan_front", help="상자를 잴 원본 앞 라이다 (차체 가림 지우기 전, 기본 /scan_front)")
    ap.add_argument("--box-loose", action="store_true",
                    help="상자 측정 느슨한 기준 (bf.FACE_CFG_LOOSE · bf.AVG_LOOSE — 실물 잡음 조정용, 퍽 2 cm 틈 구분 포기)")
    ap.add_argument("--odom-topic", default="", help="상자 다가가기 회전 · 거리 (기본: 실물 /odometry/filtered · 시뮬 /odom)")
    # 11-21 (2) 자율 순회 · 다가가기 보강
    ap.add_argument("--auto", action="store_true",
                    help="자율 순회: 계획 순서대로 모든 곳 → 출발 자세 복귀. 실패한 곳은 건너뜀 · 연속 --auto-fail-home 곳 실패면 바로 복귀 (계획 legs 파일만)")
    ap.add_argument("--dwell", type=float, default=3.0, help="auto: 정지점마다 머무는 시간 [s] (step 이면 안 씀, 기본 3)")
    ap.add_argument("--auto-fail-home", type=int, default=2, help="auto: 연속 이만큼 곳이 실패하면 바로 복귀 (기본 2)")
    ap.add_argument("--home-tol", default="0.03,5", help="auto: 출발 자세 복귀 판정 m,deg (기본 0.03,5)")
    ap.add_argument("--from", dest="from_leg", default=None,
                    help="auto: 이 곳 (이름 box3 · L2) 또는 구간 번호 (3) 부터 — 출발은 출발 자세, 첫 곳이 멀면 planner 길, 끝나면 복귀. --legs 와 같이 주면 그 범위 안에서")
    ap.add_argument("--ap-return-stop", action=argparse.BooleanOptionalAction, default=True,
                    help="상자 다가가기 done 뒤 정지점이 뒤쪽이면 곧게 후진해 정지점으로 (상자 쪽으로는 안 감, 기본 켬)")
    ap.add_argument("--ap-cut-yaw", type=float, default=20.0,
                    help="끊는 점 방향이 정지점과 이 각 [deg] 넘게 다르면 끊지 않고 도착 뒤 후진해서 잼 (기본 20, 0 = 예전처럼 늘 끊음)")
    # 11-21 (3) 상자 앞 Spin "Collision Ahead" 대책 · 다가가기 S 자 보정
    ap.add_argument("--turn-direct", type=float, default=math.degrees(TURN_DIRECT),
                    help=f"이 각 [deg] 이하 제자리 회전은 Nav2 Spin 대신 cmd_vel 직접 (오도메트리 되먹임). 상자 앞 (bf.turn_clear 됨) 은 {math.degrees(TURN_DIRECT_NEAR):.0f}° 까지 (기본 {math.degrees(TURN_DIRECT):.0f}, 0 = 끔 = 늘 Spin)")
    ap.add_argument("--turn-room", action=argparse.BooleanOptionalAction, default=TURN_ROOM,
                    help="계획 제자리 조각 전 상자 앞이면 틈 확보 후진 (틈 ≥ 0.12 + bf.turn_clear, 기본 켬)")
    ap.add_argument("--ap-leave-all", action=argparse.BooleanOptionalAction, default=AP_LEAVE_ALL,
                    help="다가가기가 안 된 상자 정지점에서도 떼기 후진 (잰 상자면 · 없으면 발제 좌표, 기본 켬). --no-ap-return-stop 이면 함께 꺼짐")
    ap.add_argument("--ap-mid-fix", type=int, default=AP_MID_MAX,
                    help=f"다가가기 0.20 m 자리 다시 잰 뒤 S 자 보정 최대 횟수 (기본 {AP_MID_MAX}, 0 = 예전처럼 확인만 · 3 cm 넘으면 그만둠)")
    ap.add_argument("--ap-mid-tol", type=float, default=AP_MID_TOL * 1000, help=f"S 자 보정 허용 옆 오차 [mm] (기본 {AP_MID_TOL * 1000:.0f})")
    ap.add_argument("--ap-measure-gap", type=float, default=AP_MEASURE_GAP * 100,
                    help=f"상자 측정 자리 = 측정할 때 상판 앞 끝 ↔ 상자 틈 [cm] (기본 {AP_MEASURE_GAP * 100:.1f}, 앞면 x 0.26~0.29 — 상자 점이 충분히 보임). 정지점 앞 끊는 거리 = 이 값 − 정지점 틈 {STOP_GAP * 100:.1f} cm. {AP_CLOSE_GAP * 100:.0f} cm 이하면 S 자 한 번으로 5 cm 까지")
    a = ap.parse_args()
    AP_MEASURE_GAP = min(max(a.ap_measure_gap, 8.0), 50.0) / 100.0
    AP_MEASURE_D = max(AP_MEASURE_GAP - STOP_GAP, 0.0)
    global AP_CUT_YAW, AP_RETURN_STOP, AUTO_MODE
    TURN_DIRECT, TURN_ROOM, AP_LEAVE_ALL = math.radians(max(a.turn_direct, 0.0)), a.turn_room, a.ap_leave_all
    AP_MID_MAX, AP_MID_TOL = max(a.ap_mid_fix, 0), max(a.ap_mid_tol, 1.0) / 1000.0
    AP_CUT_YAW, AP_RETURN_STOP, AUTO_MODE = math.radians(max(a.ap_cut_yaw, 0.0)), a.ap_return_stop, a.auto
    try:
        a.home_tol_xy, a.home_tol_yaw = (float(v) for v in a.home_tol.split(","))
    except ValueError:
        raise SystemExit(f"--home-tol {a.home_tol} 는 'm,deg' (예 0.03,5)")
    if not a.auto and a.from_leg is not None:
        print("⚠ --from 은 --auto 에서만 — 무시", flush=True)
    AP["mode"], AP["gap"], AP["loose"] = a.box_approach, a.box_gap, a.box_loose
    want = {norm(x.replace(" ", "")) for x in a.box_approach_at.split(",") if x.strip()}   # 대소문자 · 공백 무시 (norm: box* 소문자 · L* 대문자)
    bad = sorted(want - set(NAMES[1:]))
    if bad:
        print(f"⚠ --box-approach-at 에 모르는 이름 {bad} — 무시 (쓸 수 있는 이름: {' '.join(NAMES[1:])})", flush=True)
    AP["at"] = want - set(bad)
    if want and not AP["at"]:
        print("⚠ --box-approach-at 에 쓸 이름이 없음 — 상자 다가가기 끔", flush=True)
        AP["mode"] = "off"
    if not 0.0 <= a.box_gap <= 0.15:
        raise SystemExit(f"--box-gap {a.box_gap} 는 0 ~ 0.15 m 이어야 함")
    if bf is not None:
        bf.FRONT_OFFSET, bf.BOX_SIDE = AP_FRONT, AP_BOX_SIDE
    if a.no_detour:
        global DETOUR_ON
        DETOUR_ON = False
    if a.off_path is not None:
        global OFF_PATH
        OFF_PATH = a.off_path
    global RECOVER_TRIES, STEP_MODE
    STEP_MODE = a.step
    if a.no_recover:
        RECOVER_TRIES = 0
    if a.no_head_fix:
        global HEAD_FIX
        HEAD_FIX = 0.0
    data = json.load(open(a.plan, encoding="utf-8"))
    route = data.get("mode") == "route"
    if route and a.auto:
        raise SystemExit("--auto 는 계획 (legs) 파일만 — route 파일 (mode route) 은 안 됨")
    if route:   # 11-15 기본 경로 + 들르기: 길은 길 모음에서 그때그때 꺼냄
        lp = data.get("lib", "legs.json")
        lp = lp if os.path.isabs(lp) else os.path.join(os.path.dirname(os.path.abspath(a.plan)), lp)
        lib = json.load(open(lp, encoding="utf-8"))
        LIB, PTS = {g["name"]: g for g in lib["legs"] if g.get("ok")}, lib["points"]
        bad = [x for x in data["route"] if x not in NAMES]
        if bad:
            raise SystemExit(f"모르는 이름 {bad}")
        data["start"], data["legs"] = PTS["HOME"]["start"], []
        data.setdefault("course", data["plan"] + ".yaml")
        legs, start = [], data["start"]
    else:
      legs = data["legs"]
      if a.legs:
        lo, _, hi = a.legs.partition("-")
        keep = set(range(int(lo), int(hi or lo) + 1))
        legs = [g for g in legs if g["leg"] in keep]
      if a.auto and a.from_leg is not None:
        f = a.from_leg.strip()
        k0 = int(f) if f.isdigit() else next((g["leg"] for g in legs if leg_target(g) == norm(f)), None)
        if k0 is None:
            raise SystemExit(f"--from {f}: 계획에 없는 곳 (있는 곳: {' '.join(leg_target(g) for g in legs)})")
        legs = [g for g in legs if g["leg"] >= k0]
      if not legs:
        raise SystemExit(f"고른 구간이 없음 (--legs {a.legs or '-'} · --from {a.from_leg})")
      first = legs[0]["leg"]
      start = data["start"] if first == data["legs"][0]["leg"] else data["legs"][first - 1]["goal"]
      if a.auto and first == data["legs"][0]["leg"]:   # 11-21 (2): 처음부터면 출발 자세 (home_pose) 에서.
        start = list(data.get("home_pose") or data["start"])   #   --from 재개는 위 그대로 직전 정지점 (계획 길을 그대로 씀, 사람이 그 자리에 놓고 init)

    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    global LIVE
    os.makedirs(a.out, exist_ok=True)
    LIVE = open(os.path.join(a.out, "track_live.csv"), "w", encoding="utf-8", buffering=1)
    LIVE.write("t,x,y,yaw,v,w,step,v_nav,w_nav,loc\n")
    rclpy.init()
    node = Follower(a.sim, a.scan_topic)
    node.quiet_m = max(a.box_quiet_m, 0.0)
    odom_topic = a.odom_topic or ("/odom" if a.sim else "/odometry/filtered")
    node.setup_approach(a.box_scan_topic, odom_topic)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    def _spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001  (끝낼 때 rclpy 종료와 겹치면 나는 오류 — 기록에는 영향 없음)
            pass
    threading.Thread(target=_spin, daemon=True).start()
    print(f"출발 자세 (계획) {start[0]:.3f}, {start[1]:.3f}, {math.degrees(start[2]):.0f}° · "
          + (f"기본 경로 {' → '.join(data['route'])} · 들르기 요청 /drive_demo/detour" + (" 또는 step 입력" if a.step else "")
             if route else f"구간 {legs[0]['name']} ~ {legs[-1]['name']}"), flush=True)
    print("상자 구간: " + (f"상자 정지점 {node.quiet_m:.2f} m 안 (끝 {node.quiet_m + BOX_QUIET_HYST:.2f} m 밖) — 정지 띠 · HOLD 는 상자 점 뺀 라이다 ({a.scan_topic}) · "
                            f"{BOX_CAP_M:.2f} m 안 속도 ≤ {ZONE_V:.2f} m/s" if node.quiet_m > 0 else "꺼짐 (--box-quiet-m 0, 상자에도 멈춤)"), flush=True)
    print("상자 다가가기: " + ("꺼짐 (--box-approach off)" if AP["mode"] == "off" else f"⚠ 꺼짐 (box_face.py 못 읽음: {BF_ERR})" if bf is None else
                              f"{'dry (재기만, 조각 안 끊음)' if AP['mode'] == 'dry' else '켬'} · {' '.join(sorted(AP['at'])) or 'box1 ~ 6 · L1 ~ 3'} · "
                              f"상판 앞 끝 ↔ 상자 {AP['gap'] * 100:.0f} cm · 정지점 {AP_MEASURE_D:.2f} m 앞에서 잼 (마지막이 전진 아니면 최대 {AP_BACK_MAX:.2f} m 후진) · "
                              f"라이다 {a.box_scan_topic} · 오도메트리 {odom_topic}" + (" · 느슨한 기준 (--box-loose)" if AP["loose"] else "")), flush=True)

    # 출발 확인
    t0 = time.time()
    if a.sim:
        while not STOP.is_set():
            node.set_initial(*start)
            time.sleep(1.0)
            p = node.pose()
            if p and math.hypot(p[0] - start[0], p[1] - start[1]) < 0.01:
                break
            if time.time() - t0 > 90:
                print("✗ 시뮬 TF 가 시작 자세로 안 옴", flush=True)
                STOP.set()
    else:
        if a.set_initial:
            time.sleep(1.0)
            node.set_initial(*start)
            print("  /initialpose = 출발 자세 (한 번)", flush=True)
            time.sleep(2.0)
        # 출발 조건 (11-15): 위치 감시가 떠 있으면 상태 OK 만 (SUSPECT · 상태 없음은 안 됨),
        #   꺼져 있으면 AMCL 자세가 있고 σ ≤ 5 cm · 5° + 사람이 RViz 로 라이다 ↔ 벽 겹침 확인 (y)
        why = ""
        while not STOP.is_set():
            p = node.pose()
            live = node.loc_live()
            am = node.amcl
            if live:
                loc_ok = node.loc == "OK"
                loc_txt = f"위치 감시 {node.loc or '상태 아직 없음'}"
            else:
                loc_ok = am is not None and am[0] <= AMCL_XY_OK and am[1] <= AMCL_YAW_OK
                loc_txt = "위치 감시 꺼짐 · AMCL " + (f"σ {am[0] * 100:.1f} cm · {am[1]:.1f}°" if am else "자세 아직 없음")
            if p is None:
                why = "map → base_link TF 없음 (위치 추정이 안 떴거나 초기 자세 없음)"
                print(f"\r  {why} · {loc_txt}   ", end="", flush=True)
            else:
                dp, dy = math.hypot(p[0] - start[0], p[1] - start[1]), abs(wrap(p[2] - start[2]))
                print(f"\r  지금 위치 ({p[0]:.3f}, {p[1]:.3f}, {math.degrees(p[2]):.0f}°) · 출발과 {dp * 100:.1f} cm · {math.degrees(dy):.1f}° · "
                      f"{loc_txt}   ", end="", flush=True)
                if dp < START_TOL and dy < START_YAW_TOL and loc_ok:
                    print("", flush=True)
                    break
                why = (f"출발 자리와 {dp * 100:.0f} cm · {math.degrees(dy):.0f}° (한계 {START_TOL * 100:.0f} cm · {math.degrees(START_YAW_TOL):.0f}°)"
                       if not (dp < START_TOL and dy < START_YAW_TOL) else
                       f"위치 감시 상태가 OK 아님 ({node.loc or '없음'})" if live else
                       "AMCL 자세 없음 — RViz 2D Pose Estimate 또는 run_demo.sh … init" if am is None else
                       f"AMCL σ {am[0] * 100:.1f} cm · {am[1]:.1f}° 가 한계 ({AMCL_XY_OK * 100:.0f} cm · {AMCL_YAW_OK:.0f}°) 넘음 — 조금 기다리거나 초기 자세 다시")
            if time.time() - t0 > 60:
                print(f"\n✗ 60 s 안에 출발 조건이 안 맞음: {why}", flush=True)
                STOP.set()
            time.sleep(0.5)
        if not STOP.is_set() and not node.loc_live():
            if not confirm("  위치 감시가 꺼져 있음 — RViz 에서 라이다 점이 지도 벽과 겹치면 y + Enter (아니면 q) : "):
                print("✗ 출발 안 함 (위치 확인 안 됨)", flush=True)
                STOP.set()
    if not STOP.is_set():
        for c, n in ((node.fp, "follow_path"), (node.spin, "spin")):
            if not c.wait_for_server(timeout_sec=30.0):
                print(f"✗ {n} action server 없음 (run_demo.sh 의 Nav2 노드가 떴는지)", flush=True)
                STOP.set()

    results = []
    try:
        if route:
            run_route(a, node, data, LIB, PTS, results)
        elif a.auto:
            run_auto(a, node, data, legs, results)
        else:
            for g in legs:
                if STOP.is_set():
                    break
                if a.step and not ask(f"\n▶ {g['name']} — Enter 출발 · q 끝 : "):
                    break
                if not run_leg(node, g, results):
                    break
    finally:
        node.quiet_close()   # 11-20: 정지 띠를 끈 채 끝나지 않게 (정상 · Ctrl+C · 예외)
    node.step = -1
    time.sleep(1.0)
    with node.lock:
        rows = list(node.rows)
    ex.shutdown()
    rclpy.try_shutdown()
    save(a, data, results, rows, len(data["legs"]) if route else len(legs))



# ───────────── 11-21 상자 다가가기 (box approach) ─────────────
def ap_target(g):
    """상자 다가가기 대상 구간이면 목적지 이름, 아니면 None (모드 off · box_face 없음 · --box-approach-at 밖)"""
    if AP["mode"] == "off" or bf is None or box_goal(g) is None:
        return None
    t = leg_target(g)
    if AP["at"] and t not in AP["at"]:
        return None
    return t


def ap_split(pieces, d=None):
    """마지막 전진 조각을 끝 d m 앞에서 둘로: (앞부분 _stop · 나머지 _box_tail) 로 바꾼 조각 목록, 끊은 길이. 못 끊으면 (None, 0)"""
    d = AP_MEASURE_D if d is None else d   # main 이 바꾼 값을 쓰게 (기본 인자는 정의 때 값으로 굳음)
    if not pieces or pieces[-1]["kind"] != "F" or len(pieces[-1].get("poses") or []) < 3:
        return None, 0.0
    poses = pieces[-1]["poses"]
    cum = [0.0]
    for a, b in zip(poses, poses[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    if cum[-1] < d + AP_CUT_MIN:
        return None, 0.0
    j = next(k for k, c in enumerate(cum) if c >= cum[-1] - d)
    j = min(max(j, 1), len(poses) - 2)
    head = {k: v for k, v in pieces[-1].items() if k != "_near"}
    tail = dict(head, poses=list(poses[j:]), _box_tail=True)
    head = dict(head, poses=list(poses[:j + 1]), _stop=True)
    return pieces[:-1] + [head, tail], cum[-1] - cum[j]


def ask_run(prompt):
    """step 모드 기동 확인: Enter = 실행 → True, s · q = 건너뛰기 → False. Ctrl+C 도 False"""
    print(prompt, end="", flush=True)
    while not STOP.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            return sys.stdin.readline().strip().lower() not in ("s", "q")
    return False


def ap_tf6(node, frame):
    """base_link ← 라이다 frame 정적 TF 의 2D 부분 (tx, ty, r00, r01, r10, r11) — 회전행렬 전체에서 (roll π 장착도 맞게). 없으면 None"""
    if frame not in node.ftf:
        try:
            t = node.tf.lookup_transform("base_link", frame, Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        node.ftf[frame] = (t.transform.translation.x, t.transform.translation.y,
                           1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * y + z * w), 1 - 2 * (x * x + z * z))
    return node.ftf[frame]


def ap_cmd(node, v, w):
    m = TwistStamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = "base_link"
    m.twist.linear.x, m.twist.angular.z = float(v), float(w)
    node.ap_pub.publish(m)


def ap_zero(node):
    for _ in range(3):
        ap_cmd(node, 0.0, 0.0)
        time.sleep(0.03)


def ap_guard(node, scan=True):
    """기동 중 매 주기: Ctrl+C · 전체 시간 상한 · 오도메트리 끊김 · (scan 이면) 앞 라이다 끊김 → ApFail. 지금 오도메트리 (x, y, yaw, t)"""
    if STOP.is_set():
        raise ApFail("Ctrl+C")
    o, s, now = node.odom, node.fscan, time.time()
    dl = getattr(node, "ap_deadline", None)
    if dl is not None and now > dl:
        raise ApFail(f"다가가기 전체 {AP_TOTAL_T:.0f} s 넘음")
    if o is None or now - o[3] > ODOM_STALE:
        raise ApFail(f"오도메트리 끊김 ({ODOM_STALE:.1f} s 넘게)")
    if scan and (s is None or now - s[0] > AP_SCAN_STALE):
        raise ApFail(f"앞 라이다 끊김 ({AP_SCAN_STALE:.1f} s 넘게 새 스캔 없음)")
    return o


def ap_face_now(F, o):
    """상자 앞면 F (odom) 를 지금 base_link 에서 본 FaceResult (bf.turn_clear 용)"""
    cx, cy = ap_expect_odom(F, o)
    return bf.FaceResult(True, "", cx, cy, wrap(F[2] - o[2]), AP_BOX_SIDE)


def ap_turn(node, target, F=None, scan=True, min_gap=None):
    """오도메트리 yaw 를 target (odom) 으로 제자리 회전 (ICR 둘레). 끝 오차 [rad].
    F (상자 앞면, odom) 가 있으면 먼저 틈 ≥ AP_TURN_GAP 이고 bf.turn_clear (앞 모서리가 상자 평면을 안 넘음) 일 때만 돎.
    scan=False (11-21 (3) 직접 회전 turn_safe) 면 앞 라이다 끊김은 안 봄"""
    o = ap_guard(node, scan)
    if F is not None:
        ang = wrap(target - o[2])
        gap = ap_rel(F, o)[0]
        if abs(ang) >= AP_YAW_TOL and gap < (AP_TURN_GAP if min_gap is None else min_gap):
            raise ApFail(f"회전 {math.degrees(ang):+.1f}° 안 함: 틈 {gap * 100:.1f} cm < {AP_TURN_GAP * 100:.0f} cm")
        if abs(ang) >= AP_YAW_TOL and not bf.turn_clear(ap_face_now(F, o), ang):
            raise ApFail(f"회전 {math.degrees(ang):+.1f}° 안 함: 앞 모서리가 상자면에 닿을 수 있음 (bf.turn_clear)")
    t0 = time.time()
    best, t_best, settle = None, t0, 0
    while True:
        o = ap_guard(node, scan)
        err = wrap(target - o[2])
        if abs(err) < AP_YAW_TOL:
            ap_cmd(node, 0.0, 0.0)
            settle += 1
            if settle >= 3:
                return err
        else:
            settle = 0
            ap_cmd(node, 0.0, math.copysign(min(AP_W_MAX, max(AP_W_MIN, AP_KP_YAW * abs(err))), err))
        now = time.time()
        if best is None or abs(err) < best - math.radians(0.5):
            best, t_best = abs(err), now
        elif now - t_best > AP_STUCK_T:
            raise ApFail(f"회전이 안 됨 (남은 {math.degrees(err):+.1f}°, v2_filter 제자리 90° 예산?)")
        if now - t0 > AP_TURN_T:
            raise ApFail(f"회전 {AP_TURN_T:.0f} s 넘음 (남은 {math.degrees(err):+.1f}°)")
        time.sleep(AP_DT)


def ap_drive(node, dist, heading, vmax, scan=True):
    """heading (odom) 방향을 유지하며 곧게 dist m (음수 = 후진), 오도메트리 거리로 멈춤. 남은 거리 [m] (음수 = 지나침).
    scan=False (마지막 직진, 상자가 라이다 최소 거리 안) 면 라이다 끊김은 안 보고 누적 이동이 dist + AP_OVER 를 넘으면 그만둠"""
    o0 = ap_guard(node, scan)
    L, sg = abs(dist), (1.0 if dist >= 0 else -1.0)
    if L < AP_POS_TOL:
        return 0.0
    hx, hy = math.cos(heading), math.sin(heading)
    t0 = time.time()
    best, t_best = 0.0, t0
    limit = L / (0.5 * vmax) + 8.0
    while True:
        o = ap_guard(node, scan)
        s = ((o[0] - o0[0]) * hx + (o[1] - o0[1]) * hy) * sg
        rem = L - s
        if s > L + AP_OVER:
            raise ApFail(f"{'전진' if sg > 0 else '후진'} {s * 100:.1f} cm 가 계획 {L * 100:.1f} + {AP_OVER * 100:.0f} cm 넘음")
        if rem <= AP_POS_TOL:
            ap_zero(node)
            return rem
        v = min(vmax, max(AP_V_MIN, AP_KP_LIN * rem))
        w = max(-AP_HOLD_W, min(AP_HOLD_W, AP_KP_HOLD * wrap(heading - o[2])))
        ap_cmd(node, sg * v, w)
        now = time.time()
        if s > best + 0.002:
            best, t_best = s, now
        elif now - t_best > AP_STUCK_T:
            raise ApFail(f"{'전진' if sg > 0 else '후진'}이 안 됨 ({s * 100:.1f} / {L * 100:.1f} cm, v2_filter 후진 한도?)")
        if now - t0 > limit:
            raise ApFail(f"{'전진' if sg > 0 else '후진'} {limit:.0f} s 넘음 ({s * 100:.1f} / {L * 100:.1f} cm)")
        time.sleep(AP_DT)


def ap_face_frame(face, o):
    """측정 (base_link) → 상자 앞면 중앙 · 법선 (o 프레임): (x, y, nyaw). nyaw = 로봇이 바라볼 방향 (로봇 → 상자). o = (x, y, yaw, …)"""
    ny = face.nyaw
    if math.cos(ny) * face.cx + math.sin(ny) * face.cy < 0:   # 법선이 상자 쪽을 보게 (box_face 규약이 반대여도)
        ny += math.pi
    c, s = math.cos(o[2]), math.sin(o[2])
    return o[0] + c * face.cx - s * face.cy, o[1] + s * face.cx + c * face.cy, wrap(o[2] + ny)


def ap_rel(F, o):
    """상자 앞면 F (x, y, nyaw) 와 로봇 o (x, y, yaw) → (간격 = 상판 앞 끝 ↔ 앞면 법선 거리, 중앙선 어긋남 (+ = 중앙선이 로봇 왼쪽), 방향 오차 nyaw − yaw)"""
    dx, dy = F[0] - o[0], F[1] - o[1]
    nx, ny = math.cos(F[2]), math.sin(F[2])
    return dx * nx + dy * ny - AP_FRONT, -dx * ny + dy * nx, wrap(F[2] - o[2])


def ap_txt(gap, lat, yaw):
    return f"앞 끝 ↔ 상자 {gap * 100:.1f} cm · 중앙선 {lat * 100:+.1f} cm · 방향 {math.degrees(yaw):+.1f}°"


def ap_q(face, n=None):
    """측정 품질 꼬리: 스캔 장수 · 점 수 · RMS (실물 dry 로 라이다 잡음 판단) · 느슨한 기준 표시"""
    return ((f"스캔 {n} 장 · " if n else "") + f"점 {face.n_pts} · RMS {getattr(face, 'rms', 0.0) * 1000:.1f} mm"
            + (" · 느슨한 기준" if AP["loose"] else ""))


def ap_rec(gap, lat, yaw, **kw):
    return dict({"gap_cm": round(gap * 100, 1), "lat_cm": round(lat * 100, 1), "yaw_deg": round(math.degrees(yaw), 1)}, **kw)


def ap_expect_goal(g, p):
    """정지점 계획 (앞 끝 ≈ 상자) 으로 본 상자 앞면 중앙 예상 (base_link x, y). p = map 자세"""
    gx, gy, gth = g["goal"]
    if p is None or gth is None:
        return None
    sg = (g.get("box") or {}).get("gap", STOP_GAP)   # 정지점에서 앞 끝 ↔ 상자 면
    fx, fy = gx + (AP_FRONT + sg) * math.cos(gth), gy + (AP_FRONT + sg) * math.sin(gth)
    dx, dy = fx - p[0], fy - p[1]
    c, s = math.cos(p[2]), math.sin(p[2])
    return c * dx + s * dy, -s * dx + c * dy


def ap_expect_odom(F, o):
    """오도메트리로 예측한 상자 앞면 중앙 (base_link x, y)"""
    dx, dy = F[0] - o[0], F[1] - o[1]
    c, s = math.cos(o[2]), math.sin(o[2])
    return c * dx + s * dy, -s * dx + c * dy


def ap_face_of(node, m, expect):
    tf6 = ap_tf6(node, m.header.frame_id)
    if tf6 is None:
        raise ApFail(f"TF base_link ← {m.header.frame_id} 없음")
    pts = bf.scan_to_base(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, tf6)
    cfg = bf.FACE_CFG_LOOSE if AP["loose"] else None
    f = bf.find_box_face(pts, expect=expect, cfg=cfg)
    if not f.ok and expect is not None:   # 11-21 (3): 예상 자리 창 (±expect_win) 에서 못 찾으면 정면 넓은 창 (expect 없이) 으로 한 번 더
        f2 = bf.find_box_face(pts, expect=None, cfg=cfg)
        if f2.ok:
            f2.why = (f2.why + " · 넓은 창").strip(" ·")
            f2.wide = True
            return f2
    return f


def ap_measure(node, n, expect_fn, timeout=AP_SCAN_T):
    """멈춘 채 새 앞 스캔 n 장 → 평균 측정 (FaceResult), 받은 장수. 부르기 전 받은 스캔은 안 씀"""
    t0 = time.time()
    last, seen, faces = t0, set(), []
    while time.time() - t0 < timeout and len(faces) < n:
        if STOP.is_set():
            raise ApFail("Ctrl+C")
        s = node.fscan
        now = time.time()
        if s is None or s[0] <= last:
            time.sleep(0.02)
            continue
        last = s[0]
        st = (s[1].header.stamp.sec, s[1].header.stamp.nanosec)
        if now - s[0] > AP_SCAN_FRESH or st in seen:   # 받은 지 0.2 s 넘은 것 · 같은 스캔 다시는 안 씀
            continue
        seen.add(st)
        faces.append(ap_face_of(node, s[1], expect_fn()))
    if len(faces) < AP_MIN_SCANS:
        raise ApFail(f"새 앞 라이다 스캔 {len(faces)} 장만 받음 ({timeout:.0f} s 안, {AP_MIN_SCANS} 장 필요)")
    face = bf.average_faces(faces, **(bf.AVG_LOOSE if AP["loose"] else {}))
    why = ""
    if not face.ok:   # FaceResult 를 고치지 않고 (불변일 수 있음) 이유만 따로
        whys = [f.why for f in faces if getattr(f, "why", "")]
        why = face.why or (max(set(whys), key=whys.count) if whys else "상자 앞면 못 찾음")
    return face, len(faces), why


def ap_check(face, F, o):
    """1 차 측정을 믿을지: 문제면 이유, 괜찮으면 "" """
    if not face.ok:
        return face.why or "상자 앞면 못 찾음"
    gap, lat, yaw = ap_rel(F, o)
    if not AP_GAP_RANGE[0] <= gap <= AP_GAP_RANGE[1]:
        return f"간격 {gap * 100:.1f} cm 가 {AP_GAP_RANGE[0] * 100:.0f} ~ {AP_GAP_RANGE[1] * 100:.0f} cm 밖"
    if abs(lat) > AP_LAT_MAX:
        return f"중앙선 어긋남 {lat * 100:+.1f} cm > {AP_LAT_MAX * 100:.0f} cm (다른 물체?)"
    if abs(yaw) > AP_YAW_MAX:
        return f"방향 {math.degrees(yaw):+.1f}° > {math.degrees(AP_YAW_MAX):.0f}°"
    return ""


def ap_plan_txt(plan):
    return " → ".join(f"회전 {math.degrees(v):+.1f}°" if k == "turn" else f"{'전진' if v >= 0 else '후진'} {abs(v) * 100:.1f} cm" for k, v in plan)


def last_rev(pieces):
    """조각 목록 끝의 이어진 후진 길이 [m] (끝에서부터 후진 합, 전진이 0.15 m 쌓이면 멈춤) — 후진해서 잴 때 이어진 후진 한도 계산"""
    rev = fw = 0.0
    for pc in reversed(pieces):
        if pc["kind"] == "S":
            continue
        L = path_len(pc.get("poses") or [])
        if pc["kind"] == "R":
            rev += L
            fw = 0.0
        else:
            fw += L
            if fw >= 0.15:
                break
    return rev


def box_approach(node, g, back=False, pre=None, rev0=0.0):
    """상자 앞면을 재고 상판 앞 끝 ↔ 상자 AP["gap"] 자리 (중앙선 위 · 법선 방향) 로 다가감. 기록 dict (status done · fallback · skip).
    back = 도착한 자리에서 정지점 AP_MEASURE_D 앞이 되게 곧게 후진한 뒤 잼 (최대 AP_BACK_MAX). pre = 다가가기 전 판정 (cm, deg)
    rev0 = (back) 도착 직전 조각들의 이어진 후진 [m] — 후진 b 를 REV_MAX − rev0 − 0.02 안으로 줄임"""
    ap = {"mode": AP["mode"], "target": leg_target(g), "how": "후진" if back else "끊기", "status": "fallback", "why": "", "pre": pre}
    pub_stop(node, -1.0)   # v2_filter 속도 곡선 끔
    try:
        o = ap_guard(node)
        if back:
            p = node.pose()
            gx, gy, _ = g["goal"]
            ahead = (gx - p[0]) * math.cos(p[2]) + (gy - p[1]) * math.sin(p[2]) if p else 0.0
            b = min(max(AP_MEASURE_D - ahead, 0.0), AP_BACK_MAX)
            budget = REV_MAX - rev0 - 0.02   # 11-21 (2): v2_filter 이어진 후진 한도 (시뮬 0.40) 에 걸려 멈추지 않게
            if b > budget:
                if budget < AP_BACK_MIN:
                    raise ApFail(f"이어진 후진 한도: 앞 후진 {rev0:.2f} + 측정 후진 {b:.2f} m > {REV_MAX:.2f} (줄여도 {max(budget, 0):.2f} < {AP_BACK_MIN:.2f} m)")
                print(f"    · 측정 후진 {b:.2f} → {budget:.2f} m 로 줄임 (앞 조각 끝 후진 {rev0:.2f} m, 이어진 후진 한도 {REV_MAX:.2f})", flush=True)
                b = budget
            if b >= 0.02:
                print(f"    · 이어진 후진 합 {rev0 + b:.2f} m (앞 조각 {rev0:.2f} + 측정 후진 {b:.2f}, 한도 {REV_MAX:.2f})", flush=True)
                ap["rev_chain_m"] = round(rev0 + b, 3)
                if STEP_MODE and not ask_run(f"  ▶ 상자 측정 자리로 곧게 {b:.2f} m 후진 — Enter 실행 · s 건너뛰기 : "):
                    raise ApFail("사람이 건너뜀", "skip")
                print(f"    · 상자 측정 자리로 곧게 후진 {b:.2f} m", flush=True)
                ap_drive(node, -b, o[2], AP_V_MAX)
                ap["back_m"] = round(b, 3)
        node.ap_deadline = time.time() + AP_TOTAL_T   # 측정부터 끝까지 (후진은 ap_drive 단계 상한)
        sleep_s(node, 0.3)
        face, n, why = ap_measure(node, AP_N_SCANS, lambda: ap_expect_goal(g, node.pose()))
        o = ap_guard(node)
        if face.ok:
            F = ap_face_frame(face, o)
            ap["_Fm"] = F   # 11-21 (3) fallback 이어도 잰 상자면 (odom) 을 떼기 (leave_any) 가 씀 — 기록 전에 뺌
            gap, lat, yaw = ap_rel(F, o)
            ap["before"] = ap_rec(gap, lat, yaw, len_cm=round(face.length * 100, 1), n_pts=face.n_pts, n_scans=n,
                                  rms_mm=round(getattr(face, "rms", 0.0) * 1000, 1), loose=AP["loose"])
            print(f"    · 상자 측정 ({ap_q(face, n)}): {ap_txt(gap, lat, yaw)} · 앞면 길이 {face.length * 100:.1f} cm", flush=True)
            why = ap_check(face, F, o)
        if why:
            raise ApFail(f"측정 못 믿음: {why}" + (" · 느슨한 기준" if AP["loose"] else ""))
        close = face.gap is not None and face.gap <= AP_CLOSE_GAP
        if close:   # 11-22: 가까이 잼 → 후진 없는 (옆 ≳ 3 cm 면 짧은 후진) S 자 한 번으로 5 cm 자리까지. 회전 여유는 계획이 모의로 확인 (turn_clearance ≥ 2 cm)
            sp = bf.plan_s_curve_face(face, gap_target=AP["gap"], back_room=AP_MID_BACK)
            ap["s_plan"] = {"ok": sp.ok, "why": sp.why, "alpha_deg": round(math.degrees(sp.alpha), 1), "back_m": round(sp.back, 3),
                            "fwd_m": round(sp.forward, 3), "final_m": round(sp.final, 3), "min_clear_cm": round(sp.min_clear * 100, 1) if sp.steps else None}
            if not sp.ok or not sp.steps:
                raise ApFail(f"S 자 계획 안 됨: {sp.why or '단계 없음'}")
            plan = list(sp.steps)
            print(f"    · S 자 계획: {sp.why} · 후진 {sp.back * 100:.0f} cm · α {math.degrees(sp.alpha):+.1f}° · 비스듬 전진 {sp.forward * 100:.1f} cm · 회전 여유 {sp.min_clear * 100:.1f} cm", flush=True)
        else:
            plan, why = bf.plan_approach(face, AP["gap"], AP_STANDOFF)
        if plan is None:
            raise ApFail(f"기동 상한: {why}")
        ap["plan"] = [[k, round(v, 4)] for k, v in plan]
        if STEP_MODE and not ask_run(f"  ▶ 상자 다가가기 {ap_plan_txt(plan)} → 다시 재고 앞 끝 ↔ 상자 {AP['gap'] * 100:.0f} cm 까지 — Enter 실행 · s 건너뛰기 : "):
            raise ApFail("사람이 건너뜀", "skip")
        print(f"    · 상자 다가가기 {'S 자' if close else '1 차'}: {ap_plan_txt(plan)} (앞 끝 ↔ 상자 {(AP['gap'] + (0.0 if close else AP_STANDOFF)) * 100:.0f} cm 자리)", flush=True)
        tgt = o[2]
        for i, (k, v) in enumerate(plan):
            if k == "turn":
                tgt = wrap(tgt + v)
                ap_turn(node, tgt, F, min_gap=0.0 if close else None)
            elif k == "drive":
                last = close and i == len(plan) - 1 and v > 0
                ap_drive(node, v, tgt, AP_V_FINAL if last else AP_V_MAX, scan=not last)   # 마지막 전진은 상자가 라이다 최소 거리 안
        if close:   # 11-22: 다시 재지 않음 (0.05 m 자리는 라이다에 안 보임) — 오도메트리 예측으로 아래 마무리
            gap, lat, yaw = ap_rel(F, ap_guard(node, scan=False))
            ap["mid"] = ap_rec(gap, lat, yaw, src="odom")
            ap["lat_before_cm"], ap["lat_mid_cm"], ap["mid_iter"] = ap["before"]["lat_cm"], round(lat * 100, 1), 0
        # 11-21 (3) 0.20 m 자리: 다시 재고 옆 오차가 AP_MID_TOL 넘으면 S 자 보정 (bf.plan_s_curve) → 곧게 물러나 다시 잼 (최대 AP_MID_MAX 번)
        node.ap_deadline += AP_MID_T * AP_MID_MAX
        fixes, mids, lat0 = [], [], None
        for k in range(0 if close else AP_MID_MAX + 1):
            sleep_s(node, 0.3)
            face2, n2, _ = ap_measure(node, AP_N2_SCANS, lambda: ap_expect_odom(F, node.odom))
            o = ap_guard(node)
            src = "odom"
            if face2.ok and face2.gap is not None and face2.gap >= AP_SEE_GAP:
                F2 = ap_face_frame(face2, o)
                if math.hypot(F2[0] - F[0], F2[1] - F[1]) <= 0.05:   # 다른 물체를 잡은 것 같으면 안 씀
                    F, src = F2, "lidar"
            gap, lat, yaw = ap_rel(F, o)
            lat0 = lat if lat0 is None else lat0
            mids.append(ap_rec(gap, lat, yaw, src=src, **({"n_pts": face2.n_pts, "rms_mm": round(getattr(face2, "rms", 0.0) * 1000, 1)} if src == "lidar" else {})))
            print(f"    · 다시 잼{f' ({k} 번 보정 뒤)' if k else ''} ({'라이다 · ' + ap_q(face2, n2) if src == 'lidar' else '오도메트리 예측'}): {ap_txt(gap, lat, yaw)}", flush=True)
            if AP_MID_MAX == 0 or abs(lat) <= AP_MID_TOL or k == AP_MID_MAX:
                break
            if src != "lidar" and k > 0:   # 보정 뒤 라이다로 못 재면 오도메트리 예측 (≈ 0) 뿐 — 더 고칠 근거 없음
                break
            sp = bf.plan_s_curve(lat, gap, yaw, gap_target=AP["gap"], back_room=AP_MID_BACK)
            fx = {"k": k + 1, "lat_cm": round(lat * 100, 1), "ok": sp.ok, "why": sp.why}
            fixes.append(fx)
            if not sp.ok:
                print(f"    ⚠ S 자 보정 계획 안 됨 ({sp.why}) — 그대로", flush=True)
                break
            steps = list(sp.steps or [])
            if steps and steps[-1][0] == "drive" and steps[-1][1] > 0 and abs(steps[-1][1] - sp.final) < 1e-6:
                steps = steps[:-1]   # 마지막 곧은 전진은 아래 기존 단계가 (다시 잰 뒤) 함
            fx.update(alpha_deg=round(math.degrees(sp.alpha), 1), back_m=round(sp.back, 3), fwd_m=round(sp.forward, 3), plan=[[a, round(v, 4)] for a, v in steps])
            if STEP_MODE and not ask_run(f"  ▶ S 자 보정 {k + 1}/{AP_MID_MAX}: 옆 {lat * 100:+.1f} cm → {ap_plan_txt(steps)} — Enter 실행 · s 건너뛰기 : "):
                fx["why"] = "사람이 건너뜀"
                break
            print(f"    · S 자 보정 {k + 1}/{AP_MID_MAX}: 옆 {lat * 100:+.1f} cm → {ap_plan_txt(steps)} ({sp.why})", flush=True)
            tgt = o[2]
            for kk, v in steps:
                if kk == "turn":
                    tgt = wrap(tgt + v)
                    ap_turn(node, tgt, F)
                else:
                    ap_drive(node, v, tgt, AP_V_MAX)
            back = (AP["gap"] + AP_STANDOFF) - ap_rel(F, ap_guard(node))[0]   # 0.20 m 자리로 곧게 물러나 다시 잼 (라이다가 보이는 거리)
            if back > 0.005:
                print(f"    · 다시 재려고 곧게 후진 {back * 100:.1f} cm", flush=True)
                ap_drive(node, -back, tgt, AP_V_MAX)
                fx["reback_m"] = round(back, 3)
        if not close:
            ap["mid"] = mids[-1]
            if len(mids) > 1 or fixes:
                ap["mids"], ap["mid_fix"] = mids, fixes
            ap["lat_before_cm"], ap["lat_mid_cm"], ap["mid_iter"] = round(lat0 * 100, 1), round(lat * 100, 1), sum(1 for f in fixes if f.get("plan"))
        if fixes:
            print(f"    · S 자 보정 {ap['mid_iter']} 번: 옆 {lat0 * 100:+.1f} → {lat * 100:+.1f} cm ({'✓' if abs(lat) <= AP_MID_TOL else '⚠ 허용 ' + f'{AP_MID_TOL * 1000:.0f} mm 넘음'})", flush=True)
        if abs(lat) > AP_LAT_LEFT:
            raise ApFail(f"{'S 자 보정' if fixes else '1 차 기동'} 뒤 중앙선 어긋남 {lat * 100:+.1f} cm > {AP_LAT_LEFT * 100:.0f} cm")
        ap_turn(node, F[2], F, min_gap=0.0 if close else None)
        gap, lat, yaw = ap_rel(F, ap_guard(node, scan=not close))
        d = gap - AP["gap"]
        if not AP_DRIVE_MIN <= d <= AP_DRIVE_MAX:
            raise ApFail(f"마지막 전진 {d * 100:.1f} cm 가 {AP_DRIVE_MIN * 100:.0f} ~ {AP_DRIVE_MAX * 100:.0f} cm 밖")
        print(f"    · 법선 방향으로 곧게 {'전진' if d >= 0 else '후진'} {abs(d) * 100:.1f} cm (오도메트리)", flush=True)
        ap_drive(node, d, F[2], AP_V_FINAL, scan=False)   # 상자가 라이다 최소 거리 안으로 들어감 → 라이다 대신 누적 이동 상한
        sleep_s(node, 0.3)
        gap, lat, yaw = ap_rel(F, ap_guard(node, scan=False))
        p = node.pose()
        gx, gy, gth = g["goal"]
        ap["after"] = ap_rec(gap, lat, yaw, src="odom",
                             map_err_cm=round(math.hypot(gx - p[0], gy - p[1]) * 100, 1) if p else None)
        ap["status"] = "done"
        ap["_F"] = F   # 11-21 (2) ap_leave 가 씀 (odom 상자 앞면, 기록 전에 뺌)
        print(f"    · 상자 다가가기 끝: {ap_txt(gap, lat, yaw)} (오도메트리 예측)", flush=True)
    except ApFail as e:
        ap["status"], ap["why"] = e.status, str(e)
        print(f"    ⚠ 상자 다가가기 {'건너뜀' if e.status == 'skip' else '안 함'} ({e}) — 기존 동작으로", flush=True)
    finally:
        node.ap_deadline = None
        ap_zero(node)
    return ap


def ap_return(node, g, tries, tl):
    """후진해서 잰 뒤 그만뒀을 때: 정지점으로 곧게 되돌아가고 방향 맞추기 (기존 도착 자세). 못 하면 tries 에 fail · False"""
    gx, gy, gth = g["goal"]
    p = node.pose()
    if p is None:
        tries.append({"status": "fail", "msg": "상자 다가가기 뒤 정지점 복귀 못 함: map TF 없음", "t": round(node.now_s() - tl, 1)})
        return False
    d = math.hypot(gx - p[0], gy - p[1])
    r, msg = True, ""
    if d > 0.03:
        th = math.atan2(gy - p[1], gx - p[0])
        fwd = math.cos(th - p[2]) >= 0
        n = max(int(round(d / 0.01)), 1)
        hd = th if fwd else wrap(th + math.pi)
        poses = [(p[0] + (gx - p[0]) * i / n, p[1] + (gy - p[1]) * i / n, hd) for i in range(n + 1)]
        print(f"    · 정지점으로 되돌아감 {'전진' if fwd else '후진'} {d:.2f} m", flush=True)
        r, msg = follow_piece(node, {"kind": "F" if fwd else "R", "poses": poses})
    if r and gth is not None:
        p = node.pose()
        err = wrap(gth - p[2]) if p else 0.0
        if abs(err) > YAW_FIX:
            r, msg = follow_piece(node, {"kind": "S", "dth": err})
    if not r:
        print(f"  ✗ 상자 다가가기 뒤 정지점으로 못 돌아옴 ({msg})", flush=True)
        tries.append({"status": "fail", "msg": f"상자 다가가기 뒤 정지점 복귀 못 함: {msg}", "t": round(node.now_s() - tl, 1)})
    return r


def ap_turn_margin(F, o, angles=(math.radians(30), math.radians(90))):
    """그 자리에서 ±angles 돌 때 앞 모서리 ↔ 상자 앞면 평면 최소 여유 [m] (bf.turn_clearance, 음수 = 겹침) · bf.turn_clear 결과"""
    face = ap_face_now(F, o)
    m = min(bf.turn_clearance(face, s * a) for a in angles for s in (1, -1))
    ok = all(bf.turn_clear(face, s * a) for a in angles for s in (1, -1))
    return m, ok


def ap_leave(node, g, ap):
    """11-21 (2) 다가가기 끝 (done) 뒤 떼기 (--ap-return-stop): 지금 방향 그대로 오도메트리로 곧게 후진 — 목표 = 틈 ≥ AP_LEAVE_GAP 과
    계획 정지점 (뒤쪽일 때) 중 더 뒤쪽 (최대 AP_LEAVE_MAX). 정지점이 앞 (상자 쪽) 이어도 앞으로는 안 감. 회전 · 새 길 계산 없음 —
    node.leave 에 이어진 후진을 남겨 다음 구간 leave_trim 이 첫 후진에서 이미 한 만큼 잘라 냄. 기록 dict"""
    F = ap.pop("_F", None)
    rec = {"b_m": 0.0}
    try:
        o = ap_guard(node, scan=False)
        p = node.pose()
        if F is None or p is None:
            raise ApFail("상자 앞면 · map TF 없음")
        gap, _, yaw = ap_rel(F, o)
        m0, _ = ap_turn_margin(F, o)
        b1 = max(AP_LEAVE_GAP - gap, 0.0) / max(math.cos(yaw), 0.5)
        gx, gy, _ = g["goal"]
        along = (gx - p[0]) * math.cos(p[2]) + (gy - p[1]) * math.sin(p[2])   # + = 정지점이 앞 (상자 쪽)
        b2 = max(-along, 0.0)
        b = max(b1, b2)
        if b > AP_LEAVE_MAX:
            print(f"    ⚠ 떼기 후진 {b:.2f} m > {AP_LEAVE_MAX:.2f} — {AP_LEAVE_MAX:.2f} m 까지만 (정지점 {-along:.2f} m 뒤, 나머지는 다음 구간)", flush=True)
            b = AP_LEAVE_MAX
        # 이어진 후진: 후진해서 잰 경우 다가가기 전진 (틈 줄어든 만큼) 이 0.15 m 안 되면 측정 후진이 이어짐 (v2_filter RESET)
        fwd = ((ap.get("before") or {}).get("gap_cm", 0) - (ap.get("after") or {}).get("gap_cm", 0)) / 100.0
        rev_prev = ap["_rev_prev"] if "_rev_prev" in ap else (ap.get("rev_chain_m", 0.0) if (ap.get("how") == "후진" and fwd < 0.15) else 0.0)
        why = ("틈 " + f"{AP_LEAVE_GAP * 100:.0f} cm" if b1 >= b2 else "정지점") + " 쪽이 더 뒤"
        print(f"    · 다가가기 뒤 떼기: 지금 틈 {gap * 100:.1f} cm · 정지점 {('앞 ' + f'{along * 100:.1f}') if along >= 0 else ('뒤 ' + f'{-along * 100:.1f}')} cm"
              f" → 곧게 후진 {b * 100:.1f} cm ({why}) · 그 자리에서 돌 때 모서리 여유 (bf.turn_clearance ±30·90°) {m0 * 100:+.1f} cm", flush=True)
        rec.update(gap0_cm=round(gap * 100, 1), stop_along_cm=round(along * 100, 1), margin0_cm=round(m0 * 100, 1), rev_prev_m=round(rev_prev, 3))
        if b >= 0.005:
            if STEP_MODE and not ask_run(f"  ▶ 상자에서 떼기: 곧게 {b * 100:.1f} cm 후진 — Enter 실행 · s 건너뛰기 : "):
                raise ApFail("사람이 건너뜀", "skip")
            ap_drive(node, -b, o[2], AP_V_MAX, scan=False)
            rec["b_m"] = round(b, 3)
        o = ap_guard(node, scan=False)
        gap, _, _ = ap_rel(F, o)
        m1, ok1 = ap_turn_margin(F, o)
        rec.update(gap1_cm=round(gap * 100, 1), margin1_cm=round(m1 * 100, 1), turn_clear=ok1)
        print(f"    · 떼기 끝: 틈 {gap * 100:.1f} cm · 모서리 여유 {m1 * 100:+.1f} cm (bf.turn_clear {'됨' if ok1 else '⚠ 안 됨'}) · "
              f"이어진 후진 {rev_prev + rec['b_m']:.2f} m (다음 구간 첫 후진에서 이미 한 만큼 잘라 냄)", flush=True)
    except ApFail as e:
        rec["why"] = str(e)
        print(f"    ⚠ 다가가기 뒤 떼기 못 함 ({e}) — 다음 구간 첫 후진이 이어받음", flush=True)
    finally:
        ap_zero(node)
    node.leave = {"rev": rec.get("rev_prev_m", 0.0) + rec["b_m"], "leg": g.get("leg")}
    return rec


# ───────────── 11-21 (3) 상자 앞 제자리 회전 (Spin "Collision Ahead" 대책) ─────────────
def box_face_odom(node, name, p=None, o=None):
    """상자 name (box_quiet.BOXES, map · 변이 지도 축과 나란함) 의 로봇 쪽 면 → F (x, y, nyaw) odom 프레임 (ap_rel · ap_leave 와 같은 꼴).
    nyaw = 로봇 → 상자 법선. map → odom 은 지금 map 자세 p 와 오도메트리 o 로 (멈춘 채 쓰는 값). 모르면 None"""
    c = box_quiet.BOXES.get(name)
    p = p or node.pose()
    o = o or node.odom
    if c is None or p is None or o is None:
        return None
    dx, dy = p[0] - c[0], p[1] - c[1]
    ux, uy = (math.copysign(1.0, dx), 0.0) if abs(dx) >= abs(dy) else (0.0, math.copysign(1.0, dy))
    fx, fy = c[0] + ux * BOX_HALF, c[1] + uy * BOX_HALF
    d = wrap(o[2] - p[2])
    cd, sd = math.cos(d), math.sin(d)
    rx, ry = fx - p[0], fy - p[1]
    return o[0] + cd * rx - sd * ry, o[1] + sd * rx + cd * ry, wrap(math.atan2(-uy, -ux) + d)


def near_box(node):
    """지금 앞에 있는 상자 (이름, F odom, 틈 [m]): 상판 앞 끝 ↔ 앞면 ≤ NEAR_BOX_GAP · 방향 차 ≤ 60° · 옆 ≤ 반 변 + 0.20. 없으면 None"""
    p, o = node.pose(), node.odom
    if bf is None or p is None or o is None or time.time() - o[3] > ODOM_STALE:
        return None
    best = None
    for name in box_quiet.BOXES:
        F = box_face_odom(node, name, p, o)
        if F is None:
            continue
        gap, lat, yaw = ap_rel(F, o)
        if -0.15 < gap <= NEAR_BOX_GAP and abs(yaw) <= math.radians(60) and abs(lat) <= BOX_HALF + 0.20 and (best is None or gap < best[2]):
            best = (name, F, gap)
    return best


def room_need(F, o, ang, bmax=AP_LEAVE_MAX):
    """틈 확보 후진 [m] (ap_leave 와 같은 기준): 틈 ≥ AP_LEAVE_GAP 이고 그 자리에서 ang 돌 때 bf.turn_clear 가 되는 가장 짧은 곧은 후진
    (1 cm 간격, ≤ bmax). (b, 됨)"""
    face = ap_face_now(F, o)
    gap, _, yaw = ap_rel(F, o)
    cy = max(math.cos(yaw), 0.5)
    for k in range(int(round(bmax / 0.01)) + 1):
        b = k * 0.01
        if gap + b * cy >= AP_LEAVE_GAP and bf.turn_clear(face, ang, pose=(-b, 0.0, 0.0)):
            return b, True
    return bmax, False


def turn_direct(node, target):
    """오도메트리 yaw 를 target (odom) 으로 cmd_vel_raw 직접 제자리 회전 (ap_turn, 라이다 안 봄 · Nav2 가 쉬는 동안). (됨, 메모)"""
    pub_stop(node, -1.0)
    try:
        err = ap_turn(node, target, None, scan=False)
        return True, f"끝 오차 {math.degrees(err):+.1f}°"
    except ApFail as e:
        return False, str(e)
    finally:
        ap_zero(node)


def turn_safe(node, dth, room=False, rev_budget=None):
    """11-21 (3) 제자리 회전 dth [rad] — Spin "Collision Ahead" 대책. (됨, 메시지) 는 follow_piece 와 같은 꼴.
    1) |dth| ≤ TURN_DIRECT (상자 앞이고 bf.turn_clear 면 TURN_DIRECT_NEAR) → cmd_vel_raw 직접 (local costmap 검사 없음, collision_monitor 는 그대로)
    2) room (정지점 아닌 계획 제자리) 이고 상자 앞이면 틈 확보 후진 (room_need, ≤ rev_budget) 먼저
    3) Nav2 Spin. Collision Ahead 로 거부되면 상자 기하로 여유 확인 (bf.turn_clear) 되고 ≤ TURN_DIRECT_FB 일 때 직접으로 한 번 더"""
    ang = wrap(dth)
    rec = {"deg": round(math.degrees(ang), 1), "how": "spin"}
    if not hasattr(node, "turns"):
        node.turns = []
    node.turns.append(rec)
    o = node.odom
    odom_ok = o is not None and time.time() - o[3] <= ODOM_STALE
    nb = near_box(node) if odom_ok else None
    if nb:
        rec.update(box=nb[0], gap_cm=round(nb[2] * 100, 1))
    target = wrap(o[2] + ang) if odom_ok else None
    lim = TURN_DIRECT
    if nb and TURN_DIRECT > 0 and abs(ang) > lim and bf.turn_clear(ap_face_now(nb[1], o), ang):
        lim = max(lim, TURN_DIRECT_NEAR)
    if odom_ok and TURN_DIRECT > 0 and abs(ang) <= lim:
        r, m = turn_direct(node, target)
        print(f"    · 제자리 {math.degrees(ang):+.1f}° 직접 회전 (Spin 대신" + (f", {nb[0]} 앞 틈 {nb[2] * 100:.0f} cm" if nb else "") + f") — {m if r else '⚠ ' + m + ' → Spin 으로'}", flush=True)
        rec["how"] = "direct"
        if r:
            rec["ok"] = True
            return True, ""
    if room and TURN_ROOM and nb and odom_ok and not STOP.is_set():
        o = node.odom
        rem = wrap(target - o[2])
        b, enough = room_need(nb[1], o, rem)
        cap = AP_LEAVE_MAX if rev_budget is None else max(min(AP_LEAVE_MAX, rev_budget), 0.0)
        if b > cap + 1e-6:
            print(f"    ⚠ 회전 전 틈 확보 후진 {b * 100:.0f} cm 필요 > {cap * 100:.0f} cm (이어진 후진 한도 · 떼기 최대) — {cap * 100:.0f} cm 만", flush=True)
            b, enough = cap, False
        if b >= 0.005:
            print(f"    · 제자리 {math.degrees(rem):+.0f}° 전 틈 확보: {nb[0]} 앞 틈 {nb[2] * 100:.1f} cm → 곧게 후진 {b * 100:.0f} cm"
                  + ("" if enough else " (⚠ 여유 다 못 냄)"), flush=True)
            if STEP_MODE and not ask_run(f"  ▶ 회전 전 곧게 {b * 100:.0f} cm 후진 — Enter 실행 · s 건너뛰기 : "):
                b = 0.0
            else:
                pub_stop(node, -1.0)
                try:
                    ap_drive(node, -b, o[2], AP_V_MAX, scan=False)
                    rec["back_m"] = round(b, 3)
                except ApFail as e:
                    print(f"    ⚠ 틈 확보 후진 못 함 ({e}) — 그대로 Spin", flush=True)
                finally:
                    ap_zero(node)
    rem = wrap(target - node.odom[2]) if odom_ok else ang
    r, msg = follow_piece(node, {"kind": "S", "dth": rem, "_raw": True})
    if not r and "Collision" in msg and odom_ok and TURN_DIRECT > 0 and not STOP.is_set():
        nb2 = near_box(node)
        rem = wrap(target - node.odom[2])
        if nb2 and abs(rem) <= TURN_DIRECT_FB and bf.turn_clear(ap_face_now(nb2[1], node.odom), rem):
            r2, m2 = turn_direct(node, target)
            print(f"    · Spin 거부 (Collision Ahead) — {nb2[0]} 기하로 앞 모서리 여유 됨 → {math.degrees(rem):+.1f}° 직접 회전: {m2 if r2 else '⚠ ' + m2}", flush=True)
            rec["how"] = "spin→direct"
            if r2:
                rec["ok"] = True
                return True, ""
    rec["ok"] = r
    if not r:
        rec["msg"] = msg
    return r, msg


def mark_spins(pieces, rev0=0.0):
    """11-21 (3) 구간 마지막이 아닌 제자리 조각에 _room (상자 앞이면 틈 확보 후진) · _rev_budget (이어진 후진 한도 남은 것) 표시.
    budget = REV_MAX − 0.02 − 앞쪽 이어진 후진 (앞에 전진이 없으면 떼기 rev0 포함) − 뒤 첫 후진"""
    out = list(pieces)
    for i, pc in enumerate(out):
        if pc["kind"] != "S" or i == len(out) - 1:
            continue
        before = last_rev(out[:i]) + (rev0 if not any(q["kind"] == "F" for q in out[:i]) else 0.0)
        out[i] = dict(pc, _room=True, _rev_budget=REV_MAX - 0.02 - before - first_rev(out[i + 1:]))
    return out


def leave_any(node, g, ap, pieces):
    """11-21 (3) 다가가기가 안 된 (fallback · 끔 · dry) 상자 정지점에서도 떼기 — ap_leave 와 같은 기준 (틈 ≥ AP_LEAVE_GAP, ≤ AP_LEAVE_MAX).
    상자면 = 잰 값 (ap _Fm, odom) 이 있으면 그것, 없으면 발제 상자 좌표 (box_quiet.BOXES). 상자가 앞에 없거나 틈이 이미 넉넉하면 안 함. 기록 dict | None"""
    F, src = ((ap or {}).get("_Fm"), "잰 상자면")
    if F is None:
        F, src = box_face_odom(node, leg_target(g)), "발제 상자 좌표"
    o = node.odom
    if F is None or o is None:
        return None
    gap, lat, yaw = ap_rel(F, o)
    if gap >= AP_LEAVE_GAP or abs(yaw) > math.radians(60) or abs(lat) > BOX_HALF + 0.20:
        return None
    print(f"    · 상자 앞 떼기 (다가가기 {'없음' if ap is None else ap['status']}, {src}): 틈 {gap * 100:.1f} cm < {AP_LEAVE_GAP * 100:.0f} cm", flush=True)
    rec = ap_leave(node, g, {"_F": F, "_rev_prev": last_rev(pieces) if pieces and pieces[-1]["kind"] == "R" else 0.0})
    rec["src"] = src
    return rec


def leave_trim(node, pieces):
    """11-21 (2) 앞 구간이 떼기 (ap_leave) 로 끝났으면: 첫 조각이 후진일 때 지금 자세를 그 경로에 투영해 가장 가까운 점부터 (이미 후진한 만큼 잘라 냄).
    이어진 후진 (앞 후진 + 남은 첫 후진) 이 REV_MAX − 0.02 를 넘으면 첫 후진 끝을 잘라 맞춤 · 그래도 안 되면 경고. 바뀐 조각 목록"""
    lv, node.leave = getattr(node, "leave", None), None
    if lv and pieces and pieces[0]["kind"] == "S":   # 처음이 제자리 (예 S2 +30°) 면 잘라 내지 않고 이어진 후진만 확인
        r = next((pc for pc in pieces if pc["kind"] != "S"), None)
        if r is not None and r["kind"] == "R" and lv["rev"] + path_len(r["poses"]) > REV_MAX + 1e-6:
            print(f"    ⚠ 이어진 후진: 떼기 {lv['rev']:.2f} + 첫 후진 {path_len(r['poses']):.2f} m > {REV_MAX:.2f} (처음이 제자리라 잘라 내지 않음 — v2_filter 가 후진을 멈출 수 있음)", flush=True)
        return pieces
    if not lv or not pieces or pieces[0]["kind"] != "R" or len(pieces[0].get("poses") or []) < 2:
        return pieces
    p = node.pose()
    if p is None:
        return pieces
    poses = pieces[0]["poses"]
    k = min(range(len(poses)), key=lambda j: math.hypot(poses[j][0] - p[0], poses[j][1] - p[1]))
    d = math.hypot(poses[k][0] - p[0], poses[k][1] - p[1])
    if d > AUTO_LAT + 0.03:   # 경로 위가 아님 (떼기 뒤 다른 곳) — 그대로
        print(f"    · 첫 후진 잘라 내기 안 함 (경로에서 {d * 100:.0f} cm)", flush=True)
        return pieces
    full, done = path_len(poses), path_len(poses[:k + 1])
    rest = list(poses[k:])
    budget = REV_MAX - 0.02 - lv["rev"]
    cut_end = 0.0
    if path_len(rest) > budget:
        cum, keep = 0.0, [rest[0]]
        for a, b in zip(rest, rest[1:]):
            cum += math.hypot(b[0] - a[0], b[1] - a[1])
            if cum > budget:
                break
            keep.append(b)
        cut_end = path_len(rest) - path_len(keep)
        rest = keep
    left = path_len(rest)
    msg = (f"    · 첫 후진 {full:.2f} m 중 {done:.2f} m 이미 함 (떼기) → {left:.2f} m 부터 이어감 (투영점 어긋남 {d * 100:.1f} cm) · "
           f"이어진 후진 {lv['rev'] + left:.2f} m (한도 {REV_MAX:.2f})")
    if cut_end > 0:
        msg += f" · ⚠ 한도 맞추려 끝 {cut_end:.2f} m 잘라 냄 (다음 조각 시작이 그만큼 어긋남)"
    print(msg, flush=True)
    if budget < 0.0:
        print(f"    ⚠ 이어진 후진 {lv['rev']:.2f} m 이미 한도 근처 — 첫 후진 없이 감 (v2_filter 가 후진을 0 으로 낼 수 있음)", flush=True)
    out = list(pieces)
    if len(rest) < 2 or left < 0.03:
        out.pop(0)
    else:
        out[0] = dict(pieces[0], poses=rest)
        out[0].pop("_near", None)
    return out


def ap_dry(node, g):
    """dry: 정지점 AP_MEASURE_D 앞을 지날 때 잡은 스캔으로 잰 값 · 기동 계획만 기록 (움직이지 않음)"""
    cap, node.ap_dry_goal = list(node.ap_cap), None
    ap = {"mode": "dry", "target": leg_target(g), "status": "dry", "why": ""}
    try:
        if not cap:
            raise ApFail(f"정지점 {AP_MEASURE_D:.2f} m 앞에서 앞 라이다를 못 받음")
        faces = [ap_face_of(node, m, ap_expect_goal(g, p)) for m, p in cap]
        face = next((f for f in faces if f.ok), None)
        if face is None:
            raise ApFail((faces[0].why or "상자 앞면 못 찾음") + f" (점 {faces[0].n_pts}" + (" · 느슨한 기준)" if AP["loose"] else ")"))
        gap, lat, yaw = ap_rel(ap_face_frame(face, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        ap["before"] = ap_rec(gap, lat, yaw, len_cm=round(face.length * 100, 1), n_pts=face.n_pts, n_scans=len(cap),
                              rms_mm=round(getattr(face, "rms", 0.0) * 1000, 1), loose=AP["loose"])
        plan, why = bf.plan_approach(face, AP["gap"], AP_STANDOFF)
        ap["plan"] = [[k, round(v, 4)] for k, v in plan] if plan else None
        ap["why"] = "" if plan else f"기동 상한: {why}"
        print(f"    · 상자 측정 (dry, 주행 중 정지점 약 {AP_MEASURE_D:.2f} m 앞, {ap_q(face)}): {ap_txt(gap, lat, yaw)} · 앞면 길이 {face.length * 100:.1f} cm · "
              + (f"계획 {ap_plan_txt(plan)}" if plan else f"⚠ {ap['why']}"), flush=True)
    except ApFail as e:
        ap["why"] = str(e)
        print(f"    ⚠ 상자 측정 (dry) 못 함 ({e})", flush=True)
    return ap


def ap_tail(ap):
    """구간 요약 줄 끝 꼬리"""
    if not ap:
        return ""
    if ap["status"] == "done":
        a = ap["after"]
        return f" · 상자 다가가기 {a['gap_cm']:.1f} cm · 중앙선 {a['lat_cm']:+.1f} cm · {a['yaw_deg']:+.1f}°"
    if ap["status"] == "dry":
        b = ap.get("before")
        return f" · 상자 측정 (dry) {b['gap_cm']:.1f} cm · 중앙선 {b['lat_cm']:+.1f} cm · {b['yaw_deg']:+.1f}°" if b else f" · 상자 측정 (dry) ⚠ {ap['why']}"
    return f" · 상자 다가가기 ⚠ {'건너뜀' if ap['status'] == 'skip' else '안 함'} ({ap['why']})"


def run_leg(node, g, results):
    """구간 하나 (조각들 → 정지점 방향 맞추기 → 어긋남 판정). False = 조각이 실패해 여기서 끝냄"""
    node.step = g["leg"]
    tl = node.now_s()
    node.head_fix = []
    node.head_skip = []
    node.recover = []
    node.turns = []   # 11-21 (3) 제자리 회전 기록 (turn_safe: 직접 · Spin · 틈 확보 후진)
    tries, ok = [], True
    det_n = 0   # 11-19 이 구간 우회 횟수
    node.tries, node.leg_t0, node.hold_n = tries, tl, 0   # 11-19 follow_piece 가 HOLD 를 여기 기록
    node.quiet_n = 0
    node.set_quiet_goal(box_goal(g), leg_target(g))   # 11-21: 상자 구간이면 정지점 0.40 m 안에서 상자 점 빼기 · 0.30 m 안 속도 상한
    try:
        return _run_leg(node, g, results, tl, tries, ok, det_n)
    finally:
        node.set_quiet_goal(None)   # 구간 끝 = 상자 구간 끝 (다음 구간 시작 전)


def _run_leg(node, g, results, tl, tries, ok, det_n):
    pieces = merge_pieces(g["pieces"])   # 11-18: 같은 방향 이어진 조각은 하나로 (사이에서 안 멈춤)
    lv = getattr(node, "leave", None)
    pieces = leave_trim(node, pieces)    # 11-21 (2): 앞 구간 떼기 뒤 — 첫 후진에서 이미 한 만큼 잘라 냄
    pieces = mark_spins(pieces, lv["rev"] if lv else 0.0)   # 11-21 (3): 정지점 아닌 제자리는 상자 앞이면 틈 확보 후진 먼저
    node.ap_res, node.ap_cap, node.ap_dry_goal = None, [], None   # 11-21 상자 다가가기
    tgt = ap_target(g)
    if tgt:
        now, s, o = time.time(), node.fscan, node.odom   # 측정 자리에 가기 전 확인 — 안 되면 조각을 끊지 않음 (시뮬 · 라이다 없음에서 안전)
        why = (f"앞 라이다 {AP_SCAN_ALIVE:.0f} s 안에 못 받음" if s is None or now - s[0] > AP_SCAN_ALIVE else
               f"TF base_link ← {s[1].header.frame_id} 없음" if ap_tf6(node, s[1].header.frame_id) is None else
               "오도메트리 못 받음" if AP["mode"] == "on" and (o is None or now - o[3] > ODOM_STALE) else "")
        if why:
            print(f"    ⚠ 상자 다가가기 안 함 ({why}) — 기존 동작으로", flush=True)
            node.ap_res = {"mode": AP["mode"], "target": tgt, "status": "fallback", "why": why, "pre": None}
        elif AP["mode"] == "dry":
            node.ap_dry_goal = tuple(g["goal"])
        else:
            sp, cut = ap_split(pieces)
            dcut = wrap(sp[-1]["poses"][-1][2] - sp[-2]["poses"][-1][2]) if sp is not None else 0.0
            if sp is not None and AP_CUT_YAW > 0 and abs(dcut) > AP_CUT_YAW:
                # 11-21 (2): 끊는 점에서 정지점과 방향이 크게 다르면 1 차 측정이 AP_YAW_MAX 에 걸려 늘 fallback (wide S5 · S6) → 도착 뒤 후진해서 잼
                print(f"    · 상자 다가가기: 끊는 점 방향이 정지점과 {math.degrees(dcut):+.0f}° 다름 (> {math.degrees(AP_CUT_YAW):.0f}°) — 끊지 않고 도착 뒤 후진해서 잼", flush=True)
            elif sp is not None:
                pieces = sp
                print(f"    · 상자 다가가기: 마지막 전진을 정지점 {cut:.2f} m 앞에서 끊고 상자를 잼", flush=True)
    for i, pc in enumerate(pieces):
        if pc.get("_box_tail"):   # 11-21: 끊은 점에 멈춘 뒤 — 재고 다가감. 되면 나머지 조각은 안 감, 안 되면 나머지를 그대로
            sleep_s(node, 0.5)
            p, e = node.pose(), pieces[i - 1]["poses"][-1]
            pre = {"ref": "끊은 점", "err_cm": round(math.hypot(e[0] - p[0], e[1] - p[1]) * 100, 1),
                   "err_deg": round(math.degrees(abs(wrap(e[2] - p[2]))), 1)} if p else None
            node.ap_res = box_approach(node, g, pre=pre)
            if node.ap_res["status"] == "done" or STOP.is_set():
                break
            if node.ap_res["status"] == "fallback":   # 11-21 (2): 도착한 뒤 후진해서 한 번 더 잼 (사람이 건너뛴 skip 은 안 함)
                node.ap_res["_retry"] = True
            print("    · 끊었던 나머지 전진을 그대로 감" + (" (도착 뒤 후진해서 한 번 더 잼)" if node.ap_res.get("_retry") else ""), flush=True)
        what = {"F": "전진", "R": "후진", "S": "제자리"}[pc["kind"]] + (f" {math.degrees(pc['dth']):+.0f}°" if pc["kind"] == "S" else "")
        print(f"  · 조각 {i + 1}/{len(pieces)} {what}" + (" (상자 측정 자리까지)" if pc.get("_stop") else " (상자 앞 나머지)" if pc.get("_box_tail") else ""), flush=True)
        fin = (i == len(pieces) - 1) or bool(pc.get("_stop"))   # 11-21: 끊은 앞부분 끝도 정지 (속도 곡선으로 천천히)
        cur = pc   # 11-19: 우회 뒤에는 남은 조각 (합류점부터)
        r, msg = follow_piece(node, cur, final=fin)
        k = 0
        while not r and not STOP.is_set():
            # 11-19 우회: 전진 조각이 장애물로 BLOCK_WAIT 넘게 막히면 planner 우회 경로 → 합류점부터 남은 조각
            if msg.startswith(BLOCKED) and cur["kind"] == "F" and DETOUR_ON and det_n < DETOUR_MAX:
                det_n += 1
                print(f"  ⤳ 우회 {det_n}/{DETOUR_MAX} ({msg[len(BLOCKED) + 2:]})", flush=True)
                back = detour_back(node, cur)   # 먼저 짧게 후진 (장애물과 0.20 m 넘게 떨어져야 전진만으로 비킴)
                if back is None:
                    tries.append({"status": "detour", "msg": f"조각 {i + 1} ({what}): 우회 전 후진 못 함 (TF 없음)", "t": round(node.now_s() - tl, 1)})
                    msg = "장애물로 막힘 · 우회 전 후진 못 함 (TF 없음)"
                    continue
                print(f"    · 우회 전 후진 {back['len']:.2f} m", flush=True)
                rb, mb = follow_piece(node, back, final=False)
                if not rb:
                    print(f"  ✗ 우회 전 후진 못 함 ({mb}) — 되찾기로", flush=True)
                    tries.append({"status": "detour", "msg": f"조각 {i + 1} ({what}): 우회 전 후진 못 함 ({mb})", "t": round(node.now_s() - tl, 1)})
                    msg = f"장애물로 막힘 · 우회 전 후진 못 함 ({mb[len(BLOCKED) + 2:] if mb.startswith(BLOCKED) else mb})"
                    continue
                dpc, j, info = plan_detour(node, cur)
                if dpc is None:
                    if info != "planner_server 없음":
                        print("  ✗ 우회 경로 없음 (옆 공간 부족)", flush=True)
                    tries.append({"status": "detour", "msg": f"조각 {i + 1} ({what}): 우회 경로 없음 ({info})", "t": round(node.now_s() - tl, 1)})
                    msg = f"장애물로 막힘 · 우회 경로 없음 ({info})"   # BLOCKED 표시를 떼서 아래 되찾기로
                    continue
                print(f"    · 우회 경로 {dpc['len']:.2f} m 따라가기 ({info})", flush=True)
                rd, md = follow_piece(node, dpc, final=False)
                tries.append({"status": "detour", "msg": f"조각 {i + 1} ({what}): 우회 {dpc['len']:.2f} m ({info})" + ("" if rd else f" — 중단: {md}"),
                              "t": round(node.now_s() - tl, 1)})
                if not rd:
                    r, msg = False, md   # 다시 막힘 (BLOCKED) 이면 한 번 더 우회, 아니면 원래 조각으로 되찾기
                    continue
                rest = cut_piece(cur, j)
                if rest is None:
                    print(f"    · 우회로 조각 {i + 1} 끝까지 옴", flush=True)
                    r, msg = True, ""
                    break
                cur = rest
                print(f"    · 조각 {i + 1}/{len(pieces)} {what} 이어감 (남은 {path_len(cur['poses']):.2f} m)", flush=True)
                r, msg = follow_piece(node, cur, final=fin)
                continue
            if k >= RECOVER_TRIES:   # 11-16 되찾기: 자기 위치 찾기 → 왔던 길 되돌아가기 → 다시
                break
            k += 1
            print(f"  ↺ 되찾기 {k}/{RECOVER_TRIES} ({msg})", flush=True)
            tries.append({"status": "recover", "msg": f"조각 {i + 1} ({what}): {msg}", "t": round(node.now_s() - tl, 1)})
            relocalize(node, "멈춤")
            back = backtrack_piece(node, cur) if cur["kind"] in "FR" else None
            if back:
                print(f"    · 왔던 길 {back['len']:.2f} m 되돌아가기 ({'후진' if back['kind'] == 'R' else '전진'})", flush=True)
                rb, mb = follow_piece(node, back)
                if not rb:
                    print(f"    · 되돌아가기 못 함 ({mb}) — 이 자리에서 다시", flush=True)
                relocalize(node, "되돌아간 뒤")
            if STOP.is_set():
                break
            print(f"    · 조각 {i + 1}/{len(pieces)} {what} 다시", flush=True)
            r, msg = follow_piece(node, cur, final=fin)
        if k:
            node.recover.append(k)
        if not r and not STOP.is_set() and not STEP_MODE and not AUTO_MODE and os.environ.get("DEMO_AUTO_SKIP") == "1":   # 11-16 max: 사람 없이 끝까지 — 되찾기도 안 되면 이 조각 건너뛰고 계속
            print(f"    · 조각 {i + 1} 건너뜀 (자동, 되찾기 {k} 번 뒤: {msg})", flush=True)
            tries.append({"status": "skip", "msg": f"조각 {i + 1} ({what}) 자동 건너뜀: {msg}", "t": round(node.now_s() - tl, 1)})
            r, msg = True, ""
        while not r and not STOP.is_set() and STEP_MODE:
            c = ask_choice(f"  ▶ 조각 {i + 1} 되찾기 실패 ({msg}) — Enter 한 번 더 · s 이 조각 건너뛰기 · q 구간 끝 : ")
            if c == "quit":
                break
            if c == "skip":
                print(f"    · 조각 {i + 1} 건너뜀 (사람)", flush=True)
                tries.append({"status": "skip", "msg": f"조각 {i + 1} ({what}) 사람이 건너뜀", "t": round(node.now_s() - tl, 1)})
                r, msg = True, ""
                break
            relocalize(node, "사람 요청")
            r, msg = follow_piece(node, cur, final=fin)
        if not r:
            ok = False
            tries.append({"status": "fail", "msg": f"조각 {i + 1} ({what}): {msg}", "t": round(node.now_s() - tl, 1)})
            print(f"  ✗ {msg}", flush=True)
            break
    gx, gy, gth = g["goal"]
    ap = node.ap_res
    if ap is not None and ap["status"] == "done":
        # 11-21: 끊은 점에서 재고 다가감 — 어긋남 판정은 다가가기 전 (끊은 점 기준), 다가간 뒤 값은 ap["after"]. 끊은 점 자세를 몰랐으면 실패로
        pre = ap.get("pre")
        dpos, dyaw = (pre["err_cm"] / 100.0, math.radians(pre["err_deg"])) if pre else (9.0, 0.0)
    else:
        p = node.pose()
        if ok and gth is not None and p:
            err = wrap(gth - p[2])
            if abs(err) > YAW_FIX:
                print(f"  · 정지점 방향 맞추기 {math.degrees(err):+.0f}°", flush=True)
                r, msg = follow_piece(node, {"kind": "S", "dth": err})
                if not r:
                    ok = False
                    tries.append({"status": "fail", "msg": f"방향 맞추기 {math.degrees(err):+.0f}°: {msg}", "t": round(node.now_s() - tl, 1)})
            sleep_s(node, 0.5)
            p = node.pose()
        dpos = math.hypot(gx - p[0], gy - p[1]) if p else 9.0
        dyaw = abs(wrap(gth - p[2])) if (p and gth is not None) else 0.0
        if ap_target(g) and not STOP.is_set():
            if AP["mode"] == "dry":
                ap = ap_dry(node, g)
            elif (ap is None or ap.get("_retry")) and ok:   # 마지막 조각이 전진이 아니거나 짧음 · 끊지 않음 · 끊고 잰 것이 안 됨 — 도착 자리에서 후진해서 잼 (판정은 위 도착 값)
                first = ap
                if first is not None:
                    first.pop("_retry", None)
                    print("    · 끊고 잰 다가가기 안 됨 — 도착 자리에서 후진해서 한 번 더 잼", flush=True)
                ap = box_approach(node, g, back=True, pre={"ref": "정지점", "err_cm": round(dpos * 100, 1), "err_deg": round(math.degrees(dyaw), 1)},
                                  rev0=last_rev(pieces))
                if first is not None:
                    ap["first"] = first
                if ap["status"] != "done" and not STOP.is_set() and not ap_return(node, g, tries, tl):
                    ok = False
    leave_rec = None
    if ap is not None:
        ap.pop("_retry", None)
        if ap["status"] == "done" and AP_RETURN_STOP and not STOP.is_set():
            ap["leave"] = ap_leave(node, g, ap)
        ap.pop("_F", None)
    if (ap is None or ap["status"] != "done") and ok and AP_LEAVE_ALL and AP_RETURN_STOP and leg_target(g) in BOX_STOPS and not STOP.is_set():
        leave_rec = leave_any(node, g, ap, pieces)   # 11-21 (3): 다가가기 안 된 곳도 상자 앞이면 떼기 (다음 구간 첫 제자리 · 방향 맞추기 Spin 거부 막기)
    if ap is not None:
        ap.pop("_Fm", None)
        (ap.get("first") or {}).pop("_Fm", None)
    node.ap_dry_goal = None
    if ok and (dpos > (POS_OK if gth is not None else 0.15) or dyaw > YAW_OK):
        ok = False
        tries.append({"status": "fail", "msg": f"정지점 어긋남 {dpos * 100:.1f} cm · {math.degrees(dyaw):.1f}°"})
    if ok:
        tries.append({"status": "ok", "msg": "", "t": round(node.now_s() - tl, 1)})
    results.append({"seg": f"S{g['leg']}", "name": g["name"], "points": g["points"], "ok": ok, "tries": tries,
                    "sim_s": round(node.now_s() - tl, 1), "end_err_cm": round(dpos * 100, 1), "end_err_deg": round(math.degrees(dyaw), 1),
                    "head_fix_deg": list(node.head_fix), "head_skip_deg": list(node.head_skip), "recover": list(node.recover), "detour": bool(g.get("detour")),
                    "detour_n": det_n, "hold_n": node.hold_n, "box_quiet_n": node.quiet_n, "approach": ap,
                    "turns": list(node.turns), "leave": leave_rec})
    print(f"{node.prog}{'✓' if ok else '✗'} {g['name']}: {node.now_s() - tl:.1f} s · 정지점 어긋남 {dpos * 100:.1f} cm · {math.degrees(dyaw):.1f}°"
          f" · 위치 추정 {node.loc or '-'}" + (f" · 조각 방향 맞추기 {node.head_fix}" if node.head_fix else "") + (f" · 건너뜀 {node.head_skip}" if node.head_skip else "") + (f" · 되찾기 {sum(node.recover)} 번" if node.recover else "") + (f" · 우회 {det_n} 번" if det_n else "") + (f" · HOLD {node.hold_n} 번" if node.hold_n else "") + (f" · 상자 구간 {node.quiet_n} 번" if node.quiet_n else "") + ap_tail(ap), flush=True)
    if ok:   # 도착 알림 (팔 · 미션 쪽이 들을 수 있게) — 11-21 상자 다가가기가 끝난 뒤
        node.arrived.publish(String(data=g.get("to") or g["name"].split("→")[-1]))
    # 11-16: 되찾기 (recover) · 사람 건너뛰기 (skip) · 11-19 우회 (detour) · HOLD (hold) 기록은 실패가 아님 — 실패 (fail) 만 봄
    return not any(t["status"] == "fail" and ("조각" in t["msg"] or "방향 맞추기" in t["msg"]) for t in tries)


# ───────────── 11-21 (2) 자율 순회 (--auto) ─────────────
def is_home_leg(g, home=None):
    """복귀 구간: goal 이 home_pose 와 3 cm · 5° 안이면 (우선), 아니면 이름 목적지 A1 · HOME"""
    q = g.get("goal")
    if home is not None and q and q[2] is not None and math.hypot(q[0] - home[0], q[1] - home[1]) < 0.03 and abs(wrap(q[2] - home[2])) < math.radians(5):
        return True
    return leg_target(g) in ("HOME", "A1")


def leg_ref(data, g):
    """구간 g 가 출발하는 계획 자세 (앞 구간 정지점, 첫 구간이면 출발 자세). home_legs 항목은 "start" """
    if g.get("start"):
        return g["start"]
    i = next((k for k, x in enumerate(data["legs"]) if x["leg"] == g["leg"]), None)
    return data["start"] if not i else data["legs"][i - 1]["goal"]


def auto_start_ok(p, ref, g):
    """지금 자리 p 에서 구간 g 의 계획 길을 그대로 써도 되나 → (True/False, 설명). AUTO_NEAR 안이면 됨.
    첫 조각이 후진이면 정지점 방향 선 위 AUTO_ALONG 뒤까지도 (다가가기 done 자리 = 정지점보다 상자에서 먼 곳, 그 첫 후진 길 위)"""
    d = math.hypot(p[0] - ref[0], p[1] - ref[1])
    if d <= AUTO_NEAR:
        return True, f"출발 정지점과 {d * 100:.0f} cm"
    pcs = g.get("pieces") or []
    if pcs and pcs[0]["kind"] == "R" and len(ref) > 2 and ref[2] is not None:
        c, s = math.cos(ref[2]), math.sin(ref[2])
        dx, dy = p[0] - ref[0], p[1] - ref[1]
        along, lat = dx * c + dy * s, -dx * s + dy * c
        if -AUTO_ALONG <= along <= 0 and abs(lat) <= AUTO_LAT and abs(wrap(p[2] - ref[2])) <= math.radians(20):
            return True, f"출발 정지점 {-along * 100:.0f} cm 뒤 (첫 후진 길 위, 옆 {lat * 100:+.0f} cm)"
    return False, f"출발 정지점과 {d * 100:.0f} cm (> {AUTO_NEAR * 100:.0f} cm)"


def entry_heading(g, target):
    """계획 길이 target 에 들어가는 방향 + 끝 제자리: 마지막 전진 끝이 target 3 cm 안이고 그 뒤가 제자리뿐이면 (그 끝 yaw, [제자리 조각…]).
    C 의 S9 · home_legs 처럼 120° 로 들어와 +60° 도는 끝 (출발 구역은 180° 로 곧게 들어갈 자리가 비용 안). 없으면 None"""
    pcs = list((g or {}).get("pieces") or [])
    i = max((k for k, pc in enumerate(pcs) if pc["kind"] != "S"), default=None)
    if i is None or pcs[i]["kind"] != "F" or i == len(pcs) - 1:
        return None
    e = pcs[i]["poses"][-1]
    if math.hypot(e[0] - target[0], e[1] - target[1]) > 0.03:
        return None
    return e[2], [dict(pc) for pc in pcs[i + 1:]]


def planner_leg(node, g, target, via=None):
    """auto: 지금 자리에서 target (x, y, yaw) 까지 planner_server (Detour, 전진만) 길 → 전진 조각 (+ 끝 제자리) 인 구간 dict (g 복사).
    후보 (차례로): 들어가는 방향 (via = entry_heading 결과 · 없으면 target yaw) 마다 ① target 의 AP_MEASURE_D 뒤 goal + 곧게 target 까지
    (상자 앞은 비용이 높아 바로는 안 될 수 있음, 끊기 · 다가가기가 이 곧은 길 위에서) ② target 바로. via 가 있으면 target yaw 그대로도 마지막 후보.
    시작이 막혔다는 실패 (Start occupied) 일 때만 곧게 DETOUR_BACK 후진 뒤 처음부터 한 번 더 (목표 쪽 실패에 후진하면 벽 쪽으로 가 더 나빠짐 — 11-21 시뮬).
    (구간, 설명) 또는 (None, 이유)"""
    if not node.planner.wait_for_server(timeout_sec=1.0):
        return None, "planner_server 없음 (SAFETY=0?)"
    x, y, th = float(target[0]), float(target[1]), float(target[2])
    heads = ([(via[0], via[1])] if via else []) + [(th, [])]
    whys = []
    for k in range(2):
        p = node.pose()
        if p is None:
            return None, "map → base_link TF 없음"
        found, start_bad = None, False
        for hd, tail in heads:
            near = math.hypot(x - p[0], y - p[1]) < AP_MEASURE_D + 0.10
            for pre in ((False,) if near else (True, False)):
                gx, gy = (x - AP_MEASURE_D * math.cos(hd), y - AP_MEASURE_D * math.sin(hd)) if pre else (x, y)
                gm = ComputePathToPose.Goal()
                gm.goal.header.frame_id = "map"
                gm.goal.header.stamp = node.get_clock().now().to_msg()
                gm.goal.pose.position.x, gm.goal.pose.position.y = gx, gy
                gm.goal.pose.orientation.z, gm.goal.pose.orientation.w = yaw_q(hd)
                gm.planner_id = "Detour"
                gm.use_start = False
                path, why = request_path(node, gm)
                if STOP.is_set():
                    return None, "Ctrl+C"
                tag = f"{math.degrees(hd):.0f}° {'0.30 m 뒤' if pre else '바로'}"
                if path is not None and len(path) >= 2:
                    found = (path, hd, tail, pre, gx, gy, tag)
                    break
                why = why or f"점 {len(path or [])} 개"
                whys.append(f"{tag}: {why[:60]}")
                print(f"    · planner 후보 {tag} 안 됨 ({why[:90]})", flush=True)
                start_bad = start_bad or "start occupied" in why.lower()
            if found:
                break
        if found:
            break
        if k == 1 or not start_bad:
            return None, "planner 길 없음 (" + " / ".join(whys[-3:]) + ")"
        print(f"    · 시작 자리가 막힘 — 곧게 {DETOUR_BACK:.2f} m 후진 뒤 한 번 더", flush=True)
        rb, mb = follow_piece(node, straight_back_piece(p, DETOUR_BACK), final=False)
        if not rb:
            return None, f"planner 길 없음 · 후진 못 함 ({mb})"
    path, hd, tail, pre, gx, gy, tag = found
    poses = list(path)
    if pre:   # 곧은 들어가기
        n = max(int(round(AP_MEASURE_D / 0.01)), 1)
        poses += [(gx + AP_MEASURE_D * i / n * math.cos(hd), gy + AP_MEASURE_D * i / n * math.sin(hd), hd) for i in range(1, n + 1)]
    out = dict(g, pieces=[{"kind": "F", "poses": poses}] + list(tail), goal=[x, y, th], via_planner=True, to=leg_target(g),
               name=g["name"] if g["name"].endswith("(planner)") else g["name"] + " (planner)")
    out.pop("start", None)
    return out, f"planner 길 {path_len(poses):.2f} m ({tag}" + (f" + 끝 제자리 {sum(math.degrees(pc.get('dth', 0)) for pc in tail):+.0f}°" if tail else "") + ")"


def home_settle(node, home, tol_xy):
    """출발 자세 보정: tol_xy 넘게 어긋나면 곧게 한 번 (≤ HOME_FIX_MAX, 경로 방향이 지금 방향에서 30° 넘게 돌면 지금 방향 선 위로만 —
    v2_filter 제자리 90° 예산을 두 번 쓰지 않게) → 방향 맞추기. (어긋남 m, 방향 오차 rad, 메모)"""
    notes = []
    p = node.pose()
    if p is None:
        return 9.0, 0.0, ["map TF 없음"]
    hx, hy, hth = home
    d = math.hypot(hx - p[0], hy - p[1])
    err = wrap(hth - p[2])
    if d > tol_xy and d <= HOME_FIX_MAX and abs(err) > math.radians(30) and not STOP.is_set():
        # 방향이 크게 다르면 (계획 끝 방향 ≠ 출발 자세, 예 wide S9 90°) 먼저 돌아 출발 방향 선 위에서 곧게 — 제자리 한 번만
        print(f"    · 출발 자세 방향 먼저 {math.degrees(err):+.0f}°", flush=True)
        r, m = follow_piece(node, {"kind": "S", "dth": err})
        notes.append(f"방향 먼저 {math.degrees(err):+.0f}°" + ("" if r else f" 못 함 ({m})"))
        p = node.pose() or p
    if d > tol_xy and not STOP.is_set():
        if d > HOME_FIX_MAX:
            notes.append(f"{d * 100:.1f} cm > 보정 한도 {HOME_FIX_MAX * 100:.0f} cm — 곧게 보정 안 함")
        else:
            line = math.atan2(hy - p[1], hx - p[0])
            fwd = math.cos(line - p[2]) >= 0
            hd = line if fwd else wrap(line + math.pi)
            # 10°: 계획 끝 제자리 (C 의 S9 · home_legs 끝 +60°) 로 이미 예산을 쓴 뒤라 조각 방향 맞추기 (HEAD_FIX) 로 더 돌지 않게 — 넘으면 지금 방향 선 위로만
            if abs(wrap(hd - p[2])) <= math.radians(10):
                n = max(int(round(d / 0.01)), 1)
                pc = {"kind": "F" if fwd else "R", "poses": [(p[0] + (hx - p[0]) * i / n, p[1] + (hy - p[1]) * i / n, hd) for i in range(n + 1)]}
                what = f"{'전진' if fwd else '후진'} {d * 100:.1f} cm"
            else:
                along = (hx - p[0]) * math.cos(p[2]) + (hy - p[1]) * math.sin(p[2])
                pc = (None if abs(along) < 0.01 else straight_back_piece(p, -along) if along < 0 else
                      {"kind": "F", "poses": [(p[0] + along * i / 20 * math.cos(p[2]), p[1] + along * i / 20 * math.sin(p[2]), p[2]) for i in range(21)]})
                what = f"지금 방향 선 위로만 {'전진' if along >= 0 else '후진'} {abs(along) * 100:.1f} cm (옆 어긋남은 그대로)"
            if pc is None:
                what = f"옆 어긋남 {d * 100:.1f} cm — 곧게로는 못 고침 (그대로)"
                r, m = True, ""
            else:
                print(f"    · 출발 자세 곧게 보정: {what}", flush=True)
                r, m = follow_piece(node, pc)
            notes.append(what + ("" if r else f" 못 함 ({m})"))
    p = node.pose() or p
    err = wrap(hth - p[2])
    if abs(err) > YAW_FIX and not STOP.is_set():
        print(f"    · 출발 자세 방향 맞추기 {math.degrees(err):+.0f}°", flush=True)
        r, m = follow_piece(node, {"kind": "S", "dth": err})
        notes.append(f"방향 {math.degrees(err):+.0f}°" + ("" if r else f" 못 함 ({m})"))
    sleep_s(node, 0.5)
    p = node.pose() or p
    return math.hypot(hx - p[0], hy - p[1]), abs(wrap(hth - p[2])), notes


def run_auto(a, node, data, legs, results):
    """11-21 (2) 자율 순회: legs 순서대로 곳마다 (상자 다가가기 포함) → 머묾 --dwell s → 다음. 곳 실패 (run_leg False · planner 길 없음) 는
    건너뛰고 계속, 연속 --auto-fail-home 곳 실패 · 끝나면 출발 자세로 복귀 (계획 복귀 길 · home_legs · planner 순, 실패하면 planner 로 한 번 더).
    Ctrl+C 면 아무것도 안 하고 끝. step 이면 곳마다 Enter (q = 복귀로), 머묾 없음"""
    home = list(data.get("home_pose") or data["start"])
    all_stops = [g for g in data["legs"] if not is_home_leg(g, home)]
    total = len(all_stops)
    pos = {g["leg"]: i + 1 for i, g in enumerate(all_stops)}
    stops = [g for g in legs if not is_home_leg(g, home)]
    home_plan = next((g for g in data["legs"] if is_home_leg(g, home)), None)
    home_legs = data.get("home_legs") or {}
    order = list(data.get("order") or [leg_target(g) for g in all_stops])
    if [leg_target(g) for g in all_stops] != order:
        print(f"  ⚠ 계획 order {order} 가 구간 순서 {[leg_target(g) for g in all_stops]} 와 다름 — 구간 순서대로 감", flush=True)
    tol_xy, tol_yaw = a.home_tol_xy, math.radians(a.home_tol_yaw)
    print(f"자율 순회: {' → '.join(leg_target(g) for g in stops)} → 출발 자세 ({home[0]:.3f}, {home[1]:.3f}, {math.degrees(home[2]):.0f}°) · "
          f"머묾 {0 if a.step else a.dwell:.0f} s · 연속 {a.auto_fail_home} 곳 실패면 복귀 · 복귀 판정 {tol_xy * 100:.0f} cm · {a.home_tol_yaw:.0f}° · "
          f"복귀 길 {'home_legs ' + str(len(home_legs)) + ' 곳' if home_legs else '계획 마지막 구간 · 없으면 planner'}", flush=True)
    if os.environ.get("DEMO_AUTO_SKIP") == "1":
        print("  · DEMO_AUTO_SKIP=1 은 auto 에서 안 씀 (조각 대신 곳 단위로 건너뜀)", flush=True)
    rec, t_run, consec = [], node.now_s(), 0
    extra = [max(g["leg"] for g in data["legs"]) + 1]   # 복귀 구간 새 번호 (save · moves 가 data["legs"] 에서 찾음)

    def add_rec(name, res, t0, r=None, via=False, why="", leg=None):
        ap = (r or {}).get("approach") or {}
        aft = ap.get("after") if ap.get("status") == "done" else None
        rec.append({"name": name, "leg": leg, "result": res, "leg_s": round(node.now_s() - t0, 1), "cum_s": round(node.now_s() - t_run, 1),
                    "err_cm": (r or {}).get("end_err_cm"), "gap_cm": aft["gap_cm"] if aft else None, "lat_cm": aft["lat_cm"] if aft else None,
                    "via_planner": via, "why": why})

    def fail_result(g, why, t0):
        p = node.pose()
        gx, gy = g["goal"][0], g["goal"][1]
        results.append({"seg": f"S{g['leg']}", "name": g["name"], "points": g["points"], "ok": False,
                        "tries": [{"status": "fail", "msg": why, "t": round(node.now_s() - t0, 1)}], "sim_s": round(node.now_s() - t0, 1),
                        "end_err_cm": round(math.hypot(gx - p[0], gy - p[1]) * 100, 1) if p else None, "end_err_deg": None,
                        "head_fix_deg": [], "head_skip_deg": [], "recover": [], "detour": False, "detour_n": 0, "hold_n": 0, "box_quiet_n": 0, "approach": None})

    for idx, g in enumerate(stops):
        if STOP.is_set():
            break
        nm = leg_target(g)
        prog = f"[{pos[g['leg']]}/{total}] "
        node.prog = prog
        if a.step and not ask(f"\n{prog}▶ {g['name']} — Enter 출발 · q 끝 (출발 자세로 복귀) : "):
            break
        print(f"\n{prog}▶ {g['name']}", flush=True)
        t0 = node.now_s()
        node.step = g["leg"]
        p = node.pose()
        ok_plan, why = auto_start_ok(p, leg_ref(data, g), g) if p else (False, "map TF 없음")
        via = False
        gg = g
        if not ok_plan:
            print(f"  · {why} — planner 로 {nm} 까지 길 받음", flush=True)
            gg, info = planner_leg(node, g, g["goal"])
            if gg is None:
                print(f"{prog}✗ {g['name']}: {info} — 이 곳 건너뜀", flush=True)
                fail_result(g, f"auto: {info}", t0)
                add_rec(nm, "skip", t0, results[-1], False, info, g["leg"])
                consec += 1
                if consec >= a.auto_fail_home:
                    print(f"  ✗ 연속 {consec} 곳 실패 — 출발 자세로 복귀", flush=True)
                    break
                continue
            print(f"  · {info}", flush=True)
            via = True
        ok = run_leg(node, gg, results)
        if STOP.is_set():
            add_rec(nm, "skip" if not ok else ("ok" if (results[-1].get("approach") or {}).get("status") == "done" else "nogap"), t0, results[-1], via, "Ctrl+C", g["leg"])
            break
        r = results[-1]
        if not ok:
            why = next((t["msg"] for t in reversed(r["tries"]) if t["status"] == "fail"), "")
            add_rec(nm, "skip", t0, r, via, why, g["leg"])
            consec += 1
            print(f"  ✗ {nm} 건너뜀 (연속 실패 {consec}/{a.auto_fail_home})", flush=True)
            if consec >= a.auto_fail_home:
                print(f"  ✗ 연속 {consec} 곳 실패 — 출발 자세로 복귀", flush=True)
                break
            continue
        consec = 0
        apr = r.get("approach") or {}
        done = apr.get("status") == "done"
        add_rec(nm, "ok" if done or not ap_target(g) else "nogap", t0, r, via, "" if done or not ap_target(g) else apr.get("why", ""), g["leg"])
        if not a.step and a.dwell > 0:
            gap = f"{apr['after']['gap_cm']:.1f} cm" if done else "-"
            print(f"  ⏸ {nm} 머묾 {a.dwell:.0f} s ({pos[g['leg']]}/{total}) · 틈 {gap}", flush=True)
            sleep_s(node, a.dwell)
    node.prog = ""

    # ── 출발 자세 복귀 ──
    home_info = {"ok": False, "err_cm": None, "err_deg": None, "how": "", "notes": []}
    if os.environ.get("DEMO_TEST_PIECE_FAIL") == "1":
        node.test_off = True
        print("  · 시험용 DEMO_TEST_PIECE_FAIL 은 복귀 단계에서 끔", flush=True)
    if not STOP.is_set():
        node.prog = "[복귀] "
        if a.step and not ask("\n[복귀] ▶ 출발 자세로 복귀 — Enter 출발 · q 끝 (복귀 안 함) : "):   # step = 곳마다처럼 복귀도 Enter
            home_info["how"] = "사람이 안 함 (q)"
        else:
            for attempt in range(2):
                if STOP.is_set():
                    break
                t0 = node.now_s()
                p = node.pose()
                gh, how = None, ""
                if p is not None and math.hypot(home[0] - p[0], home[1] - p[1]) <= AUTO_NEAR:
                    how = "이미 출발 자리 가까이 — 보정만"
                elif attempt == 0 and p is not None:
                    if home_plan is not None and auto_start_ok(p, leg_ref(data, home_plan), home_plan)[0]:
                        gh, how = dict(home_plan, goal=home), f"계획 복귀 길 {home_plan['name']}"
                    else:
                        s = next((k for k, e in home_legs.items() if auto_start_ok(p, e.get("start") or next(
                            (x["goal"] for x in data["legs"] if leg_target(x) == k), [99, 99, None]), e)[0]), None)
                        if s is not None:
                            e = home_legs[s]
                            gh = {"name": e.get("name", f"H {s}→HOME"), "goal": home, "pieces": e["pieces"], "start": e.get("start"), "to": "HOME"}
                            how = f"home_legs[{s}]"
                if gh is None and not how:
                    if attempt == 1:
                        relocalize(node, "복귀 다시 전")
                    print(f"  · 복귀 길: planner ({'두 번째' if attempt else '계획 · home_legs 길 시작점에서 멂'})", flush=True)
                    base = {"name": "H →HOME (planner)", "goal": home, "pieces": [], "to": "HOME"}
                    gh, info = planner_leg(node, base, home, entry_heading(home_plan, home))
                    how = f"planner ({info})"
                    if gh is None:
                        print(f"  ✗ 복귀 {info}", flush=True)
                        home_info["notes"].append(f"{attempt + 1} 번째: {info}")
                        continue
                print(f"\n[복귀] ▶ {how}", flush=True)
                home_info["how"] = how
                ok = True
                if gh is not None:
                    if not (home_plan is not None and gh.get("leg") == home_plan["leg"] and not gh.get("via_planner")):
                        k = extra[0]   # 계획 밖 복귀 구간 (home_legs · planner): 새 번호로 data["legs"] 에 (save · moves 가 번호로 찾음)
                        extra[0] += 1
                        last = max((x["leg"] for x in rec if x["leg"] is not None and x["name"] != "HOME"), default=None)
                        gh = dict(gh, leg=k, points=[pos.get(last, 0), total + 1], name=f"S{k} {gh['name']}")
                        gh.pop("start", None)
                        data["legs"].append(gh)
                    ok = run_leg(node, gh, results)
                if STOP.is_set():
                    break
                dpos, dyaw, notes = home_settle(node, home, tol_xy)
                home_info["notes"] += notes
                home_info.update(err_cm=round(dpos * 100, 1), err_deg=round(math.degrees(dyaw), 1))
                hok = ok and dpos <= tol_xy and dyaw <= tol_yaw
                if gh is not None:
                    r = results[-1]
                    r.update(ok=hok, end_err_cm=home_info["err_cm"], end_err_deg=home_info["err_deg"],
                             home={"tol_cm": round(tol_xy * 100, 1), "tol_deg": a.home_tol_yaw, "notes": notes})
                print(f"[복귀] {'✓' if hok else '✗'} 출발 자세 오차 {dpos * 100:.1f} cm · {math.degrees(dyaw):.1f}° (판정 {tol_xy * 100:.0f} cm · {a.home_tol_yaw:.0f}°)", flush=True)
                add_rec("HOME", "ok" if hok else "skip", t0, {"end_err_cm": home_info["err_cm"]}, gh is not None and gh.get("via_planner", False), "", gh["leg"] if gh else None)
                if hok:
                    home_info["ok"] = True
                    break
                if dpos <= AUTO_NEAR and ok:   # 길은 왔고 판정만 넘음 — planner 다시 가도 같음
                    break
            if not home_info["ok"] and not STOP.is_set() and home_info["how"] != "사람이 안 함 (q)":
                print("✗ 복귀 실패 — 리모컨으로 옮기기", flush=True)
    node.prog = ""

    # ── 끝 요약 ──
    mark = {"ok": "✓", "nogap": "↷5cm못함", "skip": "✗건너뜀"}
    f1 = lambda v, s="": "-" if v is None else f"{v:{s}.1f}"   # noqa: E731
    lines = [f"자율 순회 요약 ({'loopback 시뮬' if a.sim else '실물'} · 계획 {data['plan']})",
             f"{'곳':<6} | {'결과':<9} | {'구간 s':>6} | {'누적 s':>7} | {'어긋남 cm':>8} | {'틈 cm':>5} | {'옆 cm':>5}"]
    for x in rec:
        lines.append(f"{x['name']:<6} | {mark[x['result']] if x['name'] != 'HOME' else ('✓' if x['result'] == 'ok' else '✗'):<9} | {x['leg_s']:>6.1f} | {x['cum_s']:>7.1f} | "
                     f"{f1(x['err_cm']):>8} | {f1(x['gap_cm']):>5} | {f1(x['lat_cm'], '+'):>5}" + (" · planner" if x["via_planner"] else "")
                     + (f" · {x['why']}" if x["why"] and x["result"] != "ok" else ""))
    st = [x for x in rec if x["name"] != "HOME"]
    n_ok = sum(1 for x in st if x["result"] != "skip")
    n_gap = sum(1 for x in st if x["result"] == "ok")
    lines.append(f"전체 {n_ok}/{total} (5 cm {n_gap}) · 총 {node.now_s() - t_run:.1f} s · 출발 자세 복귀 "
                 + (f"오차 {home_info['err_cm']:.1f} cm · {home_info['err_deg']:.1f}°" if home_info["err_cm"] is not None else "안 함")
                 + (" ✓" if home_info["ok"] else " ✗"))
    skipped = [x["name"] for x in st if x["result"] == "skip"]
    if STOP.is_set():
        lines.append("(Ctrl+C 로 멈춤 — 복귀 안 함)")
    first_bad = next((x for x in st if x["result"] == "skip"), None)
    seen = {x["leg"] for x in st}
    left = [g for g in stops if g["leg"] not in seen]
    again_g = next((g for g in stops if g["leg"] == first_bad["leg"]), None) if first_bad else (left[0] if left else None)
    again = leg_target(again_g) if again_g else None
    if again:
        i = next(k for k, x in enumerate(data["legs"]) if x["leg"] == again_g["leg"])
        put = "출발 자세" if i == 0 else f"{leg_target(data['legs'][i - 1])} 정지점"
        lines.append(f"다시: 로봇을 {put}에 놓고 AUTO_FROM={again} ./run_demo.sh {'sim' if a.sim else 'real'} slow auto init")   # 로봇을 새로 놓으므로 늘 init (/initialpose)
    print("\n" + "\n".join(lines), flush=True)
    data["_auto_txt"] = lines
    data["_auto"] = {"order": order, "skipped": skipped, "approach_fail": [x["name"] for x in st if x["result"] == "nogap"],
                     "home_ok": home_info["ok"], "home_err_cm": home_info["err_cm"], "home_err_deg": home_info["err_deg"],
                     "via_planner": [x["name"] for x in rec if x["via_planner"]], "home_how": home_info["how"], "home_notes": home_info["notes"],
                     "places": rec, "total_s": round(node.now_s() - t_run, 1), "resume_from": again, "stopped": STOP.is_set()}


def run_route(a, node, data, LIB, PTS, results):
    """11-15 기본 경로 + 들르기. 정지점마다: 들르기 요청이 있으면 그곳 먼저 (길 모음 cur>요청), 없으면 기본 경로 다음 곳"""
    base = list(data["route"])
    if base and base[0] == "HOME":
        base = base[1:]
    auto = {norm(k): [norm(x) for x in v] for k, v in data.get("auto_detour", {}).items()}
    dwell = float(data.get("dwell_s", 0.5))
    cur, idx, k, prev = "HOME", 0, 0, None
    visited = ["HOME"]
    while not STOP.is_set():
        if k > 0:
            for nm in auto.pop(cur, []):   # 시뮬 시험: 이 정지점에서 들를 곳 (계획 파일 auto_detour)
                with node.plock:
                    node.pending.append(nm)
                print(f"  ↪ 들르기 (계획 파일 auto_detour): {nm}", flush=True)
            if not a.step and dwell > 0:
                sleep_s(node, dwell)   # 도착 알림 뒤 들르기 요청을 받을 틈
        detour = None
        with node.plock:
            while node.pending and detour is None:
                nm = node.pending.popleft()
                if nm == cur or f"{cur}>{nm}" not in LIB:
                    print(f"  ⚠ 들르기 {nm} 건너뜀 ({'지금 자리' if nm == cur else f'길 {cur}>{nm} 이 길 모음에 없음'})", flush=True)
                    continue
                detour = nm
        if detour is None and idx >= len(base):
            break
        nxt = detour or base[idx]
        if a.step:
            go, nm = ask_next(f"\n▶ 지금 {cur} · 다음 {nxt}" + (" (들르기)" if detour else "")
                              + " — Enter 출발 · L1 / L2 / L3 (또는 상자 이름) = 그곳 먼저 · q 끝 : ")
            if not go:
                break
            if nm and nm != nxt:
                if nm not in NAMES or nm == cur or f"{cur}>{nm}" not in LIB:
                    print(f"  ✗ {nm} 으로는 못 감 (지금 {cur})", flush=True)
                    if detour:
                        with node.plock:
                            node.pending.appendleft(detour)
                    continue
                if detour:   # 밀린 들르기는 다음 정지점에서
                    with node.plock:
                        node.pending.appendleft(detour)
                detour, nxt = nm, nm
        if detour is None:
            idx += 1
        L = LIB[f"{cur}>{nxt}"]
        if prev is not None:   # 이음매 규칙 v2 (앞 길 도착 제자리 + 이 길 처음 제자리 ≤ 90° · 이어지는 후진 ≤ 0.40 m)
            ds, fr = first_spin(L["pieces"]), first_rev(L["pieces"])
            if prev["arrive_spin_deg"] + ds > SPIN_MAX_DEG + 1e-6:
                print(f"  ⚠ 이음매: 앞 길 도착 제자리 {prev['arrive_spin_deg']}° + 이 길 처음 제자리 {ds:.0f}° > {SPIN_MAX_DEG:.0f}°", flush=True)
            if prev["arrive_rev_m"] + fr > REV_MAX + 1e-6:
                print(f"  ⚠ 이음매: 앞 길 끝 후진 {prev['arrive_rev_m']} + 이 길 첫 후진 {fr:.2f} > {REV_MAX} m", flush=True)
        g = {"leg": k, "name": f"S{k} {'signal' if cur == 'HOME' else cur}→{'A1' if nxt == 'HOME' else nxt}", "points": [k, k + 1],
             "goal": L["goal"], "pieces": L["pieces"], "detour": bool(detour), "to": nxt}
        data["legs"].append(g)
        if detour:
            print(f"  ↪ 들르기 {nxt} → 그 뒤 기본 경로 {base[idx] if idx < len(base) else '(끝)'} 로 복귀", flush=True)
        if not run_leg(node, g, results):
            break
        visited.append(nxt)
        prev, cur, k = L, nxt, k + 1
        if cur == "HOME":
            with node.plock:
                left = list(node.pending)
            if left:
                print(f"  (출발 구역 도착 — 남은 들르기 요청 {left} 은 안 씀)", flush=True)
            break
    # 기록용 경로 yaml (주행기록 HTML 이 정지점을 읽음)
    lines = [f"# 기본 경로 + 들르기 (demo_follow route, {time.strftime('%Y-%m-%d %H:%M')}): 기본 {' → '.join(data['route'])} · 실제 {' → '.join(visited)}",
             "# 정지점 = 길 모음 (상자 실측) · 들른 곳은 detour", "route:"]
    s0 = PTS["HOME"]["start"]
    lines.append(f"  - {{stop: signal, x: {s0[0]:.4f}, y: {s0[1]:.4f}, face: {math.degrees(s0[2]):.1f}}}")
    for g in data["legs"][:len(visited) - 1]:
        y = g["to"]
        if y == "HOME":
            lines.append("  - A1   # 출발 구역으로 돌아옴")
        else:
            q = PTS[y]["goal"]
            lines.append(f"  - {{stop: {y}, x: {q[0]:.4f}, y: {q[1]:.4f}, face: {math.degrees(q[2]):.1f}}}" + ("   # 들르기" if g.get("detour") else ""))
    data["course_text"] = "\n".join(lines) + "\n"
    print(f"\n실제로 간 곳: {' → '.join(visited)}", flush=True)

def save(a, data, results, rows, n_legs):
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, data["course"]), "w", encoding="utf-8").write(data["course_text"])
    with open(os.path.join(a.out, "track.csv"), "w", encoding="utf-8") as f:
        f.write("t,x,y,yaw,v,w,step,v_nav,w_nav,loc\n")
        for r in rows:
            f.write(f"{r[0]:.2f},{r[1]:.4f},{r[2]:.4f},{r[3]:.1f},{r[4]:.3f},{r[5]:.3f},{r[6]},{r[7]:.3f},{r[8]:.3f},{r[9]}\n")
    json.dump(results, open(os.path.join(a.out, "course_result.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    moves = []
    for res in results:
        k = int(res["seg"][1:])
        R = [r for r in rows if r[6] == k]
        if len(R) < 2:
            continue
        L = rev = 0.0
        for p, q in zip(R, R[1:]):
            ds = math.hypot(q[1] - p[1], q[2] - p[2])
            L += ds
            rev += ds if q[4] < -0.02 else 0.0
        g = next(x for x in data["legs"] if x["leg"] == k)
        moves.append({"단계번호": k, "목표이름": res["name"].split(" ", 1)[1], "단계": res["name"], "결과": "성공" if res["ok"] else "실패",
                      "시간_s": round(R[-1][0] - R[0][0], 1), "길이_m": round(L, 2), "후진_m": round(rev, 2), "제자리_번": 0, "제자리_deg": 0,
                      "재계획_바뀜": 0, "복구": 0, "멈춤_s": 0, "최소여유_cm": "", "목표_x": g["goal"][0], "목표_y": g["goal"][1],
                      "도착_x": round(R[-1][1], 3), "도착_y": round(R[-1][2], 3)})
    json.dump(moves, open(os.path.join(a.out, "moves.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    n_ok = sum(1 for r in results if r["ok"])
    with open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(f"계획 따라가기 ({'loopback 시뮬' if a.sim else '실물'}): 구간 {n_ok}/{n_legs} 성공 · 계획 {data['plan']}\n")
        for r in results:
            f.write(f"{r['name']}: {'성공' if r['ok'] else '실패'} · {r['sim_s']} s · 정지점 어긋남 {r['end_err_cm']} cm · {r['end_err_deg']}°"
                    + (f" · {r['tries'][-1]['msg']}" if not r["ok"] else "") + "\n")
        if data.get("_auto_txt"):   # 11-21 (2) auto 요약
            f.write("\n" + "\n".join(data["_auto_txt"]) + "\n")
    if data.get("_auto") is not None:   # course_result.json 은 주행기록 HTML 이 list 로 읽으므로 따로
        json.dump({"auto": data["_auto"]}, open(os.path.join(a.out, "auto_result.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"\n구간 {n_ok}/{n_legs} 성공 → 기록 {a.out}", flush=True)


if __name__ == "__main__":
    main()
