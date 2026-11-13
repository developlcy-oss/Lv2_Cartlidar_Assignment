"""scripts/rc100_joy.py 의 버튼 → /joy 변환 (ROS 없이) + 번호가 teleop.yaml · opencr_hardware.cpp 기본값과 맞는지"""
import importlib.util
import os
import re

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location('rc100_joy', os.path.join(PKG, 'scripts', 'rc100_joy.py'))
rc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rc)
B = rc.BTN


def test_arrows_move_and_hold_enable():
    axes, buttons = rc.joy_from_bits(B['U'], 0.5, 0.5)
    assert axes[rc.AX_LIN] == pytest.approx(0.5) and axes[rc.AX_YAW] == 0.0
    assert buttons[rc.J_ENABLE] == 1, '방향 버튼을 누르는 동안 데드맨 자동'
    axes, buttons = rc.joy_from_bits(B['D'] | B['L'], 0.5, 0.3)
    assert axes[rc.AX_LIN] == pytest.approx(-0.5) and axes[rc.AX_YAW] == pytest.approx(0.3), '왼쪽 = + (반시계)'
    axes, _ = rc.joy_from_bits(B['R'], 0.5, 0.3)
    assert axes[rc.AX_YAW] == pytest.approx(-0.3)
    axes, buttons = rc.joy_from_bits(B['U'] | B['D'], 0.5, 0.5)
    assert axes[rc.AX_LIN] == 0.0, 'U + D 는 서로 지움'


def test_no_arrow_no_enable():
    for bits in (0, B['1'], B['4'], B['5'], B['6'], B['2'] | B['3']):
        axes, buttons = rc.joy_from_bits(bits, 1.0, 1.0)
        assert buttons[rc.J_ENABLE] == 0 and axes == [0.0] * rc.N_AXES, rc.names(bits)


def test_estop_and_reset():
    _, buttons = rc.joy_from_bits(B['1'], 0.5, 0.5)
    assert buttons[rc.J_ESTOP] == 1 and buttons[rc.J_RESET] == 0
    _, buttons = rc.joy_from_bits(B['2'], 0.5, 0.5)
    assert buttons[rc.J_RESET] == 0, '2 만으로는 안 풀림'
    _, buttons = rc.joy_from_bits(B['3'], 0.5, 0.5)
    assert buttons[rc.J_RESET] == 0
    _, buttons = rc.joy_from_bits(B['2'] | B['3'], 0.5, 0.5)
    assert buttons[rc.J_RESET] == 1 and buttons[rc.J_ESTOP] == 0
    _, buttons = rc.joy_from_bits(0x3FF, 0.5, 0.5)
    assert len(buttons) == rc.N_BUTTONS and buttons[9] == 0 and buttons[8] == 0, '미션 시작 · 정지 (joy_teleop 9 · 8) 는 안 누름'


def test_effective_bits_age():
    u = B['U']
    assert rc.effective_bits(u, 0, 0.5) == 0, '받은 적 없음 (예전 펌웨어 0)'
    assert rc.effective_bits(u, 1, 0.5) == u
    assert rc.effective_bits(u, 501, 0.5) == u
    assert rc.effective_bits(u, 502, 0.5) == 0, 'hold_timeout 넘으면 뗀 것으로'
    assert rc.effective_bits(u, 100000, 0.0) == u, 'hold_timeout 0 = 끔'


def test_indices_match_teleop_and_host_defaults():
    t = yaml.safe_load(open(os.path.join(PKG, 'config', 'teleop.yaml')))['teleop_twist_joy_node']['ros__parameters']
    assert t['enable_button'] == rc.J_ENABLE and t['require_enable_button'] is True
    assert t['axis_linear']['x'] == rc.AX_LIN and t['axis_angular']['yaw'] == rc.AX_YAW
    src = open(os.path.join(PKG, 'src', 'opencr_hardware.cpp'), encoding='utf-8').read()
    assert re.search(r'"estop_button",\s*"(\d+)"', src).group(1) == str(rc.J_ESTOP)
    assert re.search(r'"reset_button",\s*"(\d+)"', src).group(1) == str(rc.J_RESET)
    assert t['scale_linear']['x'] * 0.5 <= 0.1 + 1e-9 and t['scale_angular']['yaw'] * 0.5 <= 0.5 + 1e-9, '기본 속도 0.1 m/s · 0.5 rad/s'


def test_bits_match_protocol_comment():
    hdr = open(os.path.join(PKG, 'include', 'opencr_hardware', 'opencr_protocol.hpp'), encoding='utf-8').read()
    assert 'U 1 · D 2 · L 4 · R 8 · 1 16 · 2 32 · 3 64 · 4 128 · 5 256 · 6 512' in hdr
    assert [B[k] for k in 'UDLR123456'] == [1, 2, 4, 8, 16, 32, 64, 128, 256, 512]
