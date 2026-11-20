"""launch/base_bringup.launch.py 의 ports:=auto (OpenCR ttyACM 찾기) — 가짜 /dev · /sys/class/tty 로"""
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('base_bringup', os.path.join(os.path.dirname(HERE), 'launch', 'base_bringup.launch.py'))
bb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bb)

OPENCR = ('0483', '5740', 'ROBOTIS', 'OpenCR Virtual ComPort in FS Mode')


def make(tmp_path, ports):
    """ports = {'ttyACM0': (vid, pid, 제조사, 제품) | None(sysfs 없음)} → (sys_tty, dev)"""
    dev, sys_tty = tmp_path / 'dev', tmp_path / 'sys' / 'class' / 'tty'
    dev.mkdir(), sys_tty.mkdir(parents=True)
    (dev / 'ttyUSB0').touch()   # ttyACM 아님 → 무시
    for i, (name, info) in enumerate(ports.items()):
        (dev / name).touch()
        if info is None:
            continue
        usb = tmp_path / 'sys' / 'devices' / 'usb1' / f'1-{i}'
        intf = usb / f'1-{i}:1.0'
        intf.mkdir(parents=True)
        for f, v in zip(('idVendor', 'idProduct', 'manufacturer', 'product'), info):
            (usb / f).write_text(v + '\n')
        (sys_tty / name).mkdir()
        os.symlink(intf, sys_tty / name / 'device')
    return str(sys_tty), str(dev)


def test_two_opencr_after_reset_numbers(tmp_path):
    s, d = make(tmp_path, {'ttyACM3': OPENCR, 'ttyACM2': OPENCR})
    assert bb.find_opencr_ports(s, d) == ([f'{d}/ttyACM2', f'{d}/ttyACM3'], [])


def test_numeric_sort_and_other_acm_excluded(tmp_path):
    other = ('2341', '0043', 'Arduino', 'Uno')
    s, d = make(tmp_path, {'ttyACM10': OPENCR, 'ttyACM2': OPENCR, 'ttyACM0': other})
    assert bb.find_opencr_ports(s, d) == ([f'{d}/ttyACM2', f'{d}/ttyACM10'], [f'{d}/ttyACM0'])


def test_name_match_and_missing_sysfs(tmp_path):
    s, d = make(tmp_path, {'ttyACM0': ('ffff', 'ffff', 'ROBOTIS', 'x'), 'ttyACM1': None})
    assert bb.find_opencr_ports(s, d) == ([f'{d}/ttyACM0', f'{d}/ttyACM1'], []), 'sysfs 없으면 후보로 둠'


def test_no_dev_dir(tmp_path):
    assert bb.find_opencr_ports(str(tmp_path / 'x'), str(tmp_path / 'none')) == ([], [])


def test_resolve():
    assert bb.resolve_ports('/dev/ttyACM5,/dev/ttyACM6', lambda: pytest.fail('직접 주면 찾지 않음')) == '/dev/ttyACM5,/dev/ttyACM6'
    assert bb.resolve_ports('auto', lambda: (['/dev/ttyACM2', '/dev/ttyACM3'], [])) == '/dev/ttyACM2,/dev/ttyACM3'
    for found in ([], ['/dev/ttyACM0'], ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']):
        with pytest.raises(RuntimeError, match='OpenCR 포트를 2 개'):
            bb.resolve_ports('AUTO', lambda f=found: (f, ['/dev/ttyACM9']))
