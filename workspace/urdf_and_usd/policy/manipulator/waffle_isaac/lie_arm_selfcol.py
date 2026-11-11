# [복사본 · 2026-11-11 Onbeom, #192] 원본: 팔 담당 LCY (developlcy-oss) 릴리스 rl-20261110 의 waffle_isaac/arm_selfcol.py (md5 45f55ebbc933).
# 누운 직육면체 세워 놓기 환경 (lieupright_env.py) 이 상속하는 릴리스 판이 dev 에 없어 새 이름으로 복사함. 원본에서 import 줄만 바꿈 (아래 '# [복사본]' 표시).
# dev 에 릴리스 판이 들어오면 이 파일을 지우고 lieupright_env* 의 import 를 원래 이름으로 되돌림. 기존 파일은 고치지 않음.
"""팔끼리 자기 충돌 판정 (2026-11-06): 팀 USD 는 팔 몸체끼리 충돌을 모두 뺐음 (FilteredPairs) -> 시뮬레이션에서 팔끼리 통과함.
팀 URDF 충돌 상자 (한 변 15 mm 넘는 것) 를 그대로 써서, 붙어 있지 않은 팔 몸체 쌍의 상자 겹침 깊이를 분리축 판정으로 잼 (GPU, 대 전체 한 번에).

- 쌍: link2 (2 번 모터 · 어깨 탑) ↔ link4 · link5 · 집게, link3 (윗팔) ↔ link5 · 집게, link4 (아랫팔) ↔ 집게.
- 한 링크의 상자는 모두 같은 자세라 쌍마다 분리축 15 개만 계산.
- 실물 전원 켤 때 자세 (stow) 가 이 상자로 link2 ↔ link4 4.1 mm 겹침 (실물은 그 자세로 섬) -> 상자가 실물보다 조금 큼. 판정 기준은 겹침 깊이 5 mm 넘음 (cfg.self_box_tol).
- 팀 12 칸 경로 경유점 2165 개 · 시작 자세 · 주행 자세: 5 mm 넘는 겹침 0 (기록/환경변경_이력.md).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import torch
from isaaclab.utils.math import matrix_from_quat

PAIRS = (("link2", "link4"), ("link2", "link5"), ("link2", "gripper_roll_link"),
         ("link3", "link5"), ("link3", "gripper_roll_link"), ("link4", "gripper_roll_link"))


def load_boxes(urdf_path: str, links, min_size: float = 0.015):
    """URDF 충돌 상자 -> {링크: (중심 (k, 3), 반 크기 (k, 3))} (링크 좌표, 회전 없는 상자만)."""
    out = {}
    for l in ET.parse(urdf_path).getroot().findall("link"):
        n = l.get("name")
        if n not in links:
            continue
        cs, hs = [], []
        for c in l.findall("collision"):
            g = c.find("geometry")[0]
            if g.tag != "box":
                continue
            s = [float(v) for v in g.get("size").split()]
            if max(s) < min_size:
                continue
            o = c.find("origin")
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            if any(abs(v) > 1e-6 for v in rpy):
                raise ValueError(f"{n}: 회전된 충돌 상자는 아직 안 다룸")
            cs.append([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            hs.append([v / 2 for v in s])
        out[n] = (torch.tensor(cs), torch.tensor(hs))
    return out


class ArmSelfCollision:
    """depth() -> (대,) 붙어 있지 않은 팔 몸체 쌍 상자 겹침 깊이 최대 [m] (겹치지 않으면 음수 = 떨어진 정도의 하한)."""

    def __init__(self, robot, urdf_path: str, device):
        links = sorted({x for p in PAIRS for x in p})
        boxes = load_boxes(urdf_path, links)
        self.robot = robot
        self.ids = {k: robot.find_bodies(k)[0][0] for k in links}
        self.box = {k: (c.to(device), h.to(device)) for k, (c, h) in boxes.items()}

    def _pair(self, a, b, P, Q):
        ca, ha = self.box[a]
        cb, hb = self.box[b]
        Ra, Rb = matrix_from_quat(Q[:, self.ids[a]]), matrix_from_quat(Q[:, self.ids[b]])  # (n, 3, 3), 열 = 링크 축
        wa = P[:, self.ids[a], None] + torch.einsum("nij,kj->nki", Ra, ca)  # (n, ka, 3)
        wb = P[:, self.ids[b], None] + torch.einsum("nij,kj->nki", Rb, cb)
        Ax, Bx = Ra.transpose(1, 2), Rb.transpose(1, 2)  # (n, 3, 3), 행 = 축
        cr = torch.cross(Ax[:, :, None, :].expand(-1, 3, 3, 3), Bx[:, None, :, :].expand(-1, 3, 3, 3), dim=-1).reshape(-1, 9, 3)
        ax = torch.cat((Ax, Bx, cr), dim=1)  # (n, 15, 3)
        nrm = ax.norm(dim=-1, keepdim=True)
        ok = nrm.squeeze(-1) > 1e-6
        ax = ax / nrm.clamp(min=1e-6)
        ra = torch.einsum("ki,nmi->nkm", ha, torch.einsum("nji,nmj->nmi", Ra, ax).abs())  # (n, ka, 15)
        rb = torch.einsum("ki,nmi->nkm", hb, torch.einsum("nji,nmj->nmi", Rb, ax).abs())
        dp = torch.einsum("nabj,nmj->nabm", wb[:, None, :, :] - wa[:, :, None, :], ax).abs()  # (n, ka, kb, 15)
        slack = ra[:, :, None, :] + rb[:, None, :, :] - dp
        slack = torch.where(ok[:, None, None, :], slack, torch.full_like(slack, 1e9))
        return slack.amin(dim=-1).flatten(1).amax(dim=1)  # 상자 쌍마다 가장 얕은 축 = 겹침 깊이, 그중 가장 깊은 것

    def depth_pairs(self):
        """(대, 쌍 6) 쌍마다 겹침 깊이 [m] (PAIRS 순서)."""
        d = self.robot.data
        P, Q = d.body_link_pos_w.torch, d.body_link_quat_w.torch
        return torch.stack([self._pair(a, b, P, Q) for a, b in PAIRS], dim=1)

    def depth(self):
        return self.depth_pairs().amax(dim=1)
