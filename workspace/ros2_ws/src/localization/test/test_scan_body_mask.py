import numpy as np

from localization.scan_body_mask import or_grow


def test_or_grow_handles_changing_scan_length():
    # 실물 라이다는 한 바퀴 점 수가 399 · 400 으로 바뀐다 (11-13 실물에서 길이 차이로 노드가 죽음)
    a = np.zeros(400, bool); a[[0, 399]] = True
    b = np.zeros(399, bool); b[5] = True
    m = or_grow(or_grow(None, a), b)
    assert len(m) == 400 and m[0] and m[5] and m[399]
    m2 = or_grow(or_grow(None, b), a)
    assert len(m2) == 400 and m2[0] and m2[5] and m2[399]
