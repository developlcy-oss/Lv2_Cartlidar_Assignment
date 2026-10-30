"""팀 저장소에서 외부 월드 파일 없이 실행 가능한 등록부 안전 회귀 시험."""
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from perception_survey.core import Survey,BOX_CENTERS

def filled():
    s=Survey()
    for frame in range(2):
        for box,(x,y) in enumerate(BOX_CENTERS):
            angle=math.radians((box-3)*8)
            for side in (-1,1):
                tid=2*box+(side+1)//2
                assert s.observe({'x':x+side*.04*math.cos(angle),'y':y+side*.04*math.sin(angle),
                    'id':tid,'color':'red' if tid%2 else 'green','shape':'cylinder','pose':'standing',
                    'camera':'arm','confidence':.95},10.+frame)
    return s

def test_complete_fixed_twelve_slots():
    r=filled().snapshot()
    assert r['complete'] and len(r['slots'])==12
    assert sorted(t['id'] for t in r['slots'])==list(range(12))

def test_atomic_finish_freezes(tmp_path):
    s=filled();p=tmp_path/'puck_survey.json';r=s.finish(p)
    import json
    assert json.loads(p.read_text())==r and s.frozen

def test_incomplete_does_not_replace_valid_file(tmp_path):
    p=tmp_path/'puck_survey.json';p.write_text('previous')
    import pytest
    with pytest.raises(ValueError):Survey().finish(p)
    assert p.read_text()=='previous'

def test_conflict_is_not_hidden_by_majority():
    s=filled();t=s.snapshot()['slots'][0]
    for k in range(3):
        assert s.observe({'x':t['x_mm']/1000,'y':t['y_mm']/1000,'id':t['id'],'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':1.},20.+k)
    assert not s.snapshot()['complete']

def test_duplicates_not_independent_observations():
    s=Survey();x,y=BOX_CENTERS[0];o={'x':x-.04,'y':y,'id':0,'color':'green','shape':'cylinder','pose':'standing','camera':'arm','confidence':1.}
    assert s.observe(o,10.) and not s.observe(o,10.)
    assert s.snapshot()['slots'][0]['n_obs']==1

def test_reset_has_new_survey_id():
    s=filled();old=s.snapshot()['survey_id'];s.reset()
    assert s.snapshot()['survey_id']!=old and not s.snapshot()['complete']

def test_merge_gate_cannot_bridge_neighbor_slots():
    import pytest
    with pytest.raises(ValueError):Survey(gate_m=.04)

def test_invalid_yaw_cannot_partially_modify_state():
    s=Survey();x,y=BOX_CENTERS[0]
    assert not s.observe({'x':x-.04,'y':y,'id':0,'camera':'arm','confidence':1.,'yaw_deg':'bad'},10.)
    assert all(t['n_obs']==0 for t in s.snapshot()['slots'])
