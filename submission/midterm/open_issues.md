# 미완료 Issue (중간 제출)

기준: 2026-11-09 GitHub 열린 이슈. 라벨 = 파트 (`mission` · `planner` · `integration` · `lead` = 팀장 결정 필요).
발제: Issue 마다 구현자와 검토자를 정한다 → 담당 **없음** 인 이슈는 지정 필요.

| Issue | 파트 | 담당 | 열린 날 | 제목 |
|---|---|---|---|---|
| #165 |  | WindForce08 | 2026-11-09 | [팔 펌웨어] 실기 확인 M3~M6 (손 확인 · move · 슬라이더 · 퍽 쥐기) |
| #161 |  | developlcy-oss | 2026-11-09 | [manipulator] joint1 영점이 약 180° 어긋난 것으로 보임 (정면이 약 ±3.2 rad) |
| #160 | mission | **없음** | 2026-11-09 | [mission/drive] 놓기 전 장소 (L1 ~ L3) 접근이 퍽을 찾다가 실패함 — 빈 상자에서 COARSE · FINE 정렬 "물체 놓침" |
| #157 | mission | **없음** | 2026-11-09 | [drive/perception] 박스 앞 FINE 정렬이 끝나면 TFmini 비상 정지가 다시 걸려 로봇이 못 움직임 (회전 · 후진 · Nav2 모두) |
| #154 | integration | **없음** | 2026-11-09 | [manipulator] READY · BOX_VIEW 자세에서 팔 카메라가 퍽을 못 봄 (FINE 정렬 · 퍽 조사) |
| #151 | integration | **없음** | 2026-11-09 | [localization] Gazebo 에서 loc_monitor LOST:residual 이 회전 중 잦고, 다시 넣은 자세 yaw 가 틀어져 미션 FAILED |
| #145 | integration, lead | WindForce08, kanichong, developlcy-oss | 2026-11-09 | [sim] 모의 팔 카메라 화각을 실물 보정값(가로 약 73°)으로 맞추기 |
| #144 | integration, lead | WindForce08, developlcy-oss | 2026-11-08 | [manipulator] 팔 시작 정리: 켜질 때 자동 STOW · 관절 한계 처짐 여유 / READY 충돌 (젯슨 실물) |
| #141 | integration | developlcy-oss | 2026-11-07 | [manipulator/policy] 제안: 집기 넘어뜨림 · 쥐는 깊이 줄이기 (fit1 시험에서 본 것) |
| #114 | planner, lead | WindForce08 | 2026-11-04 | [nav2] 목표 허용 오차 (0.025 < AMCL 오차) · 빈 박스 장애물 층 검토 |
| #113 | integration | WindForce08 | 2026-11-04 | [drive/policy] 주행 정책 관측 · 행동 형식이 학습과 배포에서 다름 |
| #112 | integration | kanichong, developlcy-oss | 2026-11-04 | [sim/test] 실기 URDF ↔ Isaac 모델 비교 시험 (바퀴 위치 · 한계 · FK) |
| #109 | integration | WindForce08 | 2026-11-04 | [hw/측정] 무작위화 중심값 실측: 퍽 마찰 · 팔 처짐 · 왕복 지연 · 퍽 질량 |
| #108 | integration | Onbeom, WindForce08 | 2026-11-04 | [manipulator/bench] 타겟체크자세 (target_look) → toppick_tilt 실물 확인 |
| #107 | integration | WindForce08 | 2026-11-04 | [manipulator/bench] 팔레트 처짐 보정 스트리밍 (sag.*) 실물 벤치 확인 |
| #88 | integration | Onbeom, kanichong | 2026-11-03 | [perception/perf] Jetson 영상 토픽 부하 측정 (tegrastats) — SDK 직접 열기 → 드라이버 토픽 전환 뒤 |
| #87 | integration | Onbeom, kanichong | 2026-11-03 | [perception/hw] TFmini 박스 거리 실측 보정 (mm 출력 · 보정표 · 렌즈 높이) + approach 거리 실측 |
| #78 | integration | developlcy-oss | 2026-11-03 | [manipulator/policy] TopPick 다시 학습: 모두 45° · 실제 박스 배치 (옆 퍽 접선 · gap 0.06 · 회전 뒤 상자 퍽) |
| #49 | planner, integration | WindForce08, kanichong, developlcy-oss | 2026-11-01 | [요청] 4륜 스키드 회전 보정(사각지대 · 회전 중심) 실측과 둘 계층 정하기 |
| #15 | integration | WindForce08, developlcy-oss | 2026-10-27 | [요청] opencr_hardware 바퀴 위치·간격을 URDF 1711과 맞춰 확인 |
| #14 | integration | Onbeom, WindForce08, kanichong | 2026-10-27 | [요청] 새 모델 1711 팔 카메라: 정렬 기준점·내려다보기 자세 |

## 경기 영향 순서 (제안)

| 순위 | Issue | 이유 |
|---|---|---|
| 1 | #161 · #165 · #139 | 팔 영점 · 펌웨어가 확인돼야 모든 실물 집기 시험이 의미 있음 |
| 2 | #157 · #160 | 박스 · 장소 앞 정렬이 막혀 집기 · 놓기 단계로 못 감 |
| 3 | #154 · #108 · #78 · #141 | 팔 카메라 시야와 집기 성공률 |
| 4 | #87 · #49 · #114 | TFmini 거리 · 회전 중심 · Nav2 허용 오차 (정확도) |
| 5 | #151 · #112 · #113 · #109 · #107 · #88 · #15 · #14 | 모의 · 측정 · 정리 |
