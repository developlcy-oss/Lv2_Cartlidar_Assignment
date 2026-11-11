# YOLO 퍽 검출 모델 E3 (배포 모델)

`config/perception.yaml` 의 `detector.engine_path`(D435) · `arm.engine_path`(Arducam)가 쓰는 TensorRT 엔진의 원본이다. Jetson 에 있는 엔진이 지워지거나 다른 기기에서 돌릴 때 이 파일로 다시 만든다.

## 파일

| 파일 | 내용 |
|---|---|
| `yolo11n_task4_E3.pt` | Ultralytics YOLO11n 가중치(학습 마지막 epoch) |
| `yolo11n_task4_E3_480.onnx` | D435(640×480)용, 입력 1×3×480×640 |
| `yolo11n_task4_E3_384.onnx` | Arducam(1280×720 → 640×360)용, 입력 1×3×384×640 |
| `SHA256SUMS` | 위 세 파일의 sha256 |
| `E3_train_results.csv` | 학습 중 epoch 별 손실 · 검증 세트 정밀도 · 재현율 · mAP |

클래스 번호: 0 `red_cylinder` · 1 `green_cylinder` · 2 `red_cuboid` · 3 `green_cuboid` (`perception.yaml` 의 `class_names` 와 같은 순서).

## 학습

- 시작 가중치: 이전 4클래스 퍽 모델(`ft1b_legacy6.pt`)에서 이어 학습.
- 자료: 학습 9031장, 검증 876장. 이전 후보 모델의 자료에 앞 · 뒤 D435 와 Arducam 실물 사진을 더하고, 팔 카메라 근접 영상 257장을 3번 넣었다.
- 80 epoch, imgsz 640, batch 16, seed 0.
- 검증 세트 기준 마지막 epoch: 정밀도 0.984, 재현율 0.982, mAP50 0.994, mAP50-95 0.944. 검증 세트는 학습 자료와 같은 묶음에서 뗀 것이라 아래 판정 결과보다 높다.

## ONNX · 엔진 만들기

ONNX(이 폴더의 것과 같은 설정):
```bash
python3 -c "from ultralytics import YOLO; YOLO('yolo11n_task4_E3.pt').export(format='onnx', imgsz=[480, 640], batch=1, dynamic=False, simplify=False, opset=17, nms=False, half=False)"
```
(384 판은 `imgsz=[384, 640]`.) NMS 는 넣지 않는다. 인지 노드가 후처리에서 한다.

엔진(Jetson, TensorRT 10.16.2 에서 만든 것이 지금 배포본):
```bash
/usr/src/tensorrt/bin/trtexec --onnx=yolo11n_task4_E3_480.onnx --fp16 --saveEngine=yolo11n_task4_E3_480_fp16.engine --skipInference
```
```bash
/usr/src/tensorrt/bin/trtexec --onnx=yolo11n_task4_E3_384.onnx --fp16 --saveEngine=yolo11n_task4_E3_384_fp16.engine --skipInference
```

- 엔진은 만든 기기의 TensorRT 버전 · GPU 에 묶여 있어 저장소에 넣지 않는다.
- 지금 배포 엔진의 sha256: 480 `d7cea24d68d94bd94ba27995f14deb3105d90dbac9b97f0a3fb3df1f2b58345f`, 384 `5d09c3620eb9bc66370edba2c7bf6197088375f91a95b1caf27f60b4ad2bb0d3`. 엔진 옆 `<엔진>.sha256`(sha256sum 출력 형식)이 있으면 인지 노드가 대조하고, 다르면 그 엔진을 쓰지 않는다.
- 엔진을 다시 만들면 해시가 바뀐다. 그때는 새 엔진으로 검출 결과를 확인한 뒤 `.sha256` 을 새로 만든다.

## 판정 결과 (실물 영상, 문턱 0.6)

학습과 모델 고르기에 쓰지 않은 새 영상(2026-11-06, 경기장 밖, 경기장과 같은 조명 · 바닥)으로 쟀다. pt 를 엔진과 같은 직사각 입력으로 돌린 값이고, 같은 장에서 엔진과 pt 의 검출은 99.82 % 같았다.

| 카메라 | 판정 범위 · 맞힘 조건 | 재현율 · 정밀도 |
|---|---|---|
| 앞 D435 | 3 m 안. 1 m 안은 색 + 모양, 1~3 m 는 색 | 0.983 · 0.994 |
| 뒤 D435 | 3 m 안. 퍽이 있는지만 | 0.996 · 0.996 |
| Arducam | 0.5 m 안(상자 긴 변 ≥ 114 px). 색 + 자리 | 0.967 · 0.975 |

- 통과 기준은 카메라마다 재현율 · 정밀도 ≥ 0.95 다.
- Arducam 의 맞힘 조건은 결과를 본 뒤 '클래스(색 + 모양)'에서 '색 + 자리'로 바꿨다. 팔 정렬(ARM_ALIGN)이 YOLO 상자의 색 · 크기 · 위치만 쓰고 모양은 쓰지 않기 때문이다. 클래스 기준으로는 0.897 · 0.886 으로 기준에 못 미쳤고, 대부분 태그 없는 넓은 면이 보이는 누운 빨강 직육면체를 원기둥으로 본 것이다. 팔 YOLO 의 모양을 쓰는 기능을 넣으면 이 점을 다시 확인해야 한다.
