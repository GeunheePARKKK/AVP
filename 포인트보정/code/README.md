# code

| 파일 | 역할 |
|---|---|
| `vlp16.py` | VLP-16 pcap 디코더. `시간동기화/code/vlp16.py` 와 같은 파일이다 (이 폴더만 클론해서 쓸 수 있도록 복사) |
| `vlp16_dataset.py` | pcap → KISS-ICP 데이터셋. 회전 단위로 끊고 점별 시각을 0~1 로 정규화 |
| `run_kiss_icp.py` | KISS-ICP 실행 → `poses_kitti.txt`, `poses_tum.txt`, `summary.txt` |
| `export_scans.py` | pcap → 스캔별 `(x,y,z,t)` npy 또는 KITTI `.bin` |
| `analyze_deskew.py` | 스캔 안 움직임·지도 정합 오차 측정, 보정식이 KISS-ICP 내부 보정과 같은지 검산 |
| `plot_results.py` | 궤적 · 스캔별 오차 · 보정 전후 그림 (SVG + PNG) |
| `deskew_to_shutter.py` | 카메라 셔터 시각 기준 보정 (Step 3 · 4 · 5) → 카메라별 포인트클라우드 |

## 필요한 것

- Python 3.10+, numpy
- `kiss-icp` (`run_kiss_icp.py` 만) — 1.3.0 에서 검증

## KISS-ICP 데이터셋 규약

`vlp16_dataset.py` 가 맞추는 규약은 두 개뿐이다.

```python
__len__()        # 스캔 개수
__getitem__(i)   # (points (N,3) float64, timestamps (N,) float64 0~1)
```

두 번째 값이 deskew 의 전부다. 이걸 넘기지 않으면 `config.data.deskew` 를 켜도 보정이 일어나지 않는다.

## 검산

```bash
python3 vlp16.py ../data/session_013/lidar.pcap    # 디코더 자체 리포트
python3 -c "
import sys; sys.path.insert(0,'.')
from vlp16_dataset import VLP16PcapDataset
ds = VLP16PcapDataset('../data/session_013/lidar.pcap')
p, t = ds[0]
print(len(ds), p.shape, t.min(), t.max())"
```
