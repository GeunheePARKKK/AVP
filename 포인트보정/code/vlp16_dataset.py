"""VLP-16 pcap 을 KISS-ICP 가 읽는 형태로 내놓는 데이터셋.

KISS-ICP 의 데이터셋 규약은 두 개뿐이다.

    __len__()        스캔(회전) 개수
    __getitem__(i)   (points, timestamps)
                     points     (N,3) float64, 라이다 좌표계
                     timestamps (N,)  float64, 스캔 안에서 0~1 로 정규화

두 번째 값이 deskew 의 전부다. KISS-ICP 는 이 값으로 각 점이 스캔의 어느
시점에 찍혔는지 알고, 추정한 속도로 점을 스캔 시작(또는 중간) 시각으로
되돌린다. 값이 없으면 deskew 는 그냥 건너뛰어진다.

VLP-16 은 점별 시각을 패킷에 직접 담지 않는다. 대신 발사 순서가 고정이라
(시퀀스 55.296 us, 채널 2.304 us) 패킷 타임스탬프에서 정확히 계산된다.
그 계산은 vlp16.decode() 가 이미 해서 Decoded.time 에 넣어둔다.
"""

import numpy as np

import vlp16


class VLP16PcapDataset:
    """pcap 하나를 회전 단위로 끊어서 내놓는다.

    Parameters
    ----------
    pcap_path : str
        VLP-16 데이터 포트(2368) 패킷이 담긴 pcap.
    min_points : int
        이보다 점이 적은 회전은 버린다. pcap 의 첫 회전과 마지막 회전은
        녹화 시작·종료가 회전 중간에 걸려서 반 토막인 경우가 많고, 그걸
        정상 스캔으로 넘기면 ICP 가 첫 프레임부터 흔들린다.
    """

    def __init__(self, pcap_path, min_points=5000):
        self.pcap_path = str(pcap_path)
        d = vlp16.decode(self.pcap_path)

        self._xyz = d.xyz.astype(np.float64)
        self._time = d.time.astype(np.float64)
        self.intensity = d.intensity
        self.ring = d.ring
        self.model = d.model
        self.return_mode = d.return_mode

        self._spans = [(a, b) for a, b in d.revolutions()
                       if b - a >= min_points]

        # 각 스캔의 절대 시각(초). 패킷 타임스탬프는 '정시 이후 us' 라서
        # 한 시간마다 0 으로 돌아간다. 20 초 녹화가 그 경계를 넘었다면
        # 이어 붙여야 단조증가한다.
        t0 = np.array([d.packet_ts_us[a] for a, _ in self._spans]) / 1e6
        wrap = np.concatenate(([0], np.cumsum(np.diff(t0) < -1800.0))) * 3600.0
        self.scan_times = t0 + wrap

        self.sequence_id = "session_013"

    def __len__(self):
        return len(self._spans)

    def __getitem__(self, i):
        a, b = self._spans[i]
        points = self._xyz[a:b]
        t = self._time[a:b]
        span = t[-1] - t[0]
        if span <= 0:
            timestamps = np.zeros(len(t))
        else:
            timestamps = (t - t[0]) / span
        return points, timestamps

    def get_frames_timestamps(self):
        """KISS-ICP 가 궤적을 TUM 형식으로 저장할 때 쓰는 스캔별 시각."""
        return self.scan_times
