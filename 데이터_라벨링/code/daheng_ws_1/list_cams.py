"""연결된 Daheng 카메라의 시리얼 번호를 나열한다.

    python list_cams.py

가장 먼저 실행할 것. 여기서 나온 sn 4개를 sync_cam.py 의 CAMERA_SNS 에 적고,
같은 순서로 Line1 케이블을 ESP32 GPIO 6 / 7 / 15 / 16 에 연결한다.

케이블에 CAM1~CAM4 라벨을 붙여 둘 것. 소프트웨어 순서와 물리 배선 순서가
어긋나면 코드는 그것을 절대 알아채지 못하고, offset 만 서로 뒤바뀐다.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_VENDOR = os.path.join(_HERE, "vendor")
if _VENDOR not in sys.path:
    sys.path.insert(0, _VENDOR)

import gxipy as gx  # noqa: E402


def main():
    dm = gx.DeviceManager()
    count, dev_list = dm.update_device_list()

    if count == 0:
        print("카메라를 찾지 못했습니다.")
        print("  - USB3 케이블 확인")
        print("  - GalaxyView 가 카메라를 잡고 있으면 종료할 것")
        return 1

    print(f"{count} 대 발견\n")
    print(f"{'idx':<5} {'sn':<20} {'model':<24} {'user_id'}")
    print("-" * 70)
    for i, d in enumerate(dev_list):
        print(f"{i + 1:<5} {d.get('sn', ''):<20} "
              f"{d.get('model_name', ''):<24} {d.get('user_id', '')}")

    print("\n아래를 sync_cam.py 의 CAMERA_SNS 에 붙여넣으세요:\n")
    print("CAMERA_SNS = [")
    for d in dev_list:
        print(f'    "{d.get("sn", "")}",')
    print("]")
    print("\n※ 순서가 CAM1~CAM4 입니다. Line1 배선 순서와 반드시 맞출 것.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
