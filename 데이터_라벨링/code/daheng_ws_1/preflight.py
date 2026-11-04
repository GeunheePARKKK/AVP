"""우분투 환경 점검 — collect.py 를 처음 돌리기 전에 한 번 실행할 것.

    python preflight.py

USB3 카메라 4대 + ESP32 시리얼 조합은 리눅스에서 커널/권한 설정을 몇 개
건드려야 한다. 안 건드리면 "카메라는 열리는데 프레임이 안 온다"거나
"1~2대까지는 되는데 4대째에서 실패한다" 같은 형태로 나타나서
원인을 찾기 어렵다. 여기서 미리 걸러낸다.
"""

import os
import subprocess
import sys

OK   = "  [OK]   "
WARN = "  [주의] "
FAIL = "  [실패] "

# 2048x1200 Bayer8 = 2.46 MB. 4대 x 버퍼 40개면 약 400 MB 가 필요하다.
NEED_USBFS_MB = 1000


def check_platform():
    print("1. 플랫폼")
    if not sys.platform.startswith("linux"):
        print(WARN + f"{sys.platform} — 이 스크립트는 리눅스 전용입니다.")
        return False
    try:
        with open("/etc/os-release") as f:
            name = [l for l in f if l.startswith("PRETTY_NAME")]
        print(OK + (name[0].split("=")[1].strip().strip('"') if name else "linux"))
    except OSError:
        print(OK + "linux")
    return True


def check_sdk():
    print("\n2. Galaxy SDK (libgxiapi.so)")
    paths = ["/usr/lib/libgxiapi.so", "/usr/lib/x86_64-linux-gnu/libgxiapi.so",
             "/usr/local/lib/libgxiapi.so"]
    found = [p for p in paths if os.path.exists(p)]
    if found:
        print(OK + found[0])
        if not os.path.exists("/usr/lib/libgxiapi.so"):
            print(WARN + "gxipy 는 /usr/lib/libgxiapi.so 를 고정 경로로 찾습니다.")
            print("         sudo ln -s %s /usr/lib/libgxiapi.so" % found[0])
            return False
        return True
    print(FAIL + "찾지 못했습니다.")
    print("         Daheng 리눅스 SDK 를 설치하세요 (Galaxy_Linux-x86_Gige-U3_*.tar.gz)")
    print("         압축 해제 후 sudo ./Galaxy_camera.run")
    return False


def check_gxipy():
    print("\n3. gxipy import")
    here = os.path.dirname(os.path.abspath(__file__))
    vendor = os.path.join(here, "vendor")
    if vendor not in sys.path:
        sys.path.insert(0, vendor)
    try:
        import gxipy as gx  # noqa: F401
        print(OK + "import 성공")
        return True
    except Exception as e:
        print(FAIL + str(e))
        return False


def check_usbfs():
    print("\n4. USB 메모리 한도 (usbfs_memory_mb)  ★ 4대 동시에는 필수")
    path = "/sys/module/usbcore/parameters/usbfs_memory_mb"
    try:
        with open(path) as f:
            cur = int(f.read().strip())
    except (OSError, ValueError):
        print(WARN + "값을 읽지 못했습니다.")
        return False

    if cur >= NEED_USBFS_MB:
        print(OK + f"{cur} MB")
        return True

    print(FAIL + f"{cur} MB — 4대 동시 스트리밍에는 부족합니다.")
    print("         기본값 16 MB 로는 카메라가 열려도 프레임이 오지 않거나")
    print("         3~4대째에서 실패합니다.")
    print()
    print("         임시 적용 (재부팅하면 사라짐):")
    print(f"           sudo sh -c 'echo {NEED_USBFS_MB} > {path}'")
    print()
    print("         영구 적용:")
    print("           sudo nano /etc/default/grub")
    print('           GRUB_CMDLINE_LINUX_DEFAULT="quiet splash '
          f'usbcore.usbfs_memory_mb={NEED_USBFS_MB}"')
    print("           sudo update-grub && sudo reboot")
    return False


def check_serial():
    print("\n5. 시리얼 포트")
    try:
        import serial.tools.list_ports
    except ImportError:
        print(FAIL + "pyserial 이 없습니다 →  pip install pyserial")
        return False

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print(FAIL + "포트가 없습니다. ESP32 가 연결되어 있나요?")
        print("         ls /dev/ttyACM* /dev/ttyUSB*")
        return False

    esp = None
    for p in ports:
        mark = ""
        if (p.vid or 0) == 0x303A:
            mark, esp = "  ← ESP32 (Espressif)", p
        print(f"         {p.device:<16} {p.description}{mark}")

    target = esp.device if esp else ports[0].device

    ok = True
    if not os.access(target, os.R_OK | os.W_OK):
        print(FAIL + f"{target} 에 접근 권한이 없습니다.")
        print("         sudo usermod -aG dialout $USER")
        print("         (로그아웃 후 재로그인해야 적용됩니다)")
        ok = False
    else:
        print(OK + f"{target} 접근 가능")

    if _service_running("ModemManager"):
        print(WARN + "ModemManager 가 실행 중입니다. 포트를 가로챌 수 있습니다.")
        print("         sudo systemctl stop ModemManager")
        print("         sudo systemctl disable ModemManager")
    return ok


def _service_running(name):
    try:
        r = subprocess.run(["systemctl", "is-active", name],
                           capture_output=True, text=True, timeout=3)
        return r.stdout.strip() == "active"
    except Exception:
        return False


def check_gui():
    print("\n6. 프리뷰 (cv2.imshow)")
    try:
        import cv2
    except ImportError:
        print(FAIL + "opencv 가 없습니다 →  pip install opencv-python")
        return False

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print(WARN + "DISPLAY 가 없습니다 (SSH 접속 등). 프리뷰가 안 뜹니다.")
        print("         collect.py --no-preview 로 실행하세요.")
        return True

    if "headless" in getattr(cv2, "__file__", ""):
        print(WARN + "opencv-python-headless 가 설치된 것 같습니다.")
        print("         pip uninstall opencv-python-headless")
        print("         pip install opencv-python")
        return True

    try:
        cv2.namedWindow("preflight", cv2.WINDOW_AUTOSIZE)
        cv2.destroyWindow("preflight")
        cv2.waitKey(1)
        print(OK + f"창 생성 가능 (opencv {cv2.__version__})")
    except Exception as e:
        print(WARN + f"창을 열 수 없습니다: {e}")
        print("         collect.py --no-preview 로 실행하세요.")
    return True


def check_disk():
    print("\n7. 저장 공간")
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        st = os.statvfs(here)
        free_gb = st.f_bavail * st.f_frsize / 1e9
    except OSError:
        print(WARN + "확인 실패")
        return True

    # 20초 세션 하나 = 4대 x 200장 x 2.46 MB ≒ 2 GB
    print(f"         여유 {free_gb:.1f} GB  (20초 세션 하나당 약 2 GB)")
    print(f"         → 약 {int(free_gb / 2)} 세션 분량")
    if free_gb < 10:
        print(FAIL + "10 GB 미만입니다.")
        return False
    print(OK + "충분")
    return True


def main():
    print("=" * 60)
    print(" 우분투 환경 점검")
    print("=" * 60)

    results = [
        check_platform(),
        check_sdk(),
        check_gxipy(),
        check_usbfs(),
        check_serial(),
        check_gui(),
        check_disk(),
    ]

    print("\n" + "=" * 60)
    if all(results):
        print(" 전부 통과. python list_cams.py 로 넘어가세요.")
        return 0
    print(" 위 [실패] 항목을 해결한 뒤 다시 실행하세요.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
