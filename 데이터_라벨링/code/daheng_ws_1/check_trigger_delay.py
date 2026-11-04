"""카메라가 TriggerDelay 를 지원하는지, 범위가 얼마인지 확인한다.

    python3 check_trigger_delay.py

라이다는 100 ms 에 걸쳐 360° 를 훑으므로, 카메라를 전부 동시에 쏘면
후방 카메라는 라이다와 최대 50 ms 어긋난다. nuScenes 처럼 카메라마다
시차를 두면 라이다 정합이 맞는다.

TriggerDelay 를 쓰면 GPIO 를 늘리지 않고도 이것이 가능하다.
MCU 는 지금처럼 GPIO4 하나로 4대에 같은 펄스를 보내고, 카메라가
각자 설정된 시간만큼 기다렸다 셔터를 연다.

    CAM1 TriggerDelay =     0 us   (전방)
    CAM2 TriggerDelay = 25000 us   (우측)
    CAM3 TriggerDelay = 50000 us   (후방)
    CAM4 TriggerDelay = 75000 us   (좌측)

확인할 것
  1. TriggerDelay 기능 자체가 있는가
  2. 최대값이 75000 us 이상인가
  3. 지연 + 노출 이 트리거 주기(100 ms) 를 넘지 않는가
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_VENDOR = os.path.join(HERE, "vendor")
if _VENDOR not in sys.path:
    sys.path.insert(0, _VENDOR)

import gxipy as gx  # noqa: E402

TRIGGER_PERIOD_US = 100000.0
NEED_MAX_US = 75000.0


def probe(rc, name, kind="float"):
    getters = {"float": rc.get_float_feature,
               "int": rc.get_int_feature,
               "enum": rc.get_enum_feature}
    try:
        f = getters[kind](name)
        info = {"ok": True}
        try:
            r = f.get_range()
            info["min"] = r.get("min")
            info["max"] = r.get("max")
        except Exception:
            pass
        try:
            v = f.get()
            info["value"] = v[1] if isinstance(v, (tuple, list)) else v
        except Exception:
            pass
        return info
    except Exception as e:
        return {"ok": False, "err": str(e)[:80]}


def main():
    dm = gx.DeviceManager()
    count, dev_list = dm.update_device_list()
    if count == 0:
        print("카메라를 찾지 못했습니다.")
        return 1

    sn = dev_list[0].get("sn")
    print(f"카메라 {sn} 으로 확인합니다.\n")

    cam = dm.open_device_by_sn(sn)
    try:
        rc = cam.get_remote_device_feature_control()

        # 트리거 모드로 두어야 TriggerDelay 가 노출되는 경우가 있다
        for name, val in (("TriggerMode", "On"),
                          ("TriggerSource", "Line0"),
                          ("TriggerSelector", "FrameStart")):
            try:
                rc.get_enum_feature(name).set(val)
            except Exception:
                pass

        print(f"{'기능':<22}{'상태':<10}{'최소':>12}{'최대':>14}{'현재':>12}")
        print("-" * 72)

        results = {}
        for name, kind in (("TriggerDelay", "float"),
                           ("ExposureTime", "float"),
                           ("TriggerFilterRaisingEdge", "float"),
                           ("AcquisitionFrameRate", "float")):
            r = probe(rc, name, kind)
            results[name] = r
            if r["ok"]:
                print(f"{name:<22}{'지원':<10}"
                      f"{_fmt(r.get('min')):>12}{_fmt(r.get('max')):>14}"
                      f"{_fmt(r.get('value')):>12}")
            else:
                print(f"{name:<22}{'없음':<10}{'—':>12}{'—':>14}{'—':>12}")

        print()
        td = results["TriggerDelay"]
        if not td["ok"]:
            print("[결과] TriggerDelay 를 지원하지 않습니다.")
            print("  → 시차 발사를 하려면 카메라마다 GPIO 를 분리해야 합니다.")
            print("     (GPIO4 → Q1 / GPIO8 → Q2 / GPIO9 → Q3 / GPIO10 → Q4)")
            return 2

        mx = td.get("max")
        if mx is None:
            print("[결과] 지원하지만 범위를 읽지 못했습니다. 직접 값을 넣어 시험하세요.")
            return 0

        print(f"[결과] TriggerDelay 지원. 최대 {mx:,.0f} us")
        if mx >= NEED_MAX_US:
            print(f"  → 75,000 us 까지 필요한데 충분합니다. 시차 발사 가능.")
            exp = results["ExposureTime"].get("value")
            if exp:
                total = 75000 + float(exp)
                print(f"  → 최대 지연 75,000 + 현재 노출 {float(exp):,.0f} "
                      f"= {total:,.0f} us")
                if total < TRIGGER_PERIOD_US:
                    print(f"     트리거 주기 100,000 us 안에 들어옵니다. OK")
                else:
                    print(f"     [경고] 100,000 us 를 넘습니다. "
                          f"노출을 {TRIGGER_PERIOD_US - 75000:,.0f} us "
                          f"이하로 줄여야 합니다.")
        else:
            usable = int(mx // 25000) + 1
            print(f"  → 75,000 us 에 못 미칩니다. "
                  f"카메라 {usable} 대까지만 시차를 줄 수 있습니다.")
        return 0

    finally:
        try:
            cam.close_device()
        except Exception:
            pass


def _fmt(v):
    if v is None:
        return "—"
    try:
        return f"{float(v):,.0f}"
    except (TypeError, ValueError):
        return str(v)


if __name__ == "__main__":
    sys.exit(main())
