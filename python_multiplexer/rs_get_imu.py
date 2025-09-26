# rs_get_imu.py
import time
from realsense_D456 import RSMux

def main():
    mux = RSMux(host="127.0.0.1")
    mux.start()
    info = mux.request_info()
    print("[imu] role:", "MASTER" if mux.is_master else "CLIENT", info)

    time.sleep(0.2)  # SUB ısınma

    last_seq = None; n = 0; t0 = time.monotonic()
    while True:
        imu = mux.get_imu(timeout_ms=200)
        if not imu:
            continue
        now_ns = time.monotonic_ns()
        lat_ms = (now_ns - imu["mono_ns"]) / 1e6
        if last_seq is not None and imu["seq"] != ((last_seq + 1) & 0xFFFFFFFF):
            pass
        last_seq = imu["seq"]
        n += 1
        if time.monotonic() - t0 >= 1.0:
            rate = n / (time.monotonic() - t0)
            print(f"[imu] seq={imu['seq']} lat={lat_ms:.2f}ms rate={rate:.1f}Hz gyro={imu['gyro']} accel={imu['accel']}")
            t0 = time.monotonic(); n = 0

if __name__ == "__main__":
    main()
