# rs_get_frame.py
import time
import cv2
from realsense_D456 import RSMux

def main():
    mux = RSMux(host="127.0.0.1", enable_loopback=True)
    mux.start()
    print("[frames] role:", "MASTER" if mux.is_master else "CLIENT", mux.request_info())

    time.sleep(0.2)  # ısınma

    last_seq = None
    t0 = time.monotonic(); n = 0

    while True:
        frame_info = mux.get_rgb_frame(timeout_ms=500, decode=True)
        if not frame_info:
            print("NO FRAME (timeout)")
            continue

        if frame_info.get("frame") is None:
            color_bytes = frame_info.get("color_bytes") or b""
            print(
                f"NO FRAME (fmt={frame_info.get('fmt')} len={len(color_bytes)} shape={frame_info.get('color_shape')})"
            )
            continue

        img = frame_info["frame"]
        cv2.imshow("color", img)
        # 1 ms bekle (pencereyi canlı tutar)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

        n += 1
        if time.monotonic() - t0 >= 1.0:
            print(f"[frames] seq={frame_info['seq']} ts={frame_info['timestamp']:.5f}")
            t0 = time.monotonic(); n = 0

    mux.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
