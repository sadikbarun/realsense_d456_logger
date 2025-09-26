# rs_get_ir.py
import time
import cv2
from realsense_D456 import RSMux


def main():
    mux = RSMux(host="127.0.0.1", enable_loopback=True)
    mux.start()
    print("[ir] role:", "MASTER" if mux.is_master else "CLIENT", mux.request_info())

    time.sleep(0.2)

    t0 = time.monotonic()
    n = 0
    while True:
        ir = mux.get_ir_pair(timeout_ms=500, decode=True)
        if not ir or (ir.get("left") is None and ir.get("right") is None):
            print("NO IR")
            continue

        if ir.get("left") is not None:
            cv2.imshow("ir_left", ir["left"])
        if ir.get("right") is not None:
            cv2.imshow("ir_right", ir["right"])

        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

        n += 1
        if time.monotonic() - t0 >= 1.0:
            print(f"[ir] seq={ir['seq']} fmt={ir['fmt']}")
            t0 = time.monotonic(); n = 0

    mux.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
