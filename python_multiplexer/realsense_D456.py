
# ZMQ tabanlı RealSense D456 MUX
# - Auto MASTER/CLIENT (port-yoklamalı)
# - FRAMES: PUB (latest-wins) [opsiyonel JPEG]
# - IMU: PUB (tek paket: gyro + eşleşmiş/interpole accel, opsiyonel LPF)
# - CTRL: REP (durum/ayar)

import time
import json
import threading
from typing import Optional, List, Dict, Any, Tuple
from collections import deque

import numpy as np
import pyrealsense2 as rs
import zmq

try:
    import cv2  # JPEG için gerekli
except Exception:
    cv2 = None


# ===========================
# Konfig
# ===========================
HOST = "127.0.0.1"        # CLIENT için bağlanılacak adres
PORT_FRAMES = 5756
PORT_IMU    = 5757
PORT_CTRL   = 5758

# FRAMES (RGB+IR) ayarları
COLOR_W, COLOR_H, COLOR_FPS = 1280, 720, 30
IR_W, IR_H, IR_FPS          = 1280, 720, 30
ENABLE_JPEG   = True                 # JPEG ile taşı (ağ/IPC için pratik)
JPEG_QUALITY  = 95                  # 80–95 arası iyi

# FRAMES ZMQ
FRAMES_SNDHWM = 2                   # latest-wins için düşük tut

# IMU eşleme/filtre
MAX_DT_S      = 0.005               # gyro ts ile accel ts arası maks fark (s)
USE_INTERP    = True                # lineer interpolasyon
USE_LPF       = True                # interpolasyon sonrası 200 Hz tek-kutuplu LPF
LPF_FC_HZ     = 70.0                # cut-off (UAV için 60–80 iyi)
IMU_RATE_HZ   = 200.0               # gyro ritminde paket çıkar
# IMU ZMQ
IMU_SNDHWM    = 4096                # IMU conflate=off, HWM yüksek kalsın

# CTRL dönen bilgi
VERSION = "1.0-uav"

# ===========================
# ZMQ endpoint'leri
# ===========================
EP_FRAMES_BIND = f"tcp://0.0.0.0:{PORT_FRAMES}"
EP_IMU_BIND    = f"tcp://0.0.0.0:{PORT_IMU}"
EP_CTRL_BIND   = f"tcp://0.0.0.0:{PORT_CTRL}"

EP_FRAMES_CONN = f"tcp://{HOST}:{PORT_FRAMES}"
EP_IMU_CONN    = f"tcp://{HOST}:{PORT_IMU}"
EP_CTRL_CONN   = f"tcp://{HOST}:{PORT_CTRL}"


# ===========================
# Yardımcılar
# ===========================
def _try_bind(sock, ep: str) -> bool:
    try:
        sock.bind(ep)
        return True
    except Exception:
        return False


class OnePoleLPF:
    """Tek kutuplu IIR alçak geçiren filtre (v = v + α (x - v))"""
    def __init__(self, fc_hz: float, rate_hz: float):
        dt = 1.0 / rate_hz
        tau = 1.0 / (2.0 * np.pi * fc_hz)
        self.alpha = dt / (tau + dt)
        self.y = np.zeros(3, dtype=np.float32)
        self.init = False

    def filt(self, x: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x = np.array(x, dtype=np.float32)
        if not self.init:
            self.y = x.copy()
            self.init = True
        self.y += self.alpha * (x - self.y)
        return (float(self.y[0]), float(self.y[1]), float(self.y[2]))


# ===========================
# Ana sınıf
# ===========================
class RSMux:
    """
    ZMQ tabanlı, auto MASTER/CLIENT RealSense MUX.

    MASTER:
      - RS video pipeline: COLOR + IR1 + IR2
      - RS IMU pipeline: gyro (200 Hz), accel (400 Hz)
      - FRAMES PUB: latest-wins (yüksek HWM düşük tutulur)
      - IMU PUB: tek paket: (mono_ns, ts, gyro_xyz, accel_xyz)
        -> accel; gyro timestampına lineer interpolasyonla eşlenir; opsiyonel LPF
      - CTRL REP: durum/ayar döner

    CLIENT:
      - FRAMES SUB, IMU SUB, CTRL REQ (cihaza dokunmaz)
    """
    def __init__(self, host: str = HOST, enable_loopback: bool = True, debug_show_frames: bool = False):
        self.host = host
        self.enable_loopback = enable_loopback
        self.debug_show_frames = debug_show_frames
        self.ctx = zmq.Context.instance()

        # role
        self.is_master = False
        self._stop = threading.Event()

        # PUB/REP (master) soketleri
        self.pub_frames = self.ctx.socket(zmq.PUB)
        self.pub_imu    = self.ctx.socket(zmq.PUB)
        self.rep_ctrl   = self.ctx.socket(zmq.REP)

        # frames HWM
        self.pub_frames.setsockopt(zmq.SNDHWM, FRAMES_SNDHWM)

        # imu HWM
        self.pub_imu.setsockopt(zmq.SNDHWM, IMU_SNDHWM)

        ok_f = _try_bind(self.pub_frames, EP_FRAMES_BIND)
        ok_i = _try_bind(self.pub_imu, EP_IMU_BIND)
        ok_c = _try_bind(self.rep_ctrl, EP_CTRL_BIND)

        if ok_f and ok_i and ok_c:
            self.is_master = True
            # LOOPBACK: MASTER iken kendi yayınını dinle (tek süreç görüntü/imu tüketmek için)
            if self.enable_loopback:
                self.sub_frames = self.ctx.socket(zmq.SUB)
                self.sub_imu    = self.ctx.socket(zmq.SUB)
                self.req_ctrl   = self.ctx.socket(zmq.REQ)  # ctrl’i de kendi üstünden sorgulayalım

                self.sub_frames.setsockopt(zmq.RCVHWM, 2048)
                self.sub_imu.setsockopt(zmq.RCVHWM, 4096)

                # Sadece ilgili topic’ler
                self.sub_frames.setsockopt(zmq.SUBSCRIBE, b"F")
                self.sub_imu.setsockopt(zmq.SUBSCRIBE, b"I")

                # Kendi bind ettiğimiz portlara localhost üzerinden bağlan
                self.sub_frames.connect(EP_FRAMES_CONN)  # "tcp://127.0.0.1:5756"
                self.sub_imu.connect(EP_IMU_CONN)
                self.req_ctrl.connect(EP_CTRL_CONN)

                time.sleep(0.2)  # late-joiner ısınma
        else:
            # client mode
            self.pub_frames.close(0)
            self.pub_imu.close(0)
            self.rep_ctrl.close(0)

            self.sub_frames = self.ctx.socket(zmq.SUB)
            self.sub_imu    = self.ctx.socket(zmq.SUB)
            self.req_ctrl   = self.ctx.socket(zmq.REQ)

            self.sub_frames.setsockopt(zmq.RCVHWM, 2048)
            self.sub_imu.setsockopt(zmq.RCVHWM, 4096)

            self.sub_frames.setsockopt(zmq.SUBSCRIBE, b"F")
            self.sub_imu.setsockopt(zmq.SUBSCRIBE, b"I")

            self.sub_frames.connect(EP_FRAMES_CONN)
            self.sub_imu.connect(EP_IMU_CONN)
            self.req_ctrl.connect(EP_CTRL_CONN)

            time.sleep(0.2)

        # RS pipelines (master)
        self._video_pipe: Optional[rs.pipeline] = None
        self._imu_pipe: Optional[rs.pipeline] = None

        # Threads (master)
        self._threads: List[threading.Thread] = []

        # Frame seq
        self._seq_frame = 0

        # IMU pairer state
        self._accel_q: deque = deque(maxlen=4096)  # (ts, mono_ns, (ax,ay,az))
        self._gyro_q:  deque = deque(maxlen=1024)  # (ts, mono_ns, (gx,gy,gz))
        self._accel_lock = threading.Lock()
        self._gyro_lock  = threading.Lock()
        self._imu_seq = 0
        self._lpf = OnePoleLPF(LPF_FC_HZ, IMU_RATE_HZ) if USE_LPF else None
        self._frames_use_jpeg = bool(ENABLE_JPEG and cv2 is not None)
        self._warned_jpeg_fail = False

    # --------------- MASTER ---------------
    def start(self):
        if not self.is_master:
            return
        self._start_rs()
        t_frames = threading.Thread(target=self._video_loop, daemon=True)
        t_ctrl   = threading.Thread(target=self._ctrl_loop, daemon=True)
        t_accel  = threading.Thread(target=self._accel_thread, daemon=True)
        t_gyro   = threading.Thread(target=self._gyro_thread,  daemon=True)
        self._threads = [t_frames, t_ctrl, t_accel, t_gyro]
        for t in self._threads:
            t.start()

    def _start_rs(self):
        # Video pipeline
        self._video_pipe = rs.pipeline()
        vcfg = rs.config()
        vcfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
        vcfg.enable_stream(rs.stream.infrared, 1, IR_W, IR_H, rs.format.y8, IR_FPS)
        vcfg.enable_stream(rs.stream.infrared, 2, IR_W, IR_H, rs.format.y8, IR_FPS)
        self._video_pipe.start(vcfg)

        # IMU pipeline (callback -> kuyruklara)
        self._imu_pipe = rs.pipeline()
        icfg = rs.config()
        icfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
        icfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 400)
        self._imu_pipe.start(icfg, self._imu_cb)

    def _video_loop(self):
        while not self._stop.is_set():
            try:
                fs: rs.composite_frame = self._video_pipe.wait_for_frames(1000)
            except Exception:
                continue
            now_ns = time.monotonic_ns()
            ts_s = fs.get_timestamp() / 1000.0
            self._seq_frame = (self._seq_frame + 1) & 0xFFFFFFFF

            use_jpeg = self._frames_use_jpeg and cv2 is not None
            color_bytes = b""
            color_meta: Dict[str, Any] = {}
            ir_meta: Dict[str, Any] = {}

            c = fs.get_color_frame()
            color = None
            if c:
                color = np.asanyarray(c.get_data())
                if self.debug_show_frames and cv2 is not None:
                    cv2.imshow("RSMux-color", color)
                    cv2.waitKey(1)
                if use_jpeg:
                    ok, buf = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)])
                    if ok:
                        color_bytes = buf.tobytes()
                    else:
                        use_jpeg = False
                        self._frames_use_jpeg = False
                        if not self._warned_jpeg_fail:
                            print("[RSMux] WARN: JPEG encode failed, falling back to raw BGR.")
                            self._warned_jpeg_fail = True
                if not use_jpeg and color is not None:
                    color_bytes = color.tobytes()
                    color_meta["color_shape"] = list(color.shape)
                    color_meta["color_dtype"] = str(color.dtype)
            else:
                color_bytes = b""

            ir_chunks: List[bytes] = []

            for idx, win_name, key in [(1, "RSMux-irL", "irL"), (2, "RSMux-irR", "irR")]:
                ir_frame = fs.get_infrared_frame(idx)
                data = b""
                if ir_frame is not None:
                    ir_img = np.asanyarray(ir_frame.get_data())
                    if self.debug_show_frames and cv2 is not None:
                        cv2.imshow(win_name, ir_img)
                        cv2.waitKey(1)
                    if use_jpeg and cv2 is not None:
                        ok, buf = cv2.imencode(".jpg", ir_img)
                        if ok:
                            data = buf.tobytes()
                        else:
                            use_jpeg = False
                            self._frames_use_jpeg = False
                            if not self._warned_jpeg_fail:
                                print("[RSMux] WARN: IR JPEG encode failed, falling back to raw Y8.")
                                self._warned_jpeg_fail = True
                    if not use_jpeg:
                        data = ir_img.tobytes()
                ir_meta[f"{key}_shape"] = list(ir_img.shape)
                ir_meta[f"{key}_dtype"] = str(ir_img.dtype)
                ir_chunks.append(data)

            fmt = "jpeg" if use_jpeg else "bgr"
            ir_meta["ir_fmt"] = "jpeg" if use_jpeg else "y8"

            if not use_jpeg and color is not None and not color_meta:
                color_bytes = color.tobytes()
                color_meta["color_shape"] = list(color.shape)
                color_meta["color_dtype"] = str(color.dtype)

            self._frames_use_jpeg = use_jpeg

            header = {
                "topic": "frames",
                "seq": int(self._seq_frame),
                "sensor_ts": float(ts_s),
                "mono_ns": int(now_ns),
                "fmt": fmt,
            }
            header.update(color_meta)
            header.update(ir_meta)

            parts: List[bytes] = [b"F", json.dumps(header).encode("utf-8"), color_bytes]
            parts.extend(ir_chunks)

            assert parts[0] == b"F"
            self.pub_frames.send_multipart(parts)
            if self.debug_show_frames:
                print("[RSMux] frame sent", fmt, len(color_bytes))

    # ------ IMU callback: ham kuyruklara doldur ------
    def _imu_cb(self, frame: rs.frame):
        if not frame.is_motion_frame():
            return
        now_ns = time.monotonic_ns()
        ts_s = frame.get_timestamp() / 1000.0
        mf = frame.as_motion_frame()
        d = mf.get_motion_data()
        st = mf.get_profile().stream_type()
        if st == rs.stream.accel:
            with self._accel_lock:
                self._accel_q.append((ts_s, now_ns, (float(d.x), float(d.y), float(d.z))))
        elif st == rs.stream.gyro:
            with self._gyro_lock:
                self._gyro_q.append((ts_s, now_ns, (float(d.x), float(d.y), float(d.z))))

    # ------ IMU tüketici thread'leri ------
    def _accel_thread(self):
        # Sadece tamponu sıcak tutar; spesifik iş yapmıyoruz.
        while not self._stop.is_set():
            time.sleep(0.001)

    def _gyro_thread(self):
        while not self._stop.is_set():
            g = self._pop_gyro()
            if g is None:
                time.sleep(0.0005)
                continue
            ts_g, mono_g, (gx, gy, gz) = g

            a = self._match_accel(ts_g)
            if a is None:
                # Boş paket YOK; accel bulunamazsa atla
                continue
            ax, ay, az = a

            # opsiyonel LPF (200 Hz ritminde)
            if self._lpf is not None:
                ax, ay, az = self._lpf.filt((ax, ay, az))

            # tek IMU paketi
            self._imu_seq = (self._imu_seq + 1) & 0xFFFFFFFF
            msg = {
                "topic": "imu",
                "seq": int(self._imu_seq),
                "mono_ns": int(mono_g),
                "ts": float(ts_g),
                "gyro_x": float(gx), "gyro_y": float(gy), "gyro_z": float(gz),
                "accel_x": float(ax), "accel_y": float(ay), "accel_z": float(az),
            }
            self.pub_imu.send_multipart([b"I", json.dumps(msg).encode("utf-8")])

    def _pop_gyro(self) -> Optional[Tuple[float, int, Tuple[float,float,float]]]:
        with self._gyro_lock:
            if not self._gyro_q:
                return None
            return self._gyro_q.popleft()

    # ------ accel eşleme (nearest + opsiyonel lineer interpolasyon) ------
    def _match_accel(self, ts_g: float) -> Optional[Tuple[float, float, float]]:
        with self._accel_lock:
            if not self._accel_q:
                return None

            # en yakın indeksi ara (kısa kuyrukta lineer tarama hızlı)
            best_i, best_dt = None, 1e9
            for i, (ts_a, _, _) in enumerate(self._accel_q):
                dt = abs(ts_a - ts_g)
                if dt < best_dt:
                    best_dt, best_i = dt, i
                else:
                    if best_i is not None:
                        break

            if best_i is None or best_dt > MAX_DT_S:
                return None

            if not USE_INTERP:
                return self._accel_q[best_i][2]

            # lineer interpolasyon (ts0 <= ts_g <= ts1 ise)
            i0 = max(0, best_i - 1)
            i1 = min(len(self._accel_q) - 1, best_i + 1)
            ts0, _, a0 = self._accel_q[i0]
            ts1, _, a1 = self._accel_q[i1]

            if ts0 <= ts_g <= ts1 and ts1 != ts0:
                t = (ts_g - ts0) / (ts1 - ts0)
                ax = a0[0] + t * (a1[0] - a0[0])
                ay = a0[1] + t * (a1[1] - a0[1])
                az = a0[2] + t * (a1[2] - a0[2])
                return (ax, ay, az)
            # sınır durumu: nearest
            return self._accel_q[best_i][2]

    def _ctrl_loop(self):
        info = {
            "role": "MASTER",
            "version": VERSION,
            "frames": {"w": COLOR_W, "h": COLOR_H, "fps": COLOR_FPS, "jpeg": ENABLE_JPEG, "q": JPEG_QUALITY},
            "imu": {"max_dt_s": MAX_DT_S, "interp": USE_INTERP, "lpf": USE_LPF, "fc_hz": LPF_FC_HZ, "rate_hz": IMU_RATE_HZ},
            "ports": {"frames": PORT_FRAMES, "imu": PORT_IMU, "ctrl": PORT_CTRL},
        }
        while not self._stop.is_set():
            try:
                _ = self.rep_ctrl.recv(flags=zmq.NOBLOCK)
                self.rep_ctrl.send(json.dumps(info).encode("utf-8"))
            except zmq.Again:
                time.sleep(0.005)

    # --------------- CLIENT ---------------
    def request_info(self, timeout_ms: int = 300):
        if self.is_master:
            return {"role": "MASTER_LOCAL", "version": VERSION}
        req = self.req_ctrl
        req.send(b"info")
        poller = zmq.Poller(); poller.register(req, zmq.POLLIN)
        socks = dict(poller.poll(timeout_ms))
        if socks.get(req) == zmq.POLLIN:
            return json.loads(req.recv())
        return None

    def recv_frames(self, timeout_ms: int = 300):
        # MASTER olsa bile, loopback SUB açıksa dinleyebiliriz
        if not hasattr(self, "sub_frames") or self.sub_frames is None:
            return None

        poller = zmq.Poller(); poller.register(self.sub_frames, zmq.POLLIN)
        socks = dict(poller.poll(timeout_ms))
        if socks.get(self.sub_frames) != zmq.POLLIN:
            return None

        parts = self.sub_frames.recv_multipart()
        if len(parts) < 2 or parts[0] != b"F":
            print("[RSMux] recv_frames invalid parts", len(parts), parts[0] if parts else None)
            return None
        try:
            header = json.loads(parts[1].decode("utf-8"))
        except Exception as exc:
            print("[RSMux] recv_frames header decode error", exc)
            return None

        out = dict(header)
        if len(parts) >= 3:
            out["color_bytes"] = parts[2]
            if header.get("fmt") == "jpeg":
                out["color_jpg"] = parts[2]
        if len(parts) >= 4:
            out["irL_bytes"] = parts[3]
            if header.get("fmt") == "jpeg":
                out["irL_jpg"] = parts[3]
        if len(parts) >= 5:
            out["irR_bytes"] = parts[4]
            if header.get("fmt") == "jpeg":
                out["irR_jpg"] = parts[4]
        return out


    def recv_imu(self, timeout_ms: int = 100):
        if not hasattr(self, "sub_imu") or self.sub_imu is None:
            return None

        poller = zmq.Poller(); poller.register(self.sub_imu, zmq.POLLIN)
        socks = dict(poller.poll(timeout_ms))
        if socks.get(self.sub_imu) != zmq.POLLIN:
            return None

        parts = self.sub_imu.recv_multipart()
        if len(parts) < 2 or parts[0] != b"I":
            return None
        try:
            return json.loads(parts[1].decode("utf-8"))
        except Exception:
            return None

    # ------ Yüksek seviyeli convenience API ------
    def get_rgb_frame(self, timeout_ms: int = 300, decode: bool = True):
        """Tek RGB kare + zaman bilgisi döndürür."""
        pkt = self.recv_frames(timeout_ms=timeout_ms)
        if not pkt:
            return None

        frame_payload: Optional[Any] = None
        color_bytes: Optional[bytes] = pkt.get("color_bytes") or pkt.get("color_jpg")
        fmt = pkt.get("fmt")

        if not color_bytes:
            frame_payload = None
        elif fmt == "jpeg":
            if decode:
                if cv2 is None:
                    raise RuntimeError("JPEG çözmek için OpenCV (cv2) gerekli ancak bulunamadı.")
                img = cv2.imdecode(np.frombuffer(color_bytes, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    return None
                frame_payload = img
            else:
                frame_payload = color_bytes
        elif fmt == "bgr":
            if decode:
                shape = pkt.get("color_shape")
                dtype = pkt.get("color_dtype", "uint8")
                if not shape:
                    return None
                try:
                    arr = np.frombuffer(color_bytes, dtype=np.dtype(dtype)).reshape(tuple(shape))
                except Exception:
                    return None
                frame_payload = arr
            else:
                frame_payload = color_bytes
        else:
            frame_payload = color_bytes if not decode else None

        return {
            "frame": frame_payload,
            "timestamp": pkt.get("sensor_ts"),
            "mono_ns": pkt.get("mono_ns"),
            "seq": pkt.get("seq"),
            "ir_left": pkt.get("irL_bytes"),
            "ir_right": pkt.get("irR_bytes"),
            "fmt": fmt,
            "color_bytes": color_bytes,
            "color_shape": pkt.get("color_shape"),
            "color_dtype": pkt.get("color_dtype"),
            "ir_fmt": pkt.get("ir_fmt"),
            "ir_left_shape": pkt.get("irL_shape"),
            "ir_left_dtype": pkt.get("irL_dtype"),
            "ir_right_shape": pkt.get("irR_shape"),
            "ir_right_dtype": pkt.get("irR_dtype"),
            "raw": pkt,
        }

    def get_imu(self, timeout_ms: int = 100):
        """Gyro + interpolasyonlu accel değerlerini ve zaman bilgisini döndürür."""
        msg = self.recv_imu(timeout_ms=timeout_ms)
        if not msg:
            return None

        return {
            "gyro": (msg.get("gyro_x"), msg.get("gyro_y"), msg.get("gyro_z")),
            "accel": (msg.get("accel_x"), msg.get("accel_y"), msg.get("accel_z")),
            "timestamp": msg.get("ts"),
            "mono_ns": msg.get("mono_ns"),
            "seq": msg.get("seq"),
        }

    def get_ir_pair(self, timeout_ms: int = 300, decode: bool = True):
        """Sol/sağ IR karelerini ve zaman bilgisini döndürür."""
        pkt = self.recv_frames(timeout_ms=timeout_ms)
        if not pkt:
            return None

        left_bytes: Optional[bytes] = pkt.get("irL_bytes") or pkt.get("irL_jpg")
        right_bytes: Optional[bytes] = pkt.get("irR_bytes") or pkt.get("irR_jpg")
        fmt = pkt.get("ir_fmt") or pkt.get("fmt")

        def _decode_ir(buf: Optional[bytes], shape_key: str, dtype_key: str) -> Optional[Any]:
            if not buf:
                return None
            if not decode:
                return buf
            if fmt == "jpeg":
                if cv2 is None:
                    raise RuntimeError("JPEG çözmek için OpenCV (cv2) gerekli ancak bulunamadı.")
                img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_GRAYSCALE)
                return img
            if fmt in ("bgr", "y8"):
                shape = pkt.get(shape_key)
                dtype = pkt.get(dtype_key, "uint8")
                if not shape:
                    return None
                try:
                    arr = np.frombuffer(buf, dtype=np.dtype(dtype)).reshape(tuple(shape))
                except Exception:
                    return None
                return arr
            return None

        left = _decode_ir(left_bytes, "irL_shape", "irL_dtype")
        right = _decode_ir(right_bytes, "irR_shape", "irR_dtype")

        return {
            "left": left,
            "right": right,
            "left_bytes": left_bytes,
            "right_bytes": right_bytes,
            "timestamp": pkt.get("sensor_ts"),
            "mono_ns": pkt.get("mono_ns"),
            "seq": pkt.get("seq"),
            "fmt": fmt,
            "raw": pkt,
        }


    # --------------- CLOSE ---------------
    def close(self):
        self._stop.set()
        if self.is_master:
            if self._video_pipe:
                try: self._video_pipe.stop()
                except Exception: pass
                self._video_pipe = None
            if self._imu_pipe:
                try: self._imu_pipe.stop()
                except Exception: pass
                self._imu_pipe = None

        for sname in ["pub_frames","pub_imu","rep_ctrl","sub_frames","sub_imu","req_ctrl"]:
            sock = getattr(self, sname, None)
            try:
                if sock is not None:
                    sock.close(0)
            except Exception:
                pass


# ===========================
# Standalone çalıştırma
# ===========================
if __name__ == "__main__":
    mux = RSMux(host=HOST, enable_loopback=True, debug_show_frames=False)
    role = "MASTER" if mux.is_master else "CLIENT"
    print(f"[RSMux] role = {role}")
    mux.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        mux.close()
