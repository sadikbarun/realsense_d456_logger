#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using namespace std::chrono;

static std::atomic_bool g_stop{false};
static void on_sigint(int) { g_stop = true; }

struct Sample {
    double sensor_ms{0.0};
    int64_t mono_ns{0};
    float x{0}, y{0}, z{0};
};

struct Config {
    int width = 1280, height = 720, fps = 30;
    int gyro_hz = 200, accel_hz = 400;
    int jpeg_quality = 100; // 0-100 mapped to PNG compression (100 => none)
    double warmup_sec = 3.0;
    bool prefer_color_mjpeg = false; // try MJPEG, otherwise use best available raw color format
};

struct Shared {
    std::mutex mu;
    // Pending samples (set by motion callback)
    Sample pend_accel{};
    Sample pend_gyro{};
    bool has_pend_accel{false};
    bool has_pend_gyro{false};
    // Latest committed samples
    Sample last_accel{};
    Sample last_gyro{};
    std::condition_variable cv_accel, cv_gyro;

    // CSV files
    std::ofstream events_csv;
    std::ofstream imu_csv;

    // Stats
    std::atomic<long long> color_rx{0}, gyro_rx{0}, accel_rx{0};

    // Control
    std::atomic_bool warmed_up{false};
};

static inline int64_t now_ns() {
    return duration_cast<nanoseconds>(Clock::now().time_since_epoch()).count();
}

static int64_t atomic_unique_ns(std::atomic<int64_t>& prev) {
    while (true) {
        int64_t old = prev.load(std::memory_order_relaxed);
        int64_t t = now_ns();
        int64_t neu = (t > old) ? t : (old + 1);
        if (prev.compare_exchange_weak(old, neu, std::memory_order_release, std::memory_order_relaxed))
            return neu;
    }
}

static std::string make_out_dir() {
    auto t = std::time(nullptr);
    std::tm tm{};
#ifdef WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << "rs_logs_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

static void open_csvs(const fs::path &out, Shared &S) {
    S.events_csv.open(out / "events.csv");
    S.imu_csv.open(out / "imu_pairs.csv");
    S.events_csv << "frame_idx,mono_s,sensor_s,color_file,gx,gy,gz,ax,ay,az\n";
    S.imu_csv << "mono_s,sensor_s,gx,gy,gz,ax,ay,az\n";
}

static void motion_cb(Shared &S, std::atomic<int64_t> &prev_ns, const rs2::frame &f) {
    auto mf = f.as<rs2::motion_frame>();
    if (!mf) return;
    rs2_vector v = mf.get_motion_data();
    double rs_ms = f.get_timestamp();
    int64_t mono = atomic_unique_ns(prev_ns);

    std::unique_lock<std::mutex> lk(S.mu);
    if (f.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        S.pend_accel = {rs_ms, mono, v.x, v.y, v.z};
        S.has_pend_accel = true;
        lk.unlock();
        S.cv_accel.notify_one();
    }
    else if (f.get_profile().stream_type() == RS2_STREAM_GYRO) {
        S.pend_gyro = {rs_ms, mono, v.x, v.y, v.z};
        S.has_pend_gyro = true;
        lk.unlock();
        S.cv_gyro.notify_one();
    }
}

int main(int argc, char **argv) {
    std::signal(SIGINT, on_sigint);
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nexti = [&](int &d) { if (i + 1 < argc) d = std::stoi(argv[++i]); };
        auto nextd = [&](double &d) { if (i + 1 < argc) d = std::stod(argv[++i]); };
        if (a == "--quality") nexti(cfg.jpeg_quality);
        else if (a == "--warmup-sec") nextd(cfg.warmup_sec);
        else if (a == "--color-mjpeg") { int v = 1; nexti(v); cfg.prefer_color_mjpeg = (v != 0); }
    }

    Shared S;
    const fs::path out_dir = make_out_dir();
    fs::create_directories(out_dir / "color");
    open_csvs(out_dir, S);

    std::cout << "[INFO] Output: " << out_dir << "\n";

    // Discover device and sensors
    rs2::context ctx;
    auto devs = ctx.query_devices();
    if (devs.size() == 0) { std::cerr << "[ERR] No RealSense device found\n"; return 1; }
    rs2::device dev = devs.front();

    rs2::sensor rgb, motion;
    for (auto &s : dev.query_sensors()) {
        try {
            std::string name = s.get_info(RS2_CAMERA_INFO_NAME);
            if (name.find("RGB") != std::string::npos) rgb = s;
            if (name.find("Motion") != std::string::npos) motion = s;
        } catch (...) {}
    }
    if (!rgb) { std::cerr << "[ERR] RGB sensor not found\n"; return 2; }
    if (!motion) { std::cerr << "[ERR] Motion sensor not found\n"; return 3; }

    // Options for timing/latency
    try { if (rgb.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) rgb.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f); } catch (...) {}
    try { if (motion.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) motion.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f); } catch (...) {}
    try { if (motion.supports(RS2_OPTION_ENABLE_MOTION_CORRECTION)) motion.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 1.f); } catch (...) {}
    try { if (motion.supports(RS2_OPTION_FRAMES_QUEUE_SIZE)) motion.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2048.f); } catch (...) {}

    // Select profiles
    rs2::stream_profile rgb_prof_bgr, rgb_prof_rgb, rgb_prof_yuyv, rgb_prof_mjpg;
    for (auto &p : rgb.get_stream_profiles()) {
        try {
            if (p.stream_type() == RS2_STREAM_COLOR && p.fps() == cfg.fps) {
                auto vp = p.as<rs2::video_stream_profile>();
                if (vp && vp.width() == cfg.width && vp.height() == cfg.height) {
                    if (p.format() == RS2_FORMAT_BGR8)  rgb_prof_bgr = p;
                    if (p.format() == RS2_FORMAT_RGB8)  rgb_prof_rgb = p;
                    if (p.format() == RS2_FORMAT_YUYV)  rgb_prof_yuyv = p;
                    if (p.format() == RS2_FORMAT_MJPEG) rgb_prof_mjpg = p;
                }
            }
        } catch (...) {}
    }
    rs2::stream_profile rgb_prof;
    if (cfg.prefer_color_mjpeg && rgb_prof_mjpg)      rgb_prof = rgb_prof_mjpg;
    else if (rgb_prof_bgr)                            rgb_prof = rgb_prof_bgr;
    else if (rgb_prof_rgb)                            rgb_prof = rgb_prof_rgb;
    else if (rgb_prof_yuyv)                           rgb_prof = rgb_prof_yuyv;
    if (!rgb_prof) { std::cerr << "[ERR] Suitable RGB profile not found\n"; return 4; }

    rs2::stream_profile pg, pa;
    for (auto &p : motion.get_stream_profiles()) {
        try {
            if (p.stream_type() == RS2_STREAM_GYRO && p.fps() == cfg.gyro_hz)  pg = p;
            if (p.stream_type() == RS2_STREAM_ACCEL && p.fps() == cfg.accel_hz) pa = p;
        } catch (...) {}
    }
    if (!pg || !pa) { std::cerr << "[ERR] Suitable IMU profiles not found\n"; return 5; }

    // Open sensors
    try { motion.open({pg, pa}); } catch (...) { try { motion.open(motion.get_stream_profiles()); } catch (...) { std::cerr << "[ERR] Motion open failed\n"; return 6; } }
    try { rgb.open(rgb_prof); } catch (...) { std::cerr << "[ERR] RGB open failed\n"; return 7; }

    std::atomic<int64_t> prev_ns{0};
    motion.start([&](rs2::frame f) { motion_cb(S, prev_ns, f); });

    // Color thread
    std::thread color_th([&] {
        try {
            rs2::frame_queue q(2);
            rgb.start(q);
            int idx = 0; int64_t prev_l = 0;
            // Warm-up gate
            auto t0 = Clock::now();
            while (!g_stop && duration<double>(Clock::now() - t0).count() < cfg.warmup_sec) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            S.warmed_up = true;
            idx = 0; // reset index after warm-up

            while (!g_stop) {
                rs2::frame f;
                if (!q.poll_for_frame(&f)) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
                auto vf = f.as<rs2::video_frame>(); if (!vf) continue;
                double rs_ms = vf.get_timestamp();
                int64_t t = now_ns(); int64_t mono = (t > prev_l) ? t : (prev_l + 1); prev_l = mono;

                std::ostringstream ossname; ossname << "frame_" << std::setw(4) << std::setfill('0') << idx;
                const std::string base_name = ossname.str();
                rs2_format fmt = vf.get_profile().format();
                std::string ext;
                fs::path path;

                if (fmt == RS2_FORMAT_MJPEG) {
                    ext = ".jpg";
                    path = out_dir / "color" / (base_name + ext);
                    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(vf.get_data());
                    size_t sz = vf.get_data_size();
                    std::ofstream ofs(path.string(), std::ios::binary);
                    ofs.write(reinterpret_cast<const char *>(ptr), static_cast<std::streamsize>(sz));
                } else {
                    ext = ".png";
                    path = out_dir / "color" / (base_name + ext);
                    cv::Mat bgr;
                    if (fmt == RS2_FORMAT_BGR8) {
                        cv::Mat img(vf.get_height(), vf.get_width(), CV_8UC3, const_cast<void *>(vf.get_data()));
                        bgr = img.clone();
                    } else if (fmt == RS2_FORMAT_RGB8) {
                        cv::Mat img(vf.get_height(), vf.get_width(), CV_8UC3, const_cast<void *>(vf.get_data()));
                        cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
                    } else if (fmt == RS2_FORMAT_YUYV) {
                        cv::Mat img(vf.get_height(), vf.get_width(), CV_8UC2, const_cast<void *>(vf.get_data()));
                        cv::cvtColor(img, bgr, cv::COLOR_YUV2BGR_YUYV);
                    } else {
                        std::cerr << "[WARN] Unsupported color format " << fmt << ", skipping frame\n";
                        continue;
                    }
                    int png_compression = 0;
                    if (cfg.jpeg_quality <= 0) png_compression = 9;
                    else if (cfg.jpeg_quality >= 100) png_compression = 0;
                    else {
                        png_compression = (100 - cfg.jpeg_quality) / 11;
                        if (png_compression < 0) png_compression = 0;
                        if (png_compression > 9) png_compression = 9;
                    }
                    std::vector<int> params{cv::IMWRITE_PNG_COMPRESSION, png_compression};
                    cv::imwrite(path.string(), bgr, params);
                }

                const std::string name = base_name + ext;

                Sample g{}, a{};
                { std::lock_guard<std::mutex> lk(S.mu); g = S.last_gyro; a = S.last_accel; }
                if (S.events_csv.is_open()) {
                    S.events_csv << std::fixed << std::setprecision(6)
                                 << idx << "," << mono / 1e9 << "," << rs_ms / 1000.0 << ","
                                 << (std::string("color/") + name) << ","
                                 << g.x << "," << g.y << "," << g.z << ","
                                 << a.x << "," << a.y << "," << a.z << "\n";
                }
                S.color_rx.fetch_add(1, std::memory_order_relaxed);
                ++idx;
            }
            try { rgb.stop(); } catch (...) {}
        } catch (...) {}
    });

    // Accel consumer thread
    std::thread accel_th([&] {
        while (!g_stop) {
            std::unique_lock<std::mutex> lk(S.mu);
            S.cv_accel.wait_for(lk, std::chrono::milliseconds(100), [&] { return S.has_pend_accel || g_stop.load(); });
            if (g_stop) break;
            if (!S.warmed_up.load()) { S.has_pend_accel = false; continue; }
            if (S.has_pend_accel) {
                S.last_accel = S.pend_accel; S.has_pend_accel = false;
                S.accel_rx.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    // Gyro consumer thread (writes imu_pairs.csv)
    std::thread gyro_th([&] {
        while (!g_stop) {
            std::unique_lock<std::mutex> lk(S.mu);
            S.cv_gyro.wait_for(lk, std::chrono::milliseconds(100), [&] { return S.has_pend_gyro || g_stop.load(); });
            if (g_stop) break;
            if (!S.warmed_up.load()) { S.has_pend_gyro = false; continue; }
            if (S.has_pend_gyro) {
                S.last_gyro = S.pend_gyro; S.has_pend_gyro = false;
                Sample g = S.last_gyro; Sample a = S.last_accel;
                lk.unlock();
                if (S.imu_csv.is_open()) {
                    S.imu_csv << std::fixed << std::setprecision(6)
                              << g.mono_ns / 1e9 << "," << g.sensor_ms / 1000.0 << ","
                              << g.x << "," << g.y << "," << g.z << ","
                              << a.x << "," << a.y << "," << a.z << "\n";
                }
                S.gyro_rx.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    // Run until SIGINT
    while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Stop
    try { motion.stop(); } catch (...) {}
    if (color_th.joinable()) color_th.join();
    if (accel_th.joinable()) accel_th.join();
    if (gyro_th.joinable()) gyro_th.join();
    try { if (S.events_csv.is_open()) S.events_csv.close(); if (S.imu_csv.is_open()) S.imu_csv.close(); } catch (...) {}

    std::cout << "[STATS] color_rx=" << S.color_rx.load()
              << ", gyro_rx=" << S.gyro_rx.load()
              << ", accel_rx=" << S.accel_rx.load() << "\n";
    return 0;
}
