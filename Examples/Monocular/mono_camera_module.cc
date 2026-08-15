/**
* This file is part of ORB-SLAM3 modified for Generic Webcams / Raspberry Pi Cameras.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp> // Library OpenCV untuk akses Webcam
#include <opencv2/imgproc/imgproc.hpp>

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session..." << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {

    // Validasi argumen terminal
    if (argc < 3 || argc > 5) {
        cerr << endl
             << "Usage: ./mono_webcam path_to_vocabulary path_to_settings (camera_index) (trajectory_file_name)"
             << endl;
        return 1;
    }

    int cam_index = 0; // Default index video0 (/dev/video0)
    string file_name = "";

    if (argc >= 4) {
        // Jika argumen ke-4 adalah angka, jadikan indeks kamera
        stringstream ss(argv[3]);
        if (!(ss >> cam_index)) {
            // Jika bukan angka, asumsikan itu nama file trajektori
            file_name = string(argv[3]);
            cam_index = 0;
        }
    }
    if (argc == 5) {
        file_name = string(argv[4]);
    }

    // Handler interupsi CTRL+C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // --- INISIALISASI WEBCAM / RASPBERRY PI CAMERA VIA OPENCV ---
    cv::VideoCapture cap;
    
    // Membuka kamera berdasarkan indeks perangkat (0, 1, dst)
    cap.open(cam_index, cv::CAP_ANY); 

    if (!cap.isOpened()) {
        cerr << "ERROR: Tidak dapat membuka interface kamera pada indeks: " << cam_index << endl;
        return -1;
    }

    // Atur resolusi kamera (Sesuaikan dengan resolusi yang kamu tulis di file .yaml)
    // Contoh di bawah mengatur resolusi standar HD 640x480
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    cout << "\n--- Spesifikasi Hardware Kamera Aktif ---" << endl;
    cout << "Lebar Frame  : " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
    cout << "Tinggi Frame : " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "FPS Kamera   : " << cap.get(cv::CAP_PROP_FRAME_FPS) << endl;
    cout << "-----------------------------------------\n" << endl;

    // Menginisialisasi Sistem SLAM (Mode Monokular Murni tanpa IMU bawaan RealSense)
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    cv::Mat frame_raw, im;
    double t_resize = 0.f;
    double t_track = 0.f;

    cout << "Memulai penangkapan frame streaming. Tekan CTRL+C untuk berhenti secara aman." << endl;

    // Perulangan Utama Visual Tracking
    while (!SLAM.isShutDown() && b_continue_session)
    {
        // Ambil frame terbaru dari kamera
        cap >> frame_raw;

        if (frame_raw.empty()) {
            cerr << "Peringatan: Terjadi frame kosong dari kamera!" << endl;
            continue;
        }

        // Ambil timestamp waktu nyata saat frame tiba di komputer
        // Konversi ke satuan detik (double)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double timestamp = tv.tv_sec + tv.tv_usec * 1e-6;

        // ORB-SLAM3 membutuhkan gambar dalam format Grayscale (Hitam Putih)
        if (frame_raw.channels() == 3) {
            cv::cvtColor(frame_raw, im, cv::COLOR_BGR2GRAY);
        } else {
            im = frame_raw.clone();
        }

        // Skalasi Citra jika parameter pendukung diaktifkan di file .yaml
        if (imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Masukkan data citra dan waktu komputasi ke core ORB-SLAM3
        SLAM.TrackMonocular(im, timestamp);
    }

    // Bersihkan buffer hardware kamera sebelum menutup program
    cap.release();
    cout << "Menutup subsistem kamera." << endl;
    
    // Matikan seluruh thread pelacakan SLAM secara aman
    SLAM.Shutdown();
    cout << "System shutdown sukses!" << endl;

    return 0;
}