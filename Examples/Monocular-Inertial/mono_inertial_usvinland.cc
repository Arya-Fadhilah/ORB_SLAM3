#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;

void LoadImages(const string &strPathCam0, const string &strPathTimes,
                 vector<string> &vstrImageLeft, vector<double> &vTimeStamps)
{
    ifstream fTimes(strPathTimes.c_str());
    vTimeStamps.reserve(1200);
    vstrImageLeft.reserve(1200);

    int idx = 0;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if(!s.empty())
        {
            stringstream ss(s);
            double t;
            ss >> t;
            vTimeStamps.push_back(t);

            char buf[11];
            snprintf(buf, sizeof(buf), "%010d", idx);
            vstrImageLeft.push_back(strPathCam0 + "/" + string(buf) + ".jpg");
            idx++;
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu(strImuPath.c_str());
    vTimeStamps.reserve(60000);
    vAcc.reserve(60000);
    vGyro.reserve(60000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu, s);
        if(s.empty()) continue;

        replace(s.begin(), s.end(), ',', ' ');  // csv -> bisa dibaca stringstream biasa
        stringstream ss(s);

        double t, ax, ay, az, gx, gy, gz, temp;
        ss >> t >> ax >> ay >> az >> gx >> gy >> gz >> temp;

        vTimeStamps.push_back(t);
        // TODO VERIFIKASI: asumsi accel satuan 'g' -> dikonversi ke m/s^2 (BELUM PERNAH DIVALIDASI)
        vAcc.push_back(cv::Point3f(ax * 9.81f, ay * 9.81f, az * 9.81f));
        // TODO VERIFIKASI: asumsi gyro satuan deg/s -> dikonversi ke rad/s (BELUM PERNAH DIVALIDASI)
        vGyro.push_back(cv::Point3f(gx * (float)(M_PI/180.0), gy * (float)(M_PI/180.0), gz * (float)(M_PI/180.0)));
    }
}

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        cerr << "Usage: ./mono_inertial_usvinland path_to_vocabulary path_to_settings path_to_sequence_folder path_to_IMU_file path_to_timestamps_file" << endl;
        return 1;
    }

    string strPathCam0 = string(argv[3]) + "/cam0";
    string strPathTimes = string(argv[5]);
    string strPathImu = string(argv[4]) + "/IMU.csv";

    vector<string> vstrImageLeft;
    vector<double> vTimestampsCam;
    LoadImages(strPathCam0, strPathTimes, vstrImageLeft, vTimestampsCam);

    vector<double> vTimestampsImu;
    vector<cv::Point3f> vAcc, vGyro;
    LoadIMU(strPathImu, vTimestampsImu, vAcc, vGyro);

    int nImages = vstrImageLeft.size();
    int nImu = vTimestampsImu.size();
    if(nImages <= 0) { cerr << "ERROR: gagal load gambar dari " << strPathCam0 << endl; return 1; }
    if(nImu <= 0) { cerr << "ERROR: gagal load data IMU dari " << strPathImu << endl; return 1; }

    cout << "Jumlah gambar: " << nImages << ", jumlah sample IMU: " << nImu << endl;
    cout << "=====================================================================" << endl;
    cout << "PERINGATAN: Tbc dan noise parameter IMU di settings.yaml ini PLACEHOLDER" << endl;
    cout << "(dipinjam dari referensi umum EuRoC/ADIS16448), BUKAN kalibrasi asli unit" << endl;
    cout << "Mynt Eye di dataset ini. Ini eksperimen sekali-jalan -- kalau hasil jelas" << endl;
    cout << "lebih buruk dari mono/stereo murni yang sudah terbukti bagus, JANGAN" << endl;
    cout << "diteruskan/di-tuning lebih jauh. Cukup dokumentasikan sebagai percobaan." << endl;
    cout << "=====================================================================" << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

    int first_imu = 0;
    for(int ni = 0; ni < nImages; ni++)
    {
        cv::Mat im = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        if(im.empty())
        {
            cerr << "Failed to load image at index " << ni << ": " << vstrImageLeft[ni] << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        // Kumpulkan semua sample IMU sejak frame sebelumnya sampai frame ini
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        while(first_imu < nImu && vTimestampsImu[first_imu] <= tframe)
        {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                vAcc[first_imu].x, vAcc[first_imu].y, vAcc[first_imu].z,
                vGyro[first_imu].x, vGyro[first_imu].y, vGyro[first_imu].z,
                vTimestampsImu[first_imu]));
            first_imu++;
        }

        SLAM.TrackMonocular(im, tframe, vImuMeas);
        cout << "\rProcessing frame " << ni << "/" << nImages << std::flush;
    }
    cout << endl;

    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory_mono_inertial.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory_mono_inertial.txt");
    SLAM.SavePointCloud("PointCloud_mono_inertial.ply");
    return 0;
}
