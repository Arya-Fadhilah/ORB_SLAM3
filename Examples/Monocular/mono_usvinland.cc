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

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << "Usage: ./usvinland_mono path_to_vocabulary path_to_settings path_to_sequence_folder path_to_timestamps_file" << endl;
        cerr << "Contoh: ./usvinland_mono Vocabulary/ORBvoc.txt usvinland_mono_settings.yaml ~/USVinLand/Rectified ~/USVinLand/Timestamp.txt" << endl;
        return 1;
    }

    string strPathCam0 = string(argv[3]) + "/cam0";
    string strPathTimes = string(argv[4]);

    vector<string> vstrImageLeft;
    vector<double> vTimestampsCam;
    LoadImages(strPathCam0, strPathTimes, vstrImageLeft, vTimestampsCam);

    int nImages = vstrImageLeft.size();
    if(nImages <= 0)
    {
        cerr << "ERROR: gagal load gambar dari " << strPathCam0 << endl;
        return 1;
    }

    cout << "Jumlah gambar: " << nImages << endl;
    cout << "PERINGATAN: mode monocular tanpa IMU -- trajectory yang dihasilkan TIDAK punya skala metrik absolut." << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    for(int ni = 0; ni < nImages; ni++)
    {
        cv::Mat im = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        if(im.empty())
        {
            cerr << "Failed to load image at index " << ni << ": " << vstrImageLeft[ni] << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        SLAM.TrackMonocular(im, tframe);
        cout << "\rProcessing frame " << ni << "/" << nImages << std::flush;
    }
    cout << endl;

    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory_mono.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory_mono.txt");
    SLAM.SavePointCloud("PointCloud_mono.ply");
    return 0;
}