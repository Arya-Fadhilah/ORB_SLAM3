#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;

void LoadImages(const string &strPathCam0, const string &strPathCam1,
                 const string &strPathTimes,
                 vector<string> &vstrImageLeft, vector<string> &vstrImageRight,
                 vector<double> &vTimeStamps)
{
    ifstream fTimes(strPathTimes.c_str());
    vTimeStamps.reserve(1200);
    vstrImageLeft.reserve(1200);
    vstrImageRight.reserve(1200);

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
            vstrImageRight.push_back(strPathCam1 + "/" + string(buf) + ".jpg");
            idx++;
        }
    }
}

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << "Usage: ./stereo_usvinland path_to_vocabulary path_to_settings path_to_sequence_folder path_to_timestamps_file" << endl;
        cerr << "Contoh: ./stereo_usvinland Vocabulary/ORBvoc.txt usvinland_stereo_settings.yaml ~/USVinLand/Rectified ~/USVinLand/Timestamp.txt" << endl;
        return 1;
    }

    string strPathCam0 = string(argv[3]) + "/cam0";
    string strPathCam1 = string(argv[3]) + "/cam1";
    string strPathTimes = string(argv[4]);

    vector<string> vstrImageLeft, vstrImageRight;
    vector<double> vTimestampsCam;
    LoadImages(strPathCam0, strPathCam1, strPathTimes, vstrImageLeft, vstrImageRight, vTimestampsCam);

    int nImages = vstrImageLeft.size();
    if(nImages <= 0)
    {
        cerr << "ERROR: gagal load gambar dari " << strPathCam0 << endl;
        return 1;
    }
    if(vstrImageRight.size() != vstrImageLeft.size())
    {
        cerr << "ERROR: jumlah gambar cam0 (" << vstrImageLeft.size()
             << ") dan cam1 (" << vstrImageRight.size() << ") tidak sama" << endl;
        return 1;
    }

    cout << "Jumlah gambar: " << nImages << endl;
    cout << "Mode: STEREO murni (tanpa IMU) -- skala metrik dari baseline 80mm (estimasi, bukan kalibrasi resmi)" << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    for(int ni = 0; ni < nImages; ni++)
    {
        cv::Mat imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        cv::Mat imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
        if(imLeft.empty())
        {
            cerr << "Failed to load LEFT image at index " << ni << ": " << vstrImageLeft[ni] << endl;
            return 1;
        }
        if(imRight.empty())
        {
            cerr << "Failed to load RIGHT image at index " << ni << ": " << vstrImageRight[ni] << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        SLAM.TrackStereo(imLeft, imRight, tframe);
        cout << "\rProcessing frame " << ni << "/" << nImages << std::flush;
    }
    cout << endl;

    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    SLAM.SavePointCloud("PointCloud.ply");
    return 0;
}