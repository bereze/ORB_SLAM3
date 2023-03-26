#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core.hpp>

#include "System.h"
#include "ImuTypes.h"

void LoadData(const string &path_to_data, vector<string> &strImageFilenames,
              vector<double> &timestampCam, vector<double> &timestampImu,
              vector<cv::Point3f> &accData, vector<cv::Point3f> &gyroData);

using namespace std;

int main(int argc, char** argv) {
    if (argc < 4) {
        cerr << endl << "Usage: ./mono_inertial_kaist path_to_vocabulary path_to_settings path_to_data" << endl;
        return 1;
    }

    vector<string> strImageFilenames;
    vector<double> timestampCam;
    vector<cv::Point3f> accData, gyroData;
    vector<double> timestampImu;
    int nImage, nImu;
    int first_imu = 0;

    string path_to_data(argv[3]);
    LoadData(path_to_data, strImageFilenames, timestampCam, timestampImu, accData, gyroData);

    nImage = (int)strImageFilenames.size();
    nImu = (int)timestampImu.size();
    if (nImage <= 0 || nImu <= 0) {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first

    while(timestampImu[first_imu] <= timestampCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered

    cout.precision(17);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Main loop
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> imuMeas;
    for (int i = 0; i < nImage; ++i) {
        // Read image from file
        im = cv::imread(strImageFilenames[i],cv::IMREAD_UNCHANGED);

        double tframe = timestampCam[i];

        if (im.empty()) {
            cerr << endl << "Failed to load image at: "
                 << strImageFilenames[i] << endl;
            return 1;
        }

        if (imageScale != 1.f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Load imu measurements from previous frame
        imuMeas.clear();

        if (i > 0) {
            while (timestampImu[first_imu] <= timestampCam[i]) {
                imuMeas.emplace_back(accData[first_imu].x, accData[first_imu].y, accData[first_imu].z,
                                     gyroData[first_imu].x, gyroData[first_imu].y, gyroData[first_imu].z,
                                     timestampImu[first_imu]);
                first_imu++;
            }
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the image to the SLAM system
        cout << "read frame at " << tframe << " with imu " << imuMeas.size() << endl;
        SLAM.TrackMonocular(im,tframe,imuMeas); // TODO change to monocular_inertial
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // Wait to load the next frame
        double T=0;
        if(i < nImage-1)
            T = timestampCam[i+1] - tframe;
        else if (i > 0)
            T = tframe - timestampCam[i-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }

    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadData(const string &path_to_data, vector<string> &strImageFilenames,
              vector<double> &timestampCam, vector<double> &timestampImu,
              vector<cv::Point3f> &accData, vector<cv::Point3f> &gyroData) {
    // load image
    const string imgTimestampFile = path_to_data + "/stereo_stamp.csv";
    ifstream fTimes;
    fTimes.open(imgTimestampFile);
    strImageFilenames.reserve(10000);
    timestampCam.reserve(10000);

    string line;
    while (getline(fTimes, line)) {
        if (!line.empty()) {
            stringstream ss;
            ss << line;
            strImageFilenames.push_back(path_to_data + "/image/stereo_left/" + ss.str() + ".png");
            double t;
            ss >> t;
            timestampCam.push_back(t * 1e-9);
        }
    }

    // load imu
    const string imuFile = path_to_data + "/xsens_imu.csv";
    ifstream fImu;
    fImu.open(imuFile);
    timestampImu.reserve(50000);
    accData.reserve(50000);
    gyroData.reserve(50000);

    while (getline(fImu, line)) {
        if (line[0] == '#')
            continue;

        string item;
        size_t pos;
        double data[17];
        int count = 0;
        while ((pos = line.find(',')) != string::npos) {
            item = line.substr(0, pos);
            data[count++] = stod(item);
            line.erase(0, pos+1);
        }
        item = line.substr(0, pos);
        data[16] = stod(item);

        timestampImu.push_back(data[0] * 1e-9);
        accData.emplace_back(data[11], data[12], data[13]);
        gyroData.emplace_back(data[8], data[9], data[10]);
    }

    cout << "Data loaded!!" << endl;
}
