//
// Created by eirik on 29.03.19.
//
#pragma once
#ifndef PROJECT_CAMPARAM_IO_H
#define PROJECT_CAMPARAM_IO_H

using namespace cv;
using namespace std;

namespace CamParamIO {

    void readCamParams(const char *filePath, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);//, cv::FileNode features);

    void writeCamParams(cv::Mat cameraMatrix, cv::Mat distCoeffs, const char *filePath);

    int listFiles(const char *directoryPath, const char *fileType, vector<string> &list);

    void writeImageList(const char *filePath, const char *outputName, const char *fileType);

    void readImageList(const char *filePath, vector<string> &images);
}

#endif //PROJECT_CAMPARAM_IO_H
