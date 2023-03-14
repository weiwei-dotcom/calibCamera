#include "cameraCalibrate.h"

using namespace std;
using namespace cv;

cameraCalibrate::cameraCalibrate() {
    this->Intrinsic = Mat::zeros(3, 3, CV_32FC1);
}

bool cameraCalibrate::getIntrinsic(string cameraName) {
	//从文件中查找是否有同名的相机内参文件
	string calibDataDir("E:\\Visual_项目\\opencvSFM_VS"); 
	string IntrinsicFile = calibDataDir + "\\calibData\\Intrinsic" + "\\" + cameraName + ".txt";
    string calibImgFile = calibDataDir + "\\calibData\\Img\\" + cameraName + "*";
    // 读取相机的内参文件
    ifstream fr(IntrinsicFile);
    if (fr.is_open()) {
        cout << "相机内参文件打开成功，正在返回内参矩阵" << endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                fr >> this->Intrinsic.at<float>(i, j);
            }
        }
        fr.close();
        return true;
    }
	// 如果相机内参矩阵不存在，则到相机标定图像文件中查找是否有同名的相机图像文件
	else {
        cout << "相机内参文件不存在，查找是否存在足够数量标定图像" << endl;
		vector<cv::String> fileNames;
		cv::glob(calibImgFile, fileNames);
		// 不存在对应的标定图像
		if (fileNames.size() == 0) { 
			cout << "该相机不存在对应标定图像，无法完成标定" << endl;
			return false;
		}
		// 存在标定图像，但是对应数量过少
		else if (fileNames.size() > 0 && fileNames.size() < 10) {
			cout << "相机标定图像数量过少,无法完成标定" << endl;
			return false;
		}
		//利用标定图像进行标定
		else {
            cout << "标定图像存在且数量足够，正在对其进行标定" << endl;
            int boardWidth = 17;  // 棋盘格横向内角点数量
            int boardHeight = 12; // 棋盘格纵向内角点数量
            float squareSize = 1.f; // 棋盘格格子的大小，单位为米,随便设置，不影响相机内参计算
            Size boardSize(boardWidth, boardHeight);

            vector<vector<Point3f>> objectPoints;
            vector<vector<Point2f>> imagePoints;
            vector<Point2f> corners;
            Mat image, gray;
            namedWindow("image", WINDOW_NORMAL);
            if (fileNames.size() > 15)
                fileNames.erase(fileNames.begin() + 15, fileNames.end());
            for (size_t i = 0; i < fileNames.size(); i++)
            {
                image = imread(fileNames[i], IMREAD_COLOR);
                cvtColor(image, gray, COLOR_BGR2GRAY);
                bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
                if (found)
                {
                    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
                    vector<Point3f> objectCorners;
                    for (int j = 0; j < boardHeight; j++)
                    {
                        for (int k = 0; k < boardWidth; k++)
                        {
                            objectCorners.push_back(Point3f(k * squareSize, j * squareSize, 0));
                        }
                    }
                    objectPoints.push_back(objectCorners);
                    imagePoints.push_back(corners);
                }
            }
            Mat cameraMatrix;
            Mat distCoeffs;
            vector<Mat> rvecs, tvecs;
            calibrateCamera(objectPoints, imagePoints, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
            cameraMatrix.convertTo(cameraMatrix, CV_32FC1);
            this->Intrinsic = cameraMatrix;
            ofstream of(IntrinsicFile);
            if (of.is_open()) {
                cout << "内参文件创建成功，正在写入数据" << endl;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        if (j < 2)
                            of << cameraMatrix.at<float>(i, j) << " ";
                        else
                            of << cameraMatrix.at<float>(i, j) << "\n";
                    }
                }
            }
            return true;
		}
	}
}