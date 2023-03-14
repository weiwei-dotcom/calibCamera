#include "cameraCalibrate.h"

using namespace std;
using namespace cv;

cameraCalibrate::cameraCalibrate() {
    this->Intrinsic = Mat::zeros(3, 3, CV_32FC1);
}

bool cameraCalibrate::getIntrinsic(string cameraName) {
	//���ļ��в����Ƿ���ͬ��������ڲ��ļ�
	string calibDataDir("E:\\Visual_��Ŀ\\opencvSFM_VS"); 
	string IntrinsicFile = calibDataDir + "\\calibData\\Intrinsic" + "\\" + cameraName + ".txt";
    string calibImgFile = calibDataDir + "\\calibData\\Img\\" + cameraName + "*";
    // ��ȡ������ڲ��ļ�
    ifstream fr(IntrinsicFile);
    if (fr.is_open()) {
        cout << "����ڲ��ļ��򿪳ɹ������ڷ����ڲξ���" << endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                fr >> this->Intrinsic.at<float>(i, j);
            }
        }
        fr.close();
        return true;
    }
	// �������ڲξ��󲻴��ڣ�������궨ͼ���ļ��в����Ƿ���ͬ�������ͼ���ļ�
	else {
        cout << "����ڲ��ļ������ڣ������Ƿ�����㹻�����궨ͼ��" << endl;
		vector<cv::String> fileNames;
		cv::glob(calibImgFile, fileNames);
		// �����ڶ�Ӧ�ı궨ͼ��
		if (fileNames.size() == 0) { 
			cout << "����������ڶ�Ӧ�궨ͼ���޷���ɱ궨" << endl;
			return false;
		}
		// ���ڱ궨ͼ�񣬵��Ƕ�Ӧ��������
		else if (fileNames.size() > 0 && fileNames.size() < 10) {
			cout << "����궨ͼ����������,�޷���ɱ궨" << endl;
			return false;
		}
		//���ñ궨ͼ����б궨
		else {
            cout << "�궨ͼ������������㹻�����ڶ�����б궨" << endl;
            int boardWidth = 17;  // ���̸�����ڽǵ�����
            int boardHeight = 12; // ���̸������ڽǵ�����
            float squareSize = 1.f; // ���̸���ӵĴ�С����λΪ��,������ã���Ӱ������ڲμ���
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
                cout << "�ڲ��ļ������ɹ�������д������" << endl;
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