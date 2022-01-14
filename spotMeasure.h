#pragma once
// 本代码用于接触斑点测量以及可视化处理
#include <iostream>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/shot_omp.h>
#include "pcl/features/fpfh.h"
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>//拟合空间圆使用的
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include<pcl/sample_consensus/ransac.h>
#include "ui_paramemeasure.h"
#include <QMetaType>
#include<qdebug.h>
typedef pcl::PointXYZ PointT;
using namespace std;
using namespace Eigen;

//class  spotMethod : public QObject
class  spotMethod : public QObject
{
	Q_OBJECT

public:
	spotMethod() {
		//qRegisterMetaType<spotMethod>("spotMethod");
	}
	spotMethod(const spotMethod& sp) {
		//qRegisterMetaType<spotMethod>("spotMethod");
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis2(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);
	double distanceFromPointToLine(double x, double z);
	double distanceBetweenTwoPoint(double x1, double y1, double z1, double x2, double y2, double z2);
	vector<double> cel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	vector<double> measurePatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	vector<double> computeCircle3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr yuantu, vector<int> indexs);
	void getFilesN(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName);
	double lunkuoxian(double r, double h, double l);
	vector<string> split(const string& str, const string& delim);
	/*int startPoint(string splitResultPaTH);*/
	// 以下为基于结构先验进行的坐标平移以及分割
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	int startTransform(string inputDataPath, string outPuthDataPath);
signals:
	void show_SpotProcessSignal(string);//显示运算进度的信号
	void show_spotResultSignal(string);// 显示运算结果的输出信号
	void changProgessBar(int);// 发送信号改变进度条
	void sendMeasureThreadMessage2();// 发送信号告知主程序当前程序运行结束
};
