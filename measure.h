#pragma once
#include <qfiledialog.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <io.h>
#include <limits>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h>
#include<qstring.h>
#include "ui_paramemeasure.h"
#include<fstream>
using namespace Eigen;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec4_4 = Eigen::Matrix<double, 4, 4>;

using namespace std;

class Measure : public QObject
{
	Q_OBJECT
		struct Shapes
	{
		string shapeName;//形状名称
		float shapeParams[10];//形状参数
		int shapeNum;
		int beginNum, endNum;
	};
	struct point {
		float x, y, z;
		point(float x, float y, float z) :x(x), y(y), z(z) {};
	};
	struct  plane
	{
		float a, b, c, d;
		float paray;
	};

public:
	Measure() {
		//qRegisterMetaType<Measure>("Measure");
	}
	Measure(const Measure& sp) {
		//qRegisterMetaType<Measure>("Measure");
	}
	void mygetFilesMeasure(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName);
	void getFilesMeasure(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName);
	void splitStringMeasure(const string& str, float* res, const string& c);
	void getTxtDataMeasure(std::string file_path, int& shapeSize, int& pointSize, std::vector<Shapes>& shapes, std::vector<point>& points);
	void getPcdData(std::string file_path, int& pointSize, std::vector<point>& points);
	void getTxtData(std::string file_path, int& shapeSize, int& pointSize, std::vector<Shapes>& shapes, std::vector<point>& points);
	void getdata(std::string file_path, int& shapeSize, int& pointSize, std::vector<Shapes>& shapes, std::vector<point>& points);
	int sgn(float x);
	bool getLunYuan(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, vector<Shapes>& sh, Eigen::Vector3f nor);
	bool getMinCyl(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, Shapes& sh);
	bool getMaxCyls(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, vector<Shapes>& shs);
	bool getMaxCyl(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, Shapes& sh);
	bool getOurPlane(vector<Shapes>& shape, Shapes& p1, Shapes& p2);
	void getBoxss(vector<point> points, float& diameter, vector<float>& box, vector<float>& boxCenter);
	void getXYZAxis(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points);
	void getBottomCenter(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, vector<float>& boxCenter1, vector<float>& boxCenter2);
	int startMeasure(string AlloutPath);
	bool measure_getBenchMark(map<string, vector<float>>& allResult, float& xRotInit, float& zRotInit, float& yTraInit, float& zTraInit, char* szFname, point& center, string allPath);
signals:
	void show_message(string);
	void show_MeaureResultmessage(string);
	void show_startSpotProgess();// 开始接触斑测量进程
	void changMeasureProgessBar(int);// 发送信号加载进度条
	void sendMeasureThreadMessage();
};
