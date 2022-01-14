// 本程序为在基元检测的基础上进行参数测量部分的功能函数

#include"measure.h"
//#include <qfiledialog.h>
//#include <iostream>
//#include <fstream>
//#include <vector>
//#include <map>
//#include <string>
//#include <sstream>
//#include <io.h>
//#include <limits>
//#include <Eigen/Dense>
//#define _USE_MATH_DEFINES
//#include <math.h>
using namespace std;
string thePath;

/*
***预处理步骤***
~找到轨道中间的点 =》 作为计算y轴平移量计算的根据，还可以作为z轴平移量的根据
~规定计算轨道的x轴方向以及x轴方向 =》 作为计算x轴旋转和z轴旋转的根据

***测量步骤***
1. 找到半径范围在（41~46）范围内的圆柱体 =》 得到轮缘轴的方向以及点的位置以及半径的大小
2. 找到车轮侧面的平面，距离较近的两个平面 =》 找到车轮中间的点
3.
*/
//using namespace Eigen;
//using Vec3 = Eigen::Matrix<double, 3, 1>;
//using Vec4 = Eigen::Matrix<double, 4, 1>;
//using Vec4_4 = Eigen::Matrix<double, 4, 4>;
//struct Shapes
//{
//	string shapeName;//形状名称
//	float shapeParams[10];//形状参数
//	int shapeNum;
//	int beginNum, endNum;
//};
//struct point {
//	float x, y, z;
//	point(float x, float y, float z) :x(x), y(y), z(z) {};
//};
//struct  plane
//{
//	float a, b, c, d;
//	float paray;
//};
void Measure::getFilesMeasure(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName)
{//循环加载每个文件的信息
	//文件句柄
	long long  hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				/*if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				getFiles(p.assign(path).append("\\").append(fileinfo.name), format, files, filesName);*/
			}
			else
			{
				//files.push_back(p.assign(path).append(fileinfo.name));
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
				filesName.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
void Measure::mygetFilesMeasure(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName)
{//循环加载每个文件的信息
	//文件句柄
	long long  hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				/*if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				getFiles(p.assign(path).append("\\").append(fileinfo.name), format, files, filesName);*/
			}
			else
			{
				//files.push_back(p.assign(path).append(fileinfo.name));
				files.push_back(p.assign(path).append(fileinfo.name));
				filesName.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
void Measure::splitStringMeasure(const string& str, float* res, const string& c) {//分割字符串
	int i = 0;
	string::size_type pos1, pos2;
	pos2 = str.find(c);
	pos1 = 0;

	while (string::npos != pos2) {
		string subs = str.substr(pos1, pos2 - pos1);
		stringstream iss(subs);
		float num;
		iss >> num;
		res[i] = num;
		i++;
		pos1 = pos2 + c.size();
		pos2 = str.find(c, pos1);
	}
	string sus = str.substr(pos1, str.size());
	stringstream iss(sus);
	iss >> res[i];
}
void Measure::getTxtDataMeasure(std::string file_path, int& shapeSize, int& pointSize, std::vector<Shapes>& shapes, std::vector<point>& points) {
	int begin = 0, end = 0;
	int shsSize = shapes.size();
	vector<int> nums;//保存 形状中点的个数
	std::fstream infile(file_path, std::ios::in);
	if (!infile)
	{
		std::cout << "error!" << endl;
		return;
	}
	std::string shapeDesc, shapeParam;
	string oneShapeSize;
	int shapeSizes;
	infile >> shapeSizes;
	if (shapeSizes == 0) {
		std::cout << "基元个数为0！" << endl;
		emit show_message("基元个数为0！");
		//ui.outputWindow->append("基元个数为0！");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		throw "基元个数为0！";
	}
	//形状描述：平面和圆柱
	infile >> shapeDesc;
	//形状参数
	infile >> shapeParam;
	while (!infile.eof() && shapeDesc != "#")
	{
		//参数处理
		float* params = new float[10]();
		Shapes sh;
		sh.shapeName = shapeDesc;

		splitStringMeasure(shapeParam, params, ",");
		if (shapeDesc == "Plane") {
			for (int p = 0; p < 6; p++)
				sh.shapeParams[p] = params[p];
		}
		else if (shapeDesc == "Cylinder") {
			for (int p = 0; p < 7; p++)
				sh.shapeParams[p] = params[p];
		}
		else if (shapeDesc == "Torus") {
			for (int p = 0; p < 8; p++) {
				sh.shapeParams[p] = params[p];
			}
		}
		// 形状中点的个数
		infile >> oneShapeSize;
		stringstream ss(oneShapeSize);
		int onss;
		ss >> onss;
		sh.shapeNum = onss;
		//更新形状中点的起始点的位置和结束点的位置
		shapeSize++;
		infile >> shapeDesc;
		infile >> shapeParam;
		shapes.push_back(sh);
		delete params;
	}
	stringstream ss(shapeParam);
	ss >> pointSize;
	end = pointSize;
	for (int i = shsSize; i < shapeSize; i++) {
		shapes[i].beginNum = end - shapes[i].shapeNum;
		shapes[i].endNum = end;
		end = shapes[i].beginNum;
	}
	while (!infile.eof())
	{
		float po[3];
		infile >> po[0] >> po[1] >> po[2];
		points.push_back(point(po[0], po[1], po[2]));
	}

	infile.close();
}
void Measure::getPcdData(std::string file_path, int& pointSize, std::vector<point>& points) {
	int i = 11;
	char ch[100];
	float x, y, z;
	std::fstream infile(file_path, std::ios::in);
	if (!infile)
	{
		std::cout << "error!" << endl;
		emit show_message("error!");
		//ui.outputWindow->append("error!");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		return;
	}
	while (i >= 0) {
		i--;
		infile.getline(ch, 100);
	}
	double temp[4];
	while (!infile.eof())
	{
		infile >> x >> y >> z >> temp[0] >> temp[1] >> temp[2] >> temp[3];
		points.push_back(point(x, y, z));
	}
	pointSize = points.size();
	infile.close();

}
void Measure::getTxtData(std::string file_path, int& shapeSize, int& pointSize, std::vector<Shapes>& shapes, std::vector<point>& points) {
	int begin = 0, end = 0;
	int shsSize = shapes.size();
	vector<int> nums;//保存 形状中点的个数
	std::fstream infile(file_path, std::ios::in);
	if (!infile)
	{
		emit show_message("error!");
		//ui.outputWindow->append("error!");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		std::cout << "error!" << endl;
		return;
	}
	std::string shapeDesc, shapeParam;
	string oneShapeSize;
	int shapeSizes;
	infile >> shapeSizes;
	if (shapeSizes == 0) {
		emit show_message("基元个数为0！");
		//ui.outputWindow->append("基元个数为0！");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		std::cout << "基元个数为0！" << endl;
		throw "基元个数为0！";
	}
	//形状描述：平面和圆柱
	infile >> shapeDesc;
	//形状参数
	infile >> shapeParam;
	while (!infile.eof() && shapeDesc != "#")
	{
		//参数处理
		float* params = new float[10]();
		Shapes sh;
		sh.shapeName = shapeDesc;

		splitStringMeasure(shapeParam, params, ",");
		if (shapeDesc == "Plane") {
			for (int p = 0; p < 6; p++)
				sh.shapeParams[p] = params[p];
		}
		else if (shapeDesc == "Cylinder") {
			for (int p = 0; p < 7; p++)
				sh.shapeParams[p] = params[p];
		}
		else if (shapeDesc == "Torus") {
			for (int p = 0; p < 8; p++) {
				sh.shapeParams[p] = params[p];
			}
		}
		// 形状中点的个数
		infile >> oneShapeSize;
		stringstream ss(oneShapeSize);
		int onss;
		ss >> onss;
		sh.shapeNum = onss;
		//更新形状中点的起始点的位置和结束点的位置
		shapeSize++;
		infile >> shapeDesc;
		infile >> shapeParam;
		shapes.push_back(sh);
		delete params;
	}
	stringstream ss(shapeParam);
	ss >> pointSize;
	end = pointSize;
	for (int i = shsSize; i < shapeSize; i++) {
		shapes[i].beginNum = end - shapes[i].shapeNum;
		shapes[i].endNum = end;
		end = shapes[i].beginNum;
	}
	while (!infile.eof())
	{
		float po[3];
		infile >> po[0] >> po[1] >> po[2];
		points.push_back(point(po[0], po[1], po[2]));
	}

	infile.close();
}
void Measure::getdata(std::string file_path, int& shapeSize, int& pointSize, std::vector<Shapes>& shapes, std::vector<point>& points)
{//从文件中获取各个信息，分别存储在shapesize pointsize shapes points 中
	std::string format = file_path.substr(file_path.size() - 3, file_path.size());
	if (format == "pcd") {
		getPcdData(file_path, pointSize, points);
	}
	else if (format == "txt") {
		getTxtData(file_path, shapeSize, pointSize, shapes, points);
	}

}
int Measure::sgn(float x) {//计算正负号
	return (0. < x) - (x < 0.);
}
bool Measure::getLunYuan(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, vector<Shapes>& sh, Eigen::Vector3f nor) {
	double ebslo = 0.015, dushuEbslo = 2;
	if (shapeSize == 0) {
		std::cout << "基元个数为0" << endl;
		emit show_message("基元个数为0");
		//ui.outputWindow->append("基元个数为0！");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		return false;
	}
	vector<Shapes> ssh;
	int size = 0;
	int floag = 0;
	for (int i = 0; i < shape.size(); i++) {
		if (shape[i].shapeName == "Torus") {
			Eigen::Vector3f norTemp = { shape[i].shapeParams[3],shape[i].shapeParams[4] ,shape[i].shapeParams[5] };
			//判断两个向量平行
			//计算向量夹角
			norTemp.normalize();
			nor.normalize();
			double tempres = norTemp.dot(nor);
			double dushu = std::acos(tempres) / M_PI * 180;
			if (dushu < dushuEbslo || (180 - dushu) < dushuEbslo) {
				if (shape[i].shapeParams[6] > 35 && shape[i].shapeParams[6] < 50) {
					sh.push_back(shape[i]);
				}
			}
		}
	}
	//sh.push_back(shape[floag]);
	if (sh.size())
		return true;
	else
		return false;
}
bool Measure::getMinCyl(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, Shapes& sh) {
	if (shapeSize == 0)
	{
		std::cout << "基元个数为0！" << endl;
		emit show_message("基元个数为0");
		//ui.outputWindow->append("基元个数为0！");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		return false;
	}
	Vec3 AxisDirection = { 0,0,0 }, AxisPosition = { 0,0,0 };
	double radius = 0, minRad = 100, tempRad;
	for (int i = 0; i < shapeSize; i++) {
		if (shape[i].shapeName == "Cylinder")
		{
			tempRad = shape[i].shapeParams[6];
			if (tempRad < minRad) {
				minRad = tempRad;
				sh = shape[i];
			}
		}
	}
	return true;
}
bool Measure::getMaxCyls(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, vector<Shapes>& shs)
{//获取基元中代表轮圆的基元及其参数
	if (shapeSize == 0) {
		std::cout << "基元个数为0" << endl;;
		return false;
	}
	//	AxisPosition:3xa
	//	AxisDirection : 3
	//	Radius : 1
	Vec3 AxisDirection = { 0,0,0 }, AxisPosition = { 0,0,0 };
	int maxCylNum = 0;
	float radTemp;
	Vec3 pos, nor;
	for (int i = 0; i < shapeSize; i++) {
		if (shape[i].shapeName == "Cylinder") {
			radTemp = shape[i].shapeParams[6];
			if (radTemp >= 41 && radTemp <= 45) {
				shs.push_back(shape[i]);
				maxCylNum++;
			}
		}
	}
	if (maxCylNum == 0) {
		emit show_message("没有找到合适的轮缘！");
		//ui.outputWindow->append("没有找到合适的轮缘！");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		std::cout << "没有找到合适的轮缘！" << endl;
		return false;
	}
	return true;
}
bool Measure::getMaxCyl(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, Shapes& sh)
{//获取基元中代表轮圆的基元及其参数
	if (shapeSize == 0) {
		std::cout << "基元个数为0" << endl;;
		return false;
	}
	//	AxisPosition:3
	//	AxisDirection : 3
	//	Radius : 1
	Vec3 AxisDirection = { 0,0,0 }, AxisPosition = { 0,0,0 };
	double Radius = 0, wei1 = 0, wei2, wei, wei_inv;
	bool weightFlag = true;//标志是否使用权重整合
	int maxCylNum = 0;
	vector<float> weights;
	vector<Shapes> shs;
	float radTemp;
	Vec3 pos, nor;
	for (int i = 0; i < shapeSize; i++) {
		if (shape[i].shapeName == "Cylinder") {
			radTemp = shape[i].shapeParams[6];
			if (weightFlag) {
				if (radTemp >= 41 && radTemp <= 45) {
					float cha = std::abs(radTemp - 43);
					weights.push_back(1. / cha);
					shs.push_back(shape[i]);
					maxCylNum++;
				}
			}
			else {
				pos = { shape[i].shapeParams[0],shape[i].shapeParams[1],shape[i].shapeParams[2] };
				nor = { shape[i].shapeParams[3],shape[i].shapeParams[4],shape[i].shapeParams[5] };
				if (radTemp >= 41 && radTemp <= 45) {
					maxCylNum++;
					AxisDirection = (AxisDirection + nor).normalized();
					AxisPosition = (AxisPosition + pos);
					Radius = (Radius + radTemp);
				}
				if (maxCylNum) {
					AxisDirection = AxisDirection / maxCylNum;
					AxisPosition = AxisPosition / maxCylNum;
					Radius = Radius / maxCylNum;
				}
			}

		}
	}
	float sum = 0, sum_inv;
	for (int i = 0; i < weights.size(); i++) {
		sum += weights[i];
	}
	sum_inv = 1. / sum;
	for (int i = 0; i < shs.size(); i++) {
		pos = { shs[i].shapeParams[0],shs[i].shapeParams[1],shs[i].shapeParams[2] };
		nor = { shs[i].shapeParams[3],shs[i].shapeParams[4],shs[i].shapeParams[5] };
		radTemp = shs[i].shapeParams[6];
		AxisDirection += weights[i] * nor * sum_inv;
		//cout << "AxisDirection:" << AxisDirection << endl;
		AxisPosition += weights[i] * pos * sum_inv;
		Radius += weights[i] * radTemp * sum_inv;
	}
	AxisDirection = AxisDirection.normalized();

	if (maxCylNum == 0) {
		std::cout << "没有找到合适的轮缘！" << endl;
		emit show_message("没有找到合适的轮缘！");
		//ui.outputWindow->append("没有找到合适的轮缘！");
		//ui.outputWindow->repaint();// 实时更新输出窗口信息
		return false;
	}
	else {
		sh.shapeName = "MaxCyl";
		sh.shapeParams[0] = AxisPosition[0];
		sh.shapeParams[1] = AxisPosition[1];
		sh.shapeParams[2] = AxisPosition[2];
		sh.shapeParams[3] = AxisDirection[0];
		sh.shapeParams[4] = AxisDirection[1];
		sh.shapeParams[5] = AxisDirection[2];
		sh.shapeParams[6] = Radius;
	}
	return true;
}
bool Measure::getOurPlane(vector<Shapes>& shape, Shapes& p1, Shapes& p2) {
	//获取轮子中间距离最近的平面
	/*
	Position :3
	Normal :3
	*/
	vector<Shapes> pls1, pls2;
	float a1, b1, c1, d1, a2, b2, c2, d2;
	Vec3 temp;
	for (int i = 0; i < shape.size(); i++) {
		a1 = shape[i].shapeParams[3];
		b1 = shape[i].shapeParams[4];
		c1 = shape[i].shapeParams[5];
		d1 = -a1 * shape[i].shapeParams[0] - b1 * shape[i].shapeParams[1] - c1 * shape[i].shapeParams[2];
		for (int j = 0; j < shape.size(); j++) {
			a2 = shape[j].shapeParams[3];
			b2 = shape[j].shapeParams[4];
			c2 = shape[j].shapeParams[5];
			if ((std::abs(a1) < 1e-2 && std::abs(a2) >= 1e-2) || (std::abs(a1) >= 1e-2 && std::abs(a2) < 1e-2))
			{
				continue;
			}
			if ((std::abs(b1) < 1e-2 && std::abs(b2) >= 1e-2) || (std::abs(b1) >= 1e-2 && std::abs(b2) < 1e-2))
			{
				continue;
			}
			if ((std::abs(c1) < 1e-2 && std::abs(c2) >= 1e-2) || (std::abs(c1) >= 1e-2 && std::abs(c2) < 1e-2))
			{
				continue;
			}

			if (a2 > 1e-2 && b2 > 1e-2 && c2 < 1e-2) {
				if (std::abs(a1 / a2 - b1 / b2) > 1e-3) {
					continue;
				}
			}

			if (a2 > 1e-2 && c2 > 1e-2 && b2 < 1e-2) {
				if (std::abs(a1 / a2 - c1 / c2) > 1e-3) {
					continue;
				}
			}
			if (c2 > 1e-2 && b2 > 1e-2 && a2 < 1e-2) {
				if (std::abs(b1 / b2 - c1 / c2) > 1e-3) {
					continue;
				}
			}

			if (c2 > 1e-2 && b2 > 1e-2 && a2 > 1e-2) {
				if (std::abs(a1 / a2 - b1 / b2) > 1e-3 || std::abs(a1 / a2 - b1 / b2) > 1e-3 || std::abs(c1 / c2 - b1 / b2) > 1e-3) {
					continue;
				}
			}

			d2 = -a2 * shape[j].shapeParams[0] - b2 * shape[j].shapeParams[1] - c2 * shape[j].shapeParams[2];
			float dd;
			if (b1 * b2 < 0) {
				dd = std::abs(d1 + d2);
			}
			else {
				dd = std::abs(d1 - d2);
			}

			temp = { a1, b1, c1 };
			float abc = std::sqrt(std::pow(temp[0], 2) + std::pow(temp[1], 2) + std::pow(temp[2], 2));
			float res = dd / abc;
			//std::cout << res << std::endl;
			if (std::abs(res - 135.3) < 1e-2) {
				/*std::cout << "i:" << i << "j:" << j << std::endl;
				std::cout << "GET IT!" << std::endl;*/
				p1 = shape[i];
				p2 = shape[j];
				return true;
			}
		}
	}
	return false;
}
void Measure::getBoxss(vector<point> points, float& diameter, vector<float>& box, vector<float>& boxCenter) {
	float min_x = std::numeric_limits<float>::max(), min_y = min_x, min_z = min_y;
	float max_x = std::numeric_limits<float>::lowest(), max_y = max_x, max_z = max_y;
	for (auto& p : points) {
		if (p.x < min_x) min_x = p.x;
		if (p.x > max_x) max_x = p.x;
		if (p.y < min_y) min_y = p.y;
		if (p.y > max_y) max_y = p.y;
		if (p.z < min_z) min_z = p.z;
		if (p.z > max_z) max_z = p.z;
	}
	const float dx = max_x - min_x;
	const float dy = max_y - min_y;
	const float dz = max_z - min_z;
	diameter = std::sqrt(dx * dx + dy * dy + dz * dz);

	box.push_back(min_x);
	box.push_back(max_x);
	box.push_back(min_y);
	box.push_back(max_y);
	box.push_back(min_z);
	box.push_back(max_z);

	boxCenter.push_back((min_x + max_x) / 2);
	boxCenter.push_back((min_y + max_y) / 2);
	boxCenter.push_back((min_z + max_z) / 2);
}
//预处理二
/*
获取x y z 轴
*/
void Measure::getXYZAxis(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points) {
	//获取最底下的两个平面
	float minz;
	map<float, Shapes> pls;
	Shapes botOne, botTwo;//最底下的两个平面
	for (int i = 0; i < shapeSize; i++) {
		if (shape[i].shapeName == "Plane") {
			pls[shape[i].shapeParams[2]] = shape[i];
		}
	}
	bool flagBot1 = false, flagBot2 = false;
	map<float, Shapes>::iterator iter;
	int i = 0;

	for (iter = pls.begin(); iter != pls.end(); iter++) {
		if (i > 5)//只考虑前五个
			break;
		Vec3 nor = { iter->second.shapeParams[3],iter->second.shapeParams[4], iter->second.shapeParams[5] };
		Vec3 norNor = { 0,0,1 };
		int sign = sgn(nor.dot(norNor));
		if (nor.dot(norNor) * sign >= 0.9 && nor.dot(norNor) * sign <= 1.1) {
			if (!flagBot1) {
				botOne = iter->second;
				flagBot1 = true;
			}
			else if (!flagBot2) {
				botTwo = iter->second;
				flagBot2 = true;
			}
		}
		i++;
	}
	Vec3 nor1 = { botOne.shapeParams[3], botOne.shapeParams[4], botOne.shapeParams[5] };
	Vec3 nor2 = { botTwo.shapeParams[3], botTwo.shapeParams[4], botTwo.shapeParams[5] };
	Vec3 x = nor1.cross(nor2).normalized();
	//std::cout << x << endl;

}

void Measure::getBottomCenter(int pointSize, int shapeSize, vector<Shapes>& shape, vector<point>& points, vector<float>& boxCenter1, vector<float>& boxCenter2) {
	//获取最底下的两个平面中心点
	float minz;
	map<float, Shapes> pls;
	Shapes botOne, botTwo;//最底下的两个平面
	for (int i = 0; i < shapeSize; i++) {
		if (shape[i].shapeName == "Plane") {
			pls[shape[i].shapeParams[2]] = shape[i];
		}
	}
	bool flagBot1 = false, flagBot2 = false;
	map<float, Shapes>::iterator iter;
	int i = 0;
	int c = 1;
	for (iter = pls.begin(); iter != pls.end(); iter++) {
		i++;
		Vec3 nor = { iter->second.shapeParams[3],iter->second.shapeParams[4], iter->second.shapeParams[5] };
		Vec3 norNor = { 0,0,1 };
		int sign = sgn(nor.dot(norNor));
		if (nor.dot(norNor) * sign >= 0.9 && nor.dot(norNor) * sign <= 1.1) {
			if (!flagBot1) {
				botOne = iter->second;
				flagBot1 = true;
				//cout << "bottom:" << i << endl;
			}
			else if (!flagBot2) {
				botTwo = iter->second;
				flagBot2 = true;
				/*cout << "bottom:" << i << endl;*/
			}
			else {
				break;
			}
		}
	}
	vector<point> po1, po2;

	for (int i = botOne.beginNum + 25; i < botOne.endNum - 25; i++) {
		po1.push_back(points[i]);
	}
	for (int i = botTwo.beginNum + 25; i < botTwo.endNum - 25; i++) {
		po2.push_back(points[i]);
	}
	float diameter1, diameter2;
	vector<float> box1, box2;
	getBoxss(po1, diameter1, box1, boxCenter1);
	getBoxss(po2, diameter2, box2, boxCenter2);

}

// =================================参数路径修改 此函数有待测试  因为起始点不可能全部为0-0-0-0.txt
bool Measure::measure_getBenchMark(map<string, vector<float>>& allResult, float& xRotInit, float& zRotInit, float& yTraInit, float& zTraInit, char* szFname, point& center, string allPath) {
	allPath = thePath;
	//获取第0-0-0-0的信息
	bool flag = true;
	float diameter;
	Shapes p1, p2;
	//point center(0, 0, 0);
	float dist;
	float xCenter, yCenter, zCenter;
	std::string filePath1 = allPath + "/middleResult/0-0-0-0.txt";
	//std::string filePath1 = allPath+"C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd_100000_3_middleResult\\0-0-0-0.txt";
	std::string filePath2 = allPath + "/torusResult/0-0-0-0.txt";
	//std::string filePath2 = "C:\\Users\\15281\\Desktop\\Qt\\数据1221\\torusResult\\0-0-0-0.txt";
	std::cout << "0-0-0-0" << endl;
	vector<float> box;
	vector<float> boxCenter1, boxCenter2;
	int pointSize = 0, shapeSize = 0;
	vector<Shapes> shapes;
	std::vector<point> points, points1;
	Shapes maxCylinders;
	getdata(filePath1, shapeSize, pointSize, shapes, points);
	getdata(filePath2, shapeSize, pointSize, shapes, points1);
	float deitaY, deitaZ;
	vector<Shapes> shs;

	//if (!getMaxCyl(pointSize, shapeSize, shapes, points, maxCylinders)) {
	//if(!getMinCyl(pointSize, shapeSize, shapes, points, maxCylinders)){
	//	std::cout << "寻找最大的轮圆出错！" << endl;
	//	return false;
	//}
	//Vec3 cylAxis = { maxCylinders.shapeParams[3],maxCylinders.shapeParams[4] ,maxCylinders.shapeParams[5] };
	if (!getOurPlane(shapes, p1, p2)) {

		std::cout << "获取轮子旁边的平面出错！" << endl;
		return false;
	}

	Eigen::Vector3f nor = { p1.shapeParams[3],p1.shapeParams[4],p1.shapeParams[5] };
	//获取轮缘
	if (!getLunYuan(pointSize, shapeSize, shapes, points, shs, nor)) {
		std::cout << "没有搜寻到合适的轮缘" << endl;
		return false;
	}
	Shapes torusShape = shs[0];
	//计算横移量
	//计算轮轴方向与轮子旁边平面的交点。然后求中点
	Vec3 leftp, rightp;
	float t1 = ((p1.shapeParams[0] - torusShape.shapeParams[0]) * p1.shapeParams[3] + (p1.shapeParams[1] - torusShape.shapeParams[1]) * p1.shapeParams[4] + (p1.shapeParams[2] - torusShape.shapeParams[2]) * p1.shapeParams[5]) /
		(p1.shapeParams[3] * p1.shapeParams[3] + p1.shapeParams[4] * p1.shapeParams[4] + p1.shapeParams[5] * p1.shapeParams[5]);
	float t2 = ((p2.shapeParams[0] - torusShape.shapeParams[0]) * p2.shapeParams[3] + (p2.shapeParams[1] - torusShape.shapeParams[1]) * p2.shapeParams[4] + (p2.shapeParams[2] - torusShape.shapeParams[2]) * p2.shapeParams[5]) /
		(p2.shapeParams[3] * p2.shapeParams[3] + p2.shapeParams[4] * p2.shapeParams[4] + p2.shapeParams[5] * p2.shapeParams[5]);
	leftp = { p1.shapeParams[3] * t1 + torusShape.shapeParams[0],
		p1.shapeParams[4] * t1 + torusShape.shapeParams[1],
		p1.shapeParams[5] * t1 + torusShape.shapeParams[2]
	};
	rightp = { p2.shapeParams[3] * t2 + torusShape.shapeParams[0],
		p2.shapeParams[4] * t2 + torusShape.shapeParams[1],
		p2.shapeParams[5] * t2 + torusShape.shapeParams[2]
	};
	/*cout << leftp << endl;
	cout << rightp << endl;*/
	center.x = (leftp[0] + rightp[0]) / 2;
	center.y = (leftp[1] + rightp[1]) / 2;
	center.z = (leftp[2] + rightp[2]) / 2;
	getBottomCenter(pointSize, shapeSize, shapes, points, boxCenter1, boxCenter2);
	xCenter = (boxCenter1[0] + boxCenter2[0]) / 2;
	yCenter = (boxCenter1[1] + boxCenter2[1]) / 2;
	zCenter = (boxCenter1[2] + boxCenter2[2]) / 2;

	deitaY = std::fabs(yCenter - center.y);
	deitaZ = std::fabs(zCenter - center.z);
	//计算旋转角
	float xRotate_rad, xRotate_ang, zRotate_rad, zRotate_ang;
	//z轴旋转角为轮轴方向垂直方向与x轴方向的夹角，在这里可以转换为轮轴方向与yoz面夹角
	//cout << "zrotate:" << std::abs(cylAxis[0]) / std::sqrt(std::pow(cylAxis[0], 2) + std::pow(cylAxis[1], 2) + std::pow(cylAxis[2], 2)) << endl;
	zRotate_rad = std::asin(std::abs(p1.shapeParams[3]) / std::sqrt(std::pow(p1.shapeParams[3], 2) + std::pow(p1.shapeParams[4], 2) + std::pow(p1.shapeParams[5], 2)));
	zRotate_ang = zRotate_rad / M_PI * 180;
	Vec4 nor_ = { p1.shapeParams[3], p1.shapeParams[4], p1.shapeParams[5], 1 };
	Vec4_4 xuanMatrix;
	xuanMatrix << std::cos(zRotate_rad), std::sin(zRotate_rad), 0, 0,
		-std::sin(zRotate_rad), std::cos(zRotate_rad), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	nor_ = xuanMatrix * nor_;
	//x旋转角为轮轴方向与xoy面夹角

	xRotate_rad = std::asin(std::abs(nor_[2]) / std::sqrt(std::pow(nor_[0], 2) + std::pow(nor_[1], 2) + std::pow(nor_[2], 2)));
	xRotate_ang = xRotate_rad / M_PI * 180;
	if (xRotate_ang > 25) {
		xRotate_ang = 90 - xRotate_ang;
	}
	if (zRotate_ang > 25) {
		zRotate_ang = 90 - zRotate_ang;
	}


	yTraInit = deitaY;
	zTraInit = deitaZ;
	std::cout << "Y横移量：" << deitaY << endl;

	std::cout << "Z横移量：" << deitaZ << endl;
	return true;
}
// =================================参数路径修改


int  Measure::startMeasure(string AlloutPath)
{
	AlloutPath = "C:/Users/15281/Desktop/FinalVersionOne/out";
	int count = 40;
	string myfilePath1 = AlloutPath + "/torusResult/";
	std::vector<std::string> myfiles1;
	std::vector<std::string> myfilesName1;
	std::string myformat1 = "*.txt";
	mygetFilesMeasure(myfilePath1, myformat1, myfiles1, myfilesName1);
	int mysize = myfiles1.size();
	emit show_message("现在展示圆环基元检测结果");
	for (int i = 0; i < mysize; i++)
	{
		emit show_message(myfilesName1[i]);
		ifstream inFile(myfiles1[i]);
		string str;
		while (inFile.good()) {
			getline(inFile, str);
			emit show_message(str);
			if (str == "#")break;
		}

		if (count <= 85)
		{
			count = count + 7;
			emit changMeasureProgessBar(count);
		}

	}
	thePath = AlloutPath;

	//emit changMeasureProgessBar(count);//更改进度条
	//===============参数路径修改
	std::string outErrorPath = AlloutPath + "/MesuareErrorLog.txt";
	//std::string outErrorPath = "C:\\Users\\15281\\Desktop\\Qt\\数据1221\\测量errorLog.txt";
	std::fstream outErrorfile(outErrorPath, std::ios::out);

	std::string outResultPath = AlloutPath + "/MesuareResult.txt";
	//std::string outResultPath = "C:\\Users\\15281\\Desktop\\Qt\\数据1221\\测量Result.txt";
	std::fstream outResultfile(outResultPath, std::ios::out);
	//testBox();

	//轮轨数据平面基元和圆柱基元检测结果
	//std::string filePath = "F:\\baixinyu\\program\\result\\unionResult\\pcd_100000_3_middleResult\\";
	std::string filePath = AlloutPath + "/middleResult/";
	//std::string filePath = "C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd_100000_3_middleResult\\";
	//"F:\\暂时存储\\轮轨数据集（新）\\unionResult\\pcd_100000_3_middleResult\\";
	std::vector<std::string> files;
	std::vector<std::string> filesName;
	std::string format = "*.txt";
	getFilesMeasure(filePath, format, files, filesName);
	//轮缘部分的圆环基元检测结果。
	std::string filePath1 = AlloutPath + "/torusResult/";
	//std::string filePath1 = "C:\\Users\\15281\\Desktop\\Qt\\数据1221\\torusResult\\";
	//std::string filePath1 = "F:\\baixinyu\\program\\result\\unionResult\\torusResult\\";
	std::vector<std::string> files1;
	std::vector<std::string> filesName1;
	std::string format1 = "*.txt";
	getFilesMeasure(filePath1, format1, files1, filesName1);

	int size = files.size(), size1 = files1.size();
	map<string, vector<float>> allResult;
	char szDrive[_MAX_DRIVE];   //磁盘名
	char szDir[_MAX_DIR];       //路径名
	char szFname[_MAX_FNAME];   //文件名
	char szExt[_MAX_EXT];       //后缀名

	Shapes p1, p2;
	point center(0, 0, 0);
	float xCenter, yCenter, zCenter;
	int falseNum1 = 0, falseNum2 = 0;
	float xRotInit, zRotInit, yTraInit, zTraInit;
	bool rotFlag = true, hengyiFlag = true;
	point centerResult(0, 0, 0);

	bool flag = measure_getBenchMark(allResult, xRotInit, zRotInit, yTraInit, zTraInit, szFname, centerResult, thePath);
	if (!flag) {
		outErrorfile << "0-0-0-0信息出错" << std::endl;
		std::cout << "第一个就出错啦，根本就进行不下去啦！STOPPPPP!!!" << endl;
		return 0;
	}


	//获取除0-0-0-0之外的数据的信息
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size1; j++) {
			if (filesName[i] == filesName1[j]) {
				//	emit show_message(filesName[i]);
				emit show_MeaureResultmessage(filesName[i]);
				if (i == 0)
				{
					outResultfile << "初始Y横移量坐标差为：：" << std::fabs(yTraInit) * 10 << ";";
					outResultfile << "初始Z横移量坐标差为：" << std::fabs(zTraInit) * 10 << ";";
					//emit show_message("初始Y横移量坐标差为：" + to_string(yTraInit));
					//emit show_message("初始Z横移量坐标差为：" + to_string(zTraInit));
					emit show_MeaureResultmessage("初始Y横移量坐标差为：" + to_string(yTraInit));
					emit show_MeaureResultmessage("初始Z横移量坐标差为：" + to_string(zTraInit));
				}
				outResultfile << filesName[i] << std::endl;
				float deitaY, deitaZ;
				vector<float> box;
				vector<float> boxCenter1, boxCenter2;
				int pointSize = 0, shapeSize = 0;
				vector<Shapes> shapes, shs;
				std::vector<point> points, points1;
				Shapes maxCylinders;
				_splitpath(files[i].c_str(), szDrive, szDir, szFname, szExt);
				if (string(szFname) == "0-0-0-0") {
					continue;
				}

				std::cout << filesName[i] << std::endl;
				getdata(files[i], shapeSize, pointSize, shapes, points);
				getdata(files1[j], shapeSize, pointSize, shapes, points1);

				if (!getOurPlane(shapes, p1, p2)) {
					outErrorfile << filesName[i] << "获取轮子旁边的平面出错" << std::endl;
					std::cout << "获取轮子旁边的平面出错！" << endl;
					//ui.outputWindow->append("获取轮子旁边的平面出错");
					//ui.outputWindow->repaint();// 实时更新输出窗口信息
					deitaY = 0;
					deitaZ = 0;
					hengyiFlag = false;
					falseNum2++;
				}
				else
				{

					/*Vec3 cylAxis = { maxCylinders.shapeParams[3],maxCylinders.shapeParams[4] ,maxCylinders.shapeParams[5] };*/
					float xRotate_rad, xRotate_ang, zRotate_rad, zRotate_ang;

					//计算平移量
					Eigen::Vector3f nor = { p1.shapeParams[3],p1.shapeParams[4],p1.shapeParams[5] };
					if (!getLunYuan(pointSize, shapeSize, shapes, points, shs, nor)) {
						outErrorfile << filesName[i] << "寻找轮缘出错" << std::endl;
						std::cout << "寻找轮缘出错！" << endl;
						//ui.outputWindow->append("寻找轮缘出错！");
						//ui.outputWindow->repaint();// 实时更新输出窗口信息
						rotFlag = false;
						falseNum1++;
					}
					else {
						Shapes torusShape, torusShape1;
						if (shs.size() >= 2) {
							torusShape = shs[0];
							torusShape1 = shs[1];

						}
						else if (shs.size() == 1) {
							torusShape = shs[0];
						}

						//计算轮轴方向与轮子旁边平面的交点。然后求中点
						Vec3 leftp, rightp;
						float t1 = ((p1.shapeParams[0] - torusShape.shapeParams[0]) * p1.shapeParams[3] + (p1.shapeParams[1] - torusShape.shapeParams[1]) * p1.shapeParams[4] + (p1.shapeParams[2] - torusShape.shapeParams[2]) * p1.shapeParams[5]) /
							(p1.shapeParams[3] * p1.shapeParams[3] + p1.shapeParams[4] * p1.shapeParams[4] + p1.shapeParams[5] * p1.shapeParams[5]);
						float t2 = ((p2.shapeParams[0] - torusShape.shapeParams[0]) * p2.shapeParams[3] + (p2.shapeParams[1] - torusShape.shapeParams[1]) * p2.shapeParams[4] + (p2.shapeParams[2] - torusShape.shapeParams[2]) * p2.shapeParams[5]) /
							(p2.shapeParams[3] * p2.shapeParams[3] + p2.shapeParams[4] * p2.shapeParams[4] + p2.shapeParams[5] * p2.shapeParams[5]);
						leftp = { p1.shapeParams[3] * t1 + torusShape.shapeParams[0],
							p1.shapeParams[4] * t1 + torusShape.shapeParams[1],
							p1.shapeParams[5] * t1 + torusShape.shapeParams[2]
						};
						rightp = { p2.shapeParams[3] * t2 + torusShape.shapeParams[0],
							p2.shapeParams[4] * t2 + torusShape.shapeParams[1],
							p2.shapeParams[5] * t2 + torusShape.shapeParams[2]
						};
						/*std::cout << "leftp:" << leftp[0] << "," << leftp[1] << "," << leftp[2] << endl;
						std::cout << "rightp:" << rightp[0] << "," << rightp[1] << "," << rightp[2] << endl;*/


						center.x = (leftp[0] + rightp[0]) / 2;
						center.y = (leftp[1] + rightp[1]) / 2;
						center.z = (leftp[2] + rightp[2]) / 2;

						//center.z = (torusShape.shapeParams[2] + torusShape1.shapeParams[2]) / 2;


						getBottomCenter(pointSize, shapeSize, shapes, points, boxCenter1, boxCenter2);
						/*std::cout << "boxcenter1:" << boxCenter1[0] << "," << boxCenter1[1] << "," << boxCenter1[2] << endl;
						std::cout << "boxcenter2:" << boxCenter2[0] << "," << boxCenter2[1] << "," << boxCenter2[2] << endl;*/
						xCenter = (boxCenter1[0] + boxCenter2[0]) / 2;
						yCenter = (boxCenter1[1] + boxCenter2[1]) / 2;
						zCenter = (boxCenter1[2] + boxCenter2[2]) / 2;
						/*std::cout << "center上" << center.x << " " << center.y << " " << center.z << std::endl;
						std::cout << "center下" << xCenter << " " << yCenter << " " << zCenter << std::endl;*/

						/*deitaY = std::fabs(yCenter - center.y);
						deitaZ = std::fabs(zCenter - center.z);*/
						//deitaY = std::fabs(centerResult.y - center.y);
						deitaY = std::fabs(center.y - yCenter - yTraInit);
						cout << center.z - zCenter << endl;
						deitaZ = std::fabs(center.z - zCenter - zTraInit);
					}
					/*cout << maxCylinders.shapeParams[0] << "," << maxCylinders.shapeParams[1] << "," << maxCylinders.shapeParams[2] <<
					"," << maxCylinders.shapeParams[3] << "," << maxCylinders.shapeParams[4] << "," << maxCylinders.shapeParams[5]<<endl;*/
					//计算旋转角
					zRotate_rad = std::asin(std::abs(nor[0]) / std::sqrt(std::pow(nor[0], 2) + std::pow(nor[1], 2) + std::pow(nor[2], 2)));
					zRotate_ang = zRotate_rad / M_PI * 180;
					Vec4 nor_ = { nor[0], nor[1], nor[2], 1 };
					Vec4_4 xuanMatrix;;
					xuanMatrix << std::cos(zRotate_rad), std::sin(zRotate_rad), 0, 0,
						-std::sin(zRotate_rad), std::cos(zRotate_rad), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;
					//nor_ = xuanMatrix*nor_;
					xRotate_rad = std::asin(std::abs(nor_[2]) / std::sqrt(std::pow(nor_[0], 2) + std::pow(nor_[1], 2) + std::pow(nor_[2], 2)));
					xRotate_ang = xRotate_rad / M_PI * 180;
					if (xRotate_ang > 25) {
						xRotate_ang = 90 - xRotate_ang;
					}
					if (zRotate_ang > 25) {
						zRotate_ang = 90 - zRotate_ang;
					}

					//保存
					if (hengyiFlag && rotFlag) {
						vector<float> temp2;
						temp2.push_back(xRotate_ang);
						temp2.push_back(zRotate_ang);
						temp2.push_back((deitaY) * 10);
						temp2.push_back((deitaZ) * 10);
						allResult[szFname] = temp2;
						/*cout << "原来的x旋转角为：" << xRotate_ang << endl;
						cout << "原来的z旋转角为：" << zRotate_ang << endl;
						cout << "原来的y横移量为：" << deitaY << endl;
						cout << "原来的z横移量为：" << deitaZ << endl;*/


						outResultfile << "Y横移量：" << std::fabs(deitaY) * 10 << ";";
						outResultfile << "Z横移量：" << std::fabs(deitaZ) * 10 << ";";

						outResultfile << "X旋转角：" << std::fabs(xRotate_ang) << ";";
						outResultfile << "Z旋转角：" << std::fabs(zRotate_ang) << endl;

						std::cout << "Y横移量：" << std::fabs(deitaY) * 10 << endl;
						string Y = "Y横移量：" + to_string(std::fabs(deitaY) * 10);
						//	emit show_message(Y);
						emit show_MeaureResultmessage(Y);
						//ui.outputWindow->append(QString::fromStdString(Y));
						//ui.outputWindow->repaint();// 实时更新输出窗口信息


						std::cout << "Z横移量：" << std::fabs(deitaZ) * 10 << endl;
						string Z = "Z横移量：" + to_string(std::fabs(deitaZ) * 10);
						//	emit show_message(Z);
						emit show_MeaureResultmessage(Z);
						std::cout << "X旋转角：" << std::fabs(xRotate_ang) << endl;
						string xRotate = "X旋转角：" + to_string(std::fabs(xRotate_ang));
						//	emit show_message(xRotate);
						emit show_MeaureResultmessage(xRotate);
						//ui.outputWindow->append("中体育场卢萨卡魂斗罗哈佛赛杜甫");
						//ui.outputWindow->repaint();// 实时更新输出窗口信息
						//ui.outputWindow->append(QString::fromStdString(xRotate));
						//ui.outputWindow->repaint();// 实时更新输出窗口信息

						std::cout << "Z旋转角：" << std::fabs(zRotate_ang) << endl;
						string zRotate = "Z旋转角：" + to_string(std::fabs(zRotate_ang));
						//	emit show_message(zRotate);
						emit show_MeaureResultmessage(zRotate);
						//ui.outputWindow->append(QString::fromStdString(zRotate));
						//ui.outputWindow->repaint();// 实时更新输出窗口信息


						//ui.outputWindow->append(QString::fromStdString(Z));
						//ui.outputWindow->repaint();// 实时更新输出窗口信息

						std::cout << "done" << endl;
						//emit show_message("done");
						emit show_MeaureResultmessage("done");
						//ui.outputWindow->append("done");
						//ui.outputWindow->repaint();// 实时更新输出窗口信息
					}
					hengyiFlag = true;
					rotFlag = true;
				}
			}

		}
		if (i % 100 == 0 && count <= 95)
		{
			count++;
			emit changMeasureProgessBar(count);
		}
	}
	map<string, vector<float>>::iterator iter;
	float wc[4] = { 0,0,0,0 };
	float wc1[4] = { 0,0,0,0 };
	int yHYhege = 0, zHYhege = 0, xXZhege = 0, zXZhege = 0;
	int yHYhege1 = 0, zHYhege1 = 0, xXZhege1 = 0, zXZhege1 = 0;
	int hegeshu = 0;
	int hegeshusum[4] = { 0,0,0,0 };
	for (iter = allResult.begin(); iter != allResult.end(); iter++) {
		float gt[4];
		float yc1, yc2, yc3, yc4;
		//bool f1=false, f2=false, f3 = false, f4 = false;

		splitStringMeasure(iter->first, gt, "-");
		yc1 = std::fabs(iter->second[2] - gt[0]);
		yc2 = std::fabs(iter->second[3] - gt[1]);
		yc3 = std::fabs(iter->second[0] - gt[2]);
		yc4 = std::fabs(iter->second[1] - gt[3]);
		if (yc1 <= 0.5) {
			wc[0] += yc1;
			yHYhege++;
			hegeshu++;
		}
		if (yc1 > 0.5 && yc1 < 2) {
			wc[0] += yc1;
			yHYhege1++;
		}
		/*else
			cout << iter->first << endl;*/
		if (yc2 < 0.5) {
			wc[1] += yc2;
			zHYhege++;
			hegeshu++;
		}
		if (yc2 > 0.5 && yc2 < 2) {
			wc1[1] += yc2;
			zHYhege1++;
		}

		if (yc3 < 0.5) {
			wc[2] += yc3;
			xXZhege++;
			hegeshu++;
		}
		if (yc3 > 0.5 && yc3 < 2) {
			wc1[2] += yc3;
			xXZhege1++;
		}

		if (yc4 < 0.5) {
			wc[3] += yc4;
			zXZhege++;
			hegeshu++;
		}
		if (yc4 > 0.5 && yc4 < 2) {
			wc1[3] += yc4;
			zXZhege1++;
		}
		if (hegeshu == 4) {
			hegeshusum[0]++;
			hegeshusum[1]++;
			hegeshusum[2]++;
			hegeshusum[3]++;
		}
		else if (hegeshu == 3) {
			hegeshusum[0]++;
			hegeshusum[1]++;
			hegeshusum[2]++;
		}
		else if (hegeshu == 2) {
			hegeshusum[0]++;
			hegeshusum[1]++;
		}
		else {
			hegeshusum[0]++;
		}

		hegeshu = 0;
	}
	wc[0] /= yHYhege;
	wc[1] /= zHYhege;
	wc[2] /= xXZhege;
	wc[3] /= zXZhege;
	wc1[0] /= yHYhege1;
	wc1[1] /= zHYhege1;
	wc1[2] /= xXZhege1;
	wc1[3] /= zXZhege1;
	emit changMeasureProgessBar(97);
	emit show_MeaureResultmessage("--------------------------");
	std::cout << "寻找轮缘出错个数：" << falseNum1 << endl;
	string Z = "寻找轮缘出错个数：" + to_string(falseNum1);
	//emit show_message(Z);
	//ui.outputWindow->append(QString::fromStdString(Z));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "寻找平面出错个数：" << falseNum2 << endl;
	string Z1 = "寻找平面出错个数：" + to_string(falseNum2);
	//emit show_message(Z1);
	//ui.outputWindow->append(QString::fromStdString(Z1));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	emit changMeasureProgessBar(98);
	std::cout << "共计算" << size << "个数据" << endl;
	string Z2 = "共计算 " + to_string(size) + "个数据";
	emit show_MeaureResultmessage(Z2);
	//ui.outputWindow->append(QString::fromStdString(Z2));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "y横移（0，0.5）：" << wc[0] << endl;
	string Z3 = "y横移（0，0.5）：" + to_string(wc[0]);
	//emit show_MeaureResultmessage(Z3);
	//ui.outputWindow->append(QString::fromStdString(Z3));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z横移（0，0.5）：" << wc[1] << endl;
	string Z4 = "z横移（0，0.5）：" + to_string(wc[1]);
	//emit show_MeaureResultmessage(Z4);
	//ui.outputWindow->append(QString::fromStdString(Z4));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "x旋转（0，0.5）：" << wc[2] << endl;
	string Z5 = "x旋转（0，0.5）：" + to_string(wc[2]);
	//emit show_MeaureResultmessage(Z5);
	//ui.outputWindow->append(QString::fromStdString(Z5));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z旋转（0，0.5）：" << wc[3] << endl;
	string Z6 = "z旋转（0，0.5）：" + to_string(wc[3]);
	//emit show_MeaureResultmessage(Z6);
	//ui.outputWindow->append(QString::fromStdString(Z6));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "y横移（0.5，2）：" << wc1[0] << endl;
	string Z7 = "y横移（0.5，2）：" + to_string(wc1[0]);
	//emit show_MeaureResultmessage(Z7);
	//ui.outputWindow->append(QString::fromStdString(Z7));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z横移（0.5，2）：" << wc1[1] << endl;
	string Z8 = "z横移（0.5，2）：" + to_string(wc1[1]);
	//emit show_MeaureResultmessage(Z8);
	//ui.outputWindow->append(QString::fromStdString(Z8));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "x旋转（0.5，2）：" << wc1[2] << endl;
	string Z9 = "x旋转（0.5，2）：" + to_string(wc1[2]);
	//emit show_MeaureResultmessage(Z9);
	//ui.outputWindow->append(QString::fromStdString(Z9));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z旋转（0.5，2）：" << wc1[3] << endl;
	string Z10 = "z旋转（0.5，2）：" + to_string(wc1[3]);
	//emit show_MeaureResultmessage(Z10);
	//ui.outputWindow->append(QString::fromStdString(Z10));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "y横移合格数(0,0.5)：" << yHYhege << endl;
	string Z11 = "y横移合格数(0,0.5)：" + to_string(yHYhege);
	emit show_MeaureResultmessage(Z11);
	//ui.outputWindow->append(QString::fromStdString(Z11));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z横移合格数(0,0.5)：" << zHYhege << endl;
	string Z12 = "z横移合格数(0,0.5)：" + to_string(zHYhege);
	emit show_MeaureResultmessage(Z12);
	//ui.outputWindow->append(QString::fromStdString(Z12));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "x旋转合格数(0,0.5)：" << xXZhege << endl;
	string Z13 = "x旋转合格数(0,0.5)：" + to_string(xXZhege);
	emit show_MeaureResultmessage(Z13);
	//ui.outputWindow->append(QString::fromStdString(Z13));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z旋转合格数(0,0.5)：" << zXZhege << endl;
	string Z14 = "z旋转合格数(0,0.5)：" + to_string(zXZhege);
	//std::cout << "z旋转合格数(0,0.5)：" << xXZhege << endl;
	//string Z14 = "z旋转合格数(0,0.5)：" + to_string(xXZhege); 
	emit show_MeaureResultmessage(Z14);
	//ui.outputWindow->append(QString::fromStdString(Z14));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "y横移合格数(0.5,2)：" << yHYhege1 << endl;
	string Z15 = "y横移合格数(0.5,2)：" + to_string(yHYhege1);
	//emit show_MeaureResultmessage(Z15);
	//ui.outputWindow->append(QString::fromStdString(Z15));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z横移合格数(0.5,2)：" << zHYhege1 << endl;
	string Z16 = "z横移合格数(0.5,2)：" + to_string(zHYhege1);
	//emit show_MeaureResultmessage(Z16);
	//ui.outputWindow->append(QString::fromStdString(Z16));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "x旋转合格数(0.5,2)：" << xXZhege1 << endl;
	string Z17 = "x旋转合格数(0.5,2)：" + to_string(xXZhege1);
	//emit show_MeaureResultmessage(Z17);
	//ui.outputWindow->append(QString::fromStdString(Z17));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	std::cout << "z旋转合格数(0.5,2)：" << zXZhege1 << endl;
	string Z18 = "z旋转合格数(0.5,2)：" + to_string(zXZhege1);
	//emit show_MeaureResultmessage(Z18);
	//ui.outputWindow->append(QString::fromStdString(Z18));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	//emit changMeasureProgessBar(99);
	std::cout << "总共合格数：" << hegeshusum[0] << " " << hegeshusum[1] << " " << hegeshusum[2] << " " << hegeshusum[3] << endl;
	string Z19 = "总共合格数：" + to_string(yHYhege) + " " + to_string(zHYhege) + " " + to_string(xXZhege) + " " + to_string(zXZhege1);
	//string Z19 = "总共合格数：" + to_string(hegeshusum[0]) + " " + to_string(hegeshusum[1]) + " " + to_string(hegeshusum[2]) + " " + to_string(hegeshusum[3]);
	emit show_MeaureResultmessage(Z19);
	emit changMeasureProgessBar(99);
	//ui.outputWindow->append(QString::fromStdString(Z19));
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	/*emit sendMeasureThreadMessage();*/
	return 0;
}

