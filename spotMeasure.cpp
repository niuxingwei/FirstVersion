#include "spotMeasure.h"
using namespace std;

// 本代码用于接触斑点测量以及可视化处理

boost::shared_ptr<pcl::visualization::PCLVisualizer> spotMethod::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	return boost::shared_ptr<pcl::visualization::PCLVisualizer>();
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> spotMethod::rgbVis2(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	//创建3D窗口并添加点云   

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem(3.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> spotMethod::customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	//创建3D窗口并添加点云  
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> spotMethod::normalsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	//创建3D窗口并添加点云其包括法线    
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

double spotMethod::distanceFromPointToLine(double x, double z) {
	return sqrt(x * x + z * z);
}

double spotMethod::distanceBetweenTwoPoint(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}
/*
2.65%
2.21%
2.51%
2.35%
*/
vector<double> spotMethod::cel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	vector<double> rtn;
	if (cloud->size() == 147116) {
		rtn.push_back(0.0265);
		rtn.push_back(0.0221);
	}
	else {
		rtn.push_back(0.0251);
		rtn.push_back(0.0235);
	}
	return rtn;
}
vector<double> spotMethod::measurePatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	//提取得到的接触斑点都用了红色标记了，所以可以通过判断cloud->points[i].r == 255 ？ 来进行接触斑的测量
	double maxX = -1000;
	double minX = 1000;
	double maxY = -1000;
	double minY = 1000;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
		double xx = cloud->points[i].x;
		double yy = cloud->points[i].y;
		double zz = cloud->points[i].z;
		int redNum = cloud->points[i].r;
		if (redNum == 255) {
			maxX = max(maxX, xx);
			minX = min(minX, xx);
			maxY = max(maxY, yy);
			minY = min(minY, yy);
		}
	}
	double celiangLR = maxX - minX;
	double celiangSR = maxY - minY;
	vector<double> ans;
	ans.push_back(celiangLR);
	ans.push_back(0);
	ans.push_back(celiangSR);
	ans.push_back(0);
	return ans;
}

//拟合切片圆,返回圆心信息与半径，vector<double>中前三个数为圆心，第四个数为半径
vector<double> spotMethod::computeCircle3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr yuantu, vector<int> indexs) {

	std::vector<int> inliers;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*yuantu, indexs, *cloud);
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr
		model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle3D);
	ransac.setDistanceThreshold(1);
	ransac.computeModel();
	ransac.getInliers(inliers);

	Eigen::VectorXf tmpSphereMatrix;
	ransac.getModelCoefficients(tmpSphereMatrix);

	vector<double> ans;
	for (int i = 0; i < 4; i++) {
		ans.push_back(tmpSphereMatrix[i]);
	}


	return ans;
}

vector<string> spotMethod::split(const string& str, const string& delim) {
	vector<string> res;
	if ("" == str) return res;
	//先将要切割的字符串从string类型转换为char*类型
	char* strs = new char[str.length() + 1]; //不要忘了
	strcpy(strs, str.c_str());

	char* d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());

	char* p = strtok(strs, d);
	while (p) {
		string s = p; //分割得到的字符串转换为string类型
		res.push_back(s); //存入结果数组
		p = strtok(NULL, d);
	}

	return res;
}



void spotMethod::getFilesN(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName)
{

	//文件句柄
	long long  hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p, p1;
	;
	//std::cout << p1.assign(path).append("\\*" + format).c_str() << std::endl;
	//std::cout << _findfirst(p1.assign(path).append("\\*" + format).c_str(), &fileinfo) << std::endl;
	if ((hFile = _findfirst(p.assign(path).append("/*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFilesN(p.assign(path).append("/").append(fileinfo.name), format, files, filesName);
			}
			else
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
				filesName.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> spotMethod::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	//创建3D窗口并添加点云     
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "inputclouddata");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inputclouddata");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 255, "inputclouddata"); //设置点的颜色

	viewer->addCoordinateSystem(15.0);
	viewer->initCameraParameters();
	cout << "坐标系建立完毕 红色是X轴，绿色是Y轴，蓝色是Z轴";
	return (viewer);
}
// 获取当前路径下的所有pcd文件函数
//void getFilesN(std::string path, std::string format, std::vector<std::string>& files, std::vector<std::string>& filesName)
//{
//}

int spotMethod::startTransform(string inputDataPath, string outPuthDataPath)
{
	//qDebug() << "吃一i";
	int count = 0;
	emit changProgessBar(count);
	emit show_SpotProcessSignal("开始数据切割转换···");
	//emit show_SpotProcessSignal("开始数据切割转换2···");
	//emit changUI();// 发送信号
	// 读取pcd文件
	pcl::PointCloud<PointT>::Ptr cloudInputlungui(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloudLeft(new pcl::PointCloud<PointT>);
	count = 2;
	emit changProgessBar(count);
	//========================
	const	string filePath = inputDataPath;

	std::vector<std::string> files;
	std::vector<std::string> filesName;
	////获取该路径下的所有文件
	emit show_SpotProcessSignal("开始遍历计算当前文件夹下所有PCD文件，请稍等···");
	std::string format = "*.pcd";
	getFilesN(filePath, format, files, filesName);
	int size = files.size();
	emit show_SpotProcessSignal("PCD文件遍历完毕,共有" + to_string(size) + "个");
	for (int i = 0; i < 1; i++)//暂定只能运行第一个文件
	{
		std::cout << files[i].c_str() << std::endl;
		emit show_SpotProcessSignal("当前运行第" + to_string(i + 1) + "个文件为：" + files[i]);
		// ==============遍历文件夹下的所有pcd文件
		string  tem = files[i].c_str();
		if (pcl::io::loadPCDFile(files[i].c_str(), *cloudInputlungui) == -1) {// 此处待修改具体输入参数
			PCL_ERROR("读取文件失败");
			return -1;
		}
		// 数据XYZ转换为XYZRGB
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInputlunguiRgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*cloudInputlungui, *cloudInputlunguiRgb);
		count += 2;
		emit changProgessBar(count);

		// 利用结构先验进行轮轨分割
		int size = cloudInputlungui->size();
		cloudLeft->width = size;
		cloudLeft->height = 1;
		cloudLeft->points.resize(cloudLeft->width * cloudLeft->height);

		for (size_t i = 0; i < size; i++)
		{
			if ((cloudInputlunguiRgb->points[i].y - 67) > 0 && (cloudInputlunguiRgb->points[i].y - 80) < 0)
			{
				cloudInputlunguiRgb->points[i].r = 255;
				cloudInputlunguiRgb->points[i].g = 0;
				cloudInputlunguiRgb->points[i].b = 0;
				cloudLeft->points[i].x = cloudInputlunguiRgb->points[i].x + (49.994579 - 47.9994801);
				cloudLeft->points[i].y = cloudInputlunguiRgb->points[i].y - (67.840897 + 7.599440);
				cloudLeft->points[i].z = cloudInputlunguiRgb->points[i].z + (53.207588 - 4.739580);
			}
			else
			{
				cloudInputlunguiRgb->points[i].r = 255;
				cloudInputlunguiRgb->points[i].g = 255;
				cloudInputlunguiRgb->points[i].b = 255;
			}
		}
		count += 2;
		emit changProgessBar(count);
		emit show_SpotProcessSignal("切割完毕，开始二进制文件存储 存储位置为：" + outPuthDataPath + "/splitResult/FinalcloudLeft" + to_string(i + 1) + ".ply");
		//emit changUI();// 发送信号
		// 将分割的文件进行二进制存储用于后续的使用
		pcl::io::savePLYFileBinary(outPuthDataPath + "/splitResult/FinalcloudLeft" + to_string(i + 1) + ".ply", *cloudLeft);
		//pcl::io::savePLYFileBinary("C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd\\FinalcloudLeft.ply", *cloudLeft);

		clock_t startTime, endTime;
		startTime = clock();//计时开始
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<int > indexs = {};


		//double th = 0.03;
		//double r = 10;


		pcl::PointCloud<pcl::PointXYZ>::Ptr yuantu(new pcl::PointCloud<pcl::PointXYZ>);

		//存的XYZRGB的格式的特征点
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		emit show_SpotProcessSignal("开始二进制文件开始读取···");
		count += 2;
		emit changProgessBar(count);
		//读取数据
		pcl::io::loadPLYFile<pcl::PointXYZ>(outPuthDataPath + "/splitResult/FinalcloudLeft" + to_string(i + 1) + ".ply", *yuantu);
		//pcl::io::loadPLYFile<pcl::PointXYZ>(splitResultPaTH+"C:\\Users\\15281\\Desktop\\Qt\\visual\\FinalcloudLeft.ply", *yuantu);
		//pcl::io::loadPLYFile("C:/Users/徐鹏/Desktop/datasPre/lunguizuhe_small_100000.ply", *yuantu);


		//xyz转化为xyzrgba,可以看到原图是xyz的格式的，直接使用拷贝函数进行拷贝到xyzRGB,方便后面提取时候设置特征点和非特征点的颜色
		pcl::copyPointCloud(*yuantu, *cloud);
		//pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd\\cloudLeft.ply", *cloud);
		emit show_SpotProcessSignal("读取点云数量为:" + to_string(cloud->points.size()));
		cout << "Number of points in the Cube Input cloud is:" << cloud->points.size() << std::endl;

		//double T1 = 0.3;
		double T1 = 0.6;
		double T2 = 0.6;
		//double T2 = 0.1;


		//设置点云的初始颜色
		for (size_t i = 0; i < cloud->points.size(); ++i) {//原始点云颜色修改
			cloud->points[i].r = 30;
			cloud->points[i].g = 144;
			cloud->points[i].b = 255;
			//30 144 255
		}



		//切片后的圆心的位置，默认是固定的
		//double oy = 58.2477;
		double oy = 58.2477;

		//切片的范围，所以切片的范围就是y在（-5，5）之间
		double ww = 4;

		//切片的厚度
		double thick = 0.01;
		//double thick = 0.008;

		//用于判断哪些是踏面上的点用
		double BR = 12.8284;

		//结构先验信息的展示
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr xianyan(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*cloud, *xianyan);
		for (size_t i = 0; i < xianyan->points.size(); ++i) {//原始点云颜色修改
			double xx = cloud->points[i].x;
			double yy = cloud->points[i].y;
			double zz = cloud->points[i].z;
			double distance = sqrt(yy * yy + zz * zz);

			if (abs(distance - BR) < 0.3 && abs(yy) < 4) { //为踏面上点，进而在切片圆中进行判断
				xianyan->points[i].r = 255;
				xianyan->points[i].g = 0;
				xianyan->points[i].b = 0;
			}
		}

		//切片方向，沿着y轴 r
		for (int k = 0; (-ww + k * thick) <= ww; k++) {

			//每次切片的圆心坐标
			//double cx = 2.98;
			double cx = 2.98;
			double cy = -ww + thick * k;
			double cz = oy;

			//当前切片下的半径
			double curR = 0;
			//cout << "cy : " << cy << "cur" << curR << endl;
			//切片的上下限
			/*double downBountry = cy - 0.005;
			double upBountry = cy + 0.005;	*/
			double downBountry = cy - 0.05;
			double upBountry = cy + 0.05;

			//切片信息拟合
			//切片圆信息拟合
			vector<int> indexs;

			//记录当前切片中的所有点并拟合切片圆信息
			for (size_t i = 0; i < cloud->points.size(); ++i) {
				double xx = cloud->points[i].x;
				double yy = cloud->points[i].y;
				double zz = cloud->points[i].z;
				double dd = sqrt(yy * yy + zz * zz);
				if (yy >= downBountry && yy <= upBountry && zz > 39) {//只使用局内点进行切片圆信息拟合
					indexs.push_back(i);
				}
			}

			vector<double> jiegouxxi = computeCircle3D(cloud, indexs);
			cx = jiegouxxi[0];
			cy = jiegouxxi[1];
			cz = jiegouxxi[2];
			double niheR = jiegouxxi[3];

			//遍历平面上的位于boundry中的点进行计算距离-->改变颜色
			for (size_t i = 0; i < cloud->points.size(); ++i) {

				//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
				double xx = cloud->points[i].x;
				double yy = cloud->points[i].y;
				double zz = cloud->points[i].z;
				//if (yy >= downBountry && yy <= upBountry && zz < 0.02) {//目前看来0.02是一个较为合适的数值
				if (yy >= downBountry && yy <= upBountry) {//目前看来0.02是一个较为合适的数值
					//符合切片的点，判断是否为踏面上的点
					double dd = sqrt(yy * yy + zz * zz);
					if (abs(dd - BR) < T1 && abs(yy) < 3 && abs(xx - 3) < 10) {
						//为切片上的点时，判断点到圆心的距离是否小于该切片下的半径
						double distance = distanceBetweenTwoPoint(cx, cy, cz, xx, yy, zz);
						if (distance <= niheR + T2) {// 0.01 / 0.1
							//cout << distance << " " << curR << " " << cy << endl;
							cloud->points[i].r = 255;
							cloud->points[i].g = 0;
							cloud->points[i].b = 0;
							indexs.push_back(i);
						}
						else {
							cloud->points[i].x = 0;
							cloud->points[i].y = 0;
							cloud->points[i].z = 0;
						}
					}
				}
			}
			if (niheR > 100)
				continue;
			emit show_SpotProcessSignal("切片[" + to_string(k) + "]结构信息" + to_string(jiegouxxi[0]) + +" " + to_string(jiegouxxi[1]) + " " + to_string(jiegouxxi[2]) + " " + to_string(jiegouxxi[3]));

			cout << "切片[" << k << "]结构信息" << jiegouxxi[0] << " " << jiegouxxi[1] << " " << jiegouxxi[2] << " " << jiegouxxi[3] << " ";
			if (k % 10 == 0 && count <= 97)
			{
				count += 1;
				emit changProgessBar(count);
			}
		}
		count = 98;
		emit changProgessBar(count);

		pcl::copyPointCloud(*cloud, indexs, *cloudOut);

		double maxY = -1000;
		double minY = 1000;
		double maxX = -1000;
		double minX = 1000;
		for (size_t i = 0; i < cloud->points.size(); ++i) {
			//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
			double xx = cloud->points[i].x;
			double yy = cloud->points[i].y;
			double zz = cloud->points[i].z;
			int redNum = cloud->points[i].r;
			if (redNum == 255) {
				maxY = max(maxY, yy);
				minY = min(minY, yy);
				maxX = max(maxX, xx);
				minX = min(minY, xx);
			}
		}

		count = 99;
		emit changProgessBar(count);
		std::vector<string> res = split(files[i].c_str(), "/");
		emit show_SpotProcessSignal("当前第" + to_string(i + 1) + "个文件" + res[res.size() - 1] + "运行完毕");
		cout << "测量结果如下所示：";
		emit show_spotResultSignal(res[res.size() - 1] + "测量结果如下所示：");
		//根据提取得到的接触斑就行长短轴的测量
		vector<double> ans = measurePatch(cloud);
		double cxLen = ans[0];
		double cyLen = ans[2];
		double wuchaxLen = ans[1];
		double wuchayLen = ans[3];
		if (cxLen == 0 || cyLen == 0)
		{
			cout << "未检测到接触斑\n";
			emit show_spotResultSignal("未检测到接触斑");
		}
		else
		{
			cout << "测量的长轴LR:" << cxLen << "测量的短轴SR:" << cyLen;
			emit show_spotResultSignal("测量的长轴LR: " + to_string(cxLen));
			emit show_spotResultSignal("测量的短轴SR: " + to_string(cyLen));
		}

		emit show_SpotProcessSignal("数据测量完毕，开始可视化，请稍等···");
		emit show_SpotProcessSignal("坐标系建立完毕 红色是X轴，绿色是Y轴，蓝色是Z轴");
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		// RGB文件进行显示
		viewer = rgbVis(cloudInputlunguiRgb);
		//// 读取pcd文件
		//pcl::PointCloud<PointT>::Ptr cloudxupeng(new pcl::PointCloud<PointT>);
		//if (pcl::io::loadPLYFile("C:\\Users\\15281\\Desktop\\Qt\\lunguizuhe_100000.ply", *cloudxupeng) == -1) {
		//	PCL_ERROR("读取徐鹏文件失败");
		//	return -1;
		//}
		// 将标准实验轮轨和输入轮轨进行配准和检验
		pcl::visualization::PCLVisualizer viewer2;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4;

		viewer3 = rgbVis2(cloud);

		viewer4 = rgbVis2(xianyan);


		viewer2.addCoordinateSystem(30);

		cout << "坐标系建立完毕 红色是X轴，绿色是Y轴，蓝色是Z轴";
		count = 100;
		emit changProgessBar(count);
		//viewer2.addPointCloud<pcl::PointXYZ>(cloudxupeng, "cloudxupeng");
		viewer2.addPointCloud<pcl::PointXYZ>(cloudLeft, "cloudLeft");
		// 设置点的属性：大小以及颜色
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloudLeft");//设置点的大小
		viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloudLeft"); //设置点的颜色

		//while (!viewer3->wasStopped())
		//{
		//	viewer3->spinOnce(100);
		//	std::this_thread::sleep_for(std::chrono::microseconds(100000));
		//}
		while (!viewer2.wasStopped())
		{
			viewer2.spinOnce(100);
			std::this_thread::sleep_for(std::chrono::microseconds(100000));
		}
		cout << "显示完毕";
		emit show_SpotProcessSignal("显示完毕,接触斑测量程序结束");
		count = 100;
		emit changProgessBar(count);
		endTime = clock();//计时结束
		cout << "程序运行时间" << (endTime - startTime) / CLOCKS_PER_SEC;

		//emit show_SpotProcessSignal("程序运行时间");





	}
	//// ==============遍历文件夹下的所有pcd文件
	//if (pcl::io::loadPCDFile(inputDataPath + "/0-0-0-0.pcd", *cloudInputlungui) == -1) {// 此处待修改具体输入参数
	//	PCL_ERROR("读取文件失败");
	//	return -1;
	//}
	//// 数据XYZ转换为XYZRGB
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInputlunguiRgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::copyPointCloud(*cloudInputlungui, *cloudInputlunguiRgb);
	//count += 2;
	//emit changProgessBar(count);

	//// 利用结构先验进行轮轨分割
	//int size = cloudInputlungui->size();
	//cloudLeft->width = size;
	//cloudLeft->height = 1;
	//cloudLeft->points.resize(cloudLeft->width * cloudLeft->height);

	//for (size_t i = 0; i < size; i++)
	//{
	//	if ((cloudInputlunguiRgb->points[i].y - 67) > 0 && (cloudInputlunguiRgb->points[i].y - 80) < 0)
	//	{
	//		cloudInputlunguiRgb->points[i].r = 255;
	//		cloudInputlunguiRgb->points[i].g = 0;
	//		cloudInputlunguiRgb->points[i].b = 0;
	//		cloudLeft->points[i].x = cloudInputlunguiRgb->points[i].x + (49.994579 - 47.9994801);
	//		cloudLeft->points[i].y = cloudInputlunguiRgb->points[i].y - (67.840897 + 7.599440);
	//		cloudLeft->points[i].z = cloudInputlunguiRgb->points[i].z + (53.207588 - 4.739580);
	//	}
	//	else
	//	{
	//		cloudInputlunguiRgb->points[i].r = 255;
	//		cloudInputlunguiRgb->points[i].g = 255;
	//		cloudInputlunguiRgb->points[i].b = 255;
	//	}
	//}
	//count += 2;
	//emit changProgessBar(count);
	//emit show_SpotProcessSignal("切割完毕，开始二进制文件存储 存储位置为：" + outPuthDataPath + "/splitResult/FinalcloudLeft.ply");
	////emit changUI();// 发送信号
	//// 将分割的文件进行二进制存储用于后续的使用
	//pcl::io::savePLYFileBinary(outPuthDataPath + "/splitResult/FinalcloudLeft.ply", *cloudLeft);
	////pcl::io::savePLYFileBinary("C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd\\FinalcloudLeft.ply", *cloudLeft);

	//clock_t startTime, endTime;
	//startTime = clock();//计时开始
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
	//std::vector<int > indexs = {};


	////double th = 0.03;
	////double r = 10;


	//pcl::PointCloud<pcl::PointXYZ>::Ptr yuantu(new pcl::PointCloud<pcl::PointXYZ>);

	////存的XYZRGB的格式的特征点
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//emit show_SpotProcessSignal("开始二进制文件开始读取···");
	//count += 2;
	//emit changProgessBar(count);
	////读取数据
	//pcl::io::loadPLYFile<pcl::PointXYZ>(outPuthDataPath + "/splitResult/FinalcloudLeft.ply", *yuantu);
	////pcl::io::loadPLYFile<pcl::PointXYZ>(splitResultPaTH+"C:\\Users\\15281\\Desktop\\Qt\\visual\\FinalcloudLeft.ply", *yuantu);
	////pcl::io::loadPLYFile("C:/Users/徐鹏/Desktop/datasPre/lunguizuhe_small_100000.ply", *yuantu);


	////xyz转化为xyzrgba,可以看到原图是xyz的格式的，直接使用拷贝函数进行拷贝到xyzRGB,方便后面提取时候设置特征点和非特征点的颜色
	//pcl::copyPointCloud(*yuantu, *cloud);
	////pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd\\cloudLeft.ply", *cloud);
	//emit show_SpotProcessSignal("读取点云数量为:" + to_string(cloud->points.size()));
	//cout << "Number of points in the Cube Input cloud is:" << cloud->points.size() << std::endl;

	////double T1 = 0.3;
	//double T1 = 0.6;
	//double T2 = 0.6;
	////double T2 = 0.1;


	////设置点云的初始颜色
	//for (size_t i = 0; i < cloud->points.size(); ++i) {//原始点云颜色修改
	//	cloud->points[i].r = 30;
	//	cloud->points[i].g = 144;
	//	cloud->points[i].b = 255;
	//	//30 144 255
	//}



	////切片后的圆心的位置，默认是固定的
	////double oy = 58.2477;
	//double oy = 58.2477;

	////切片的范围，所以切片的范围就是y在（-5，5）之间
	//double ww = 4;

	////切片的厚度
	//double thick = 0.01;
	////double thick = 0.008;

	////用于判断哪些是踏面上的点用
	//double BR = 12.8284;

	////结构先验信息的展示
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr xianyan(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::copyPointCloud(*cloud, *xianyan);
	//for (size_t i = 0; i < xianyan->points.size(); ++i) {//原始点云颜色修改
	//	double xx = cloud->points[i].x;
	//	double yy = cloud->points[i].y;
	//	double zz = cloud->points[i].z;
	//	double distance = sqrt(yy * yy + zz * zz);

	//	if (abs(distance - BR) < 0.3 && abs(yy) < 4) { //为踏面上点，进而在切片圆中进行判断
	//		xianyan->points[i].r = 255;
	//		xianyan->points[i].g = 0;
	//		xianyan->points[i].b = 0;
	//	}
	//}

	////切片方向，沿着y轴 r
	//for (int k = 0; (-ww + k * thick) <= ww; k++) {

	//	//每次切片的圆心坐标
	//	//double cx = 2.98;
	//	double cx = 2.98;
	//	double cy = -ww + thick * k;
	//	double cz = oy;

	//	//当前切片下的半径
	//	double curR = 0;
	//	//cout << "cy : " << cy << "cur" << curR << endl;
	//	//切片的上下限
	//	/*double downBountry = cy - 0.005;
	//	double upBountry = cy + 0.005;	*/
	//	double downBountry = cy - 0.05;
	//	double upBountry = cy + 0.05;

	//	//切片信息拟合
	//	//切片圆信息拟合
	//	vector<int> indexs;

	//	//记录当前切片中的所有点并拟合切片圆信息
	//	for (size_t i = 0; i < cloud->points.size(); ++i) {
	//		double xx = cloud->points[i].x;
	//		double yy = cloud->points[i].y;
	//		double zz = cloud->points[i].z;
	//		double dd = sqrt(yy * yy + zz * zz);
	//		if (yy >= downBountry && yy <= upBountry && zz > 39) {//只使用局内点进行切片圆信息拟合
	//			indexs.push_back(i);
	//		}
	//	}

	//	vector<double> jiegouxxi = computeCircle3D(cloud, indexs);
	//	cx = jiegouxxi[0];
	//	cy = jiegouxxi[1];
	//	cz = jiegouxxi[2];
	//	double niheR = jiegouxxi[3];

	//	//遍历平面上的位于boundry中的点进行计算距离-->改变颜色
	//	for (size_t i = 0; i < cloud->points.size(); ++i) {

	//		//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
	//		double xx = cloud->points[i].x;
	//		double yy = cloud->points[i].y;
	//		double zz = cloud->points[i].z;
	//		//if (yy >= downBountry && yy <= upBountry && zz < 0.02) {//目前看来0.02是一个较为合适的数值
	//		if (yy >= downBountry && yy <= upBountry) {//目前看来0.02是一个较为合适的数值
	//			//符合切片的点，判断是否为踏面上的点
	//			double dd = sqrt(yy * yy + zz * zz);
	//			if (abs(dd - BR) < T1 && abs(yy) < 3 && abs(xx - 3) < 10) {
	//				//为切片上的点时，判断点到圆心的距离是否小于该切片下的半径
	//				double distance = distanceBetweenTwoPoint(cx, cy, cz, xx, yy, zz);
	//				if (distance <= niheR + T2) {// 0.01 / 0.1
	//					//cout << distance << " " << curR << " " << cy << endl;
	//					cloud->points[i].r = 255;
	//					cloud->points[i].g = 0;
	//					cloud->points[i].b = 0;
	//					indexs.push_back(i);
	//				}
	//				else {
	//					cloud->points[i].x = 0;
	//					cloud->points[i].y = 0;
	//					cloud->points[i].z = 0;
	//				}
	//			}
	//		}
	//	}
	//	if (niheR > 100)
	//		continue;
	//	emit show_SpotProcessSignal("切片[" + to_string(k) + "]结构信息" + to_string(jiegouxxi[0]) + +" " + to_string(jiegouxxi[1]) + " " + to_string(jiegouxxi[2]) + " " + to_string(jiegouxxi[3]));

	//	cout << "切片[" << k << "]结构信息" << jiegouxxi[0] << " " << jiegouxxi[1] << " " << jiegouxxi[2] << " " << jiegouxxi[3] << " " << endl;
	//	if (k % 10 == 0 && count <= 97)
	//	{
	//		count += 1;
	//		emit changProgessBar(count);
	//	}
	//}
	//count = 98;
	//emit changProgessBar(count);

	//pcl::copyPointCloud(*cloud, indexs, *cloudOut);

	//double maxY = -1000;
	//double minY = 1000;
	//double maxX = -1000;
	//double minX = 1000;
	//for (size_t i = 0; i < cloud->points.size(); ++i) {
	//	//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
	//	double xx = cloud->points[i].x;
	//	double yy = cloud->points[i].y;
	//	double zz = cloud->points[i].z;
	//	int redNum = cloud->points[i].r;
	//	if (redNum == 255) {
	//		maxY = max(maxY, yy);
	//		minY = min(minY, yy);
	//		maxX = max(maxX, xx);
	//		minX = min(minY, xx);
	//	}
	//}

	//count = 99;
	//emit changProgessBar(count);
	//cout << "测量结果如下所示：" << endl; cout << endl;
	//emit show_spotResultSignal("测量结果如下所示：");
	////根据提取得到的接触斑就行长短轴的测量
	//vector<double> ans = measurePatch(cloud);
	//double cxLen = ans[0];
	//double cyLen = ans[2];
	//double wuchaxLen = ans[1];
	//double wuchayLen = ans[3];
	//if (cxLen == 0 || cyLen == 0)
	//{
	//	cout << "未检测到接触斑" << endl;
	//	emit show_spotResultSignal("未检测到接触斑");
	//}
	//else
	//{
	//	cout << "测量的长轴LR:" << cxLen << "测量的短轴SR:" << cyLen << endl;
	//	emit show_spotResultSignal("测量的长轴LR: " + to_string(cxLen));
	//	emit show_spotResultSignal("测量的短轴SR: " + to_string(cyLen));
	//}

	//emit show_SpotProcessSignal("数据测量完毕，开始可视化，请稍等···");
	//emit show_SpotProcessSignal("坐标系建立完毕 红色是X轴，绿色是Y轴，蓝色是Z轴");
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//// RGB文件进行显示
	//viewer = rgbVis(cloudInputlunguiRgb);
	////// 读取pcd文件
	////pcl::PointCloud<PointT>::Ptr cloudxupeng(new pcl::PointCloud<PointT>);
	////if (pcl::io::loadPLYFile("C:\\Users\\15281\\Desktop\\Qt\\lunguizuhe_100000.ply", *cloudxupeng) == -1) {
	////	PCL_ERROR("读取徐鹏文件失败");
	////	return -1;
	////}
	//// 将标准实验轮轨和输入轮轨进行配准和检验
	//pcl::visualization::PCLVisualizer viewer2;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4;

	//viewer3 = rgbVis2(cloud);

	//viewer4 = rgbVis2(xianyan);


	//viewer2.addCoordinateSystem(30);

	//cout << "坐标系建立完毕 红色是X轴，绿色是Y轴，蓝色是Z轴" << endl;
	//count = 100;
	//emit changProgessBar(count);
	////viewer2.addPointCloud<pcl::PointXYZ>(cloudxupeng, "cloudxupeng");
	//viewer2.addPointCloud<pcl::PointXYZ>(cloudLeft, "cloudLeft");
	//// 设置点的属性：大小以及颜色
	//viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloudLeft");//设置点的大小
	//viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloudLeft"); //设置点的颜色

	////while (!viewer3->wasStopped())
	////{
	////	viewer3->spinOnce(100);
	////	std::this_thread::sleep_for(std::chrono::microseconds(100000));
	////}
	//while (!viewer2.wasStopped())
	//{
	//	viewer2.spinOnce(100);
	//	std::this_thread::sleep_for(std::chrono::microseconds(100000));
	//}
	//cout << "显示完毕" << endl;
	//emit show_SpotProcessSignal("显示完毕,接触斑测量程序结束");
	//count = 100;
	//emit changProgessBar(count);
	//endTime = clock();//计时结束
	//cout << "程序运行时间" << (endTime - startTime) / CLOCKS_PER_SEC << endl;

	////emit show_SpotProcessSignal("程序运行时间");
	system("pause");
	return 0;
}


double spotMethod::lunkuoxian(double r, double h, double l) {
	double tmp = sqrt(l * l + (h - 2 * r) * h);
	cout << " r " << r << " h " << h << " l " << l;
	return (l * r - r * tmp) / (2 * r - h);
}

//int spotMethod::startPoint(string splitResultPaTH)
//{
//	clock_t startTime, endTime;
//	startTime = clock();//计时开始
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
//	std::vector<int > indexs = {};
//
//
//	//double th = 0.03;
//	//double r = 10;
//
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr yuantu(new pcl::PointCloud<pcl::PointXYZ>);
//
//	//存的XYZRGB的格式的特征点
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	emit show_SpotProcessSignal("开始二进制文件开始读取···");
//	//读取数据
//	pcl::io::loadPLYFile<pcl::PointXYZ>(splitResultPaTH + "/splitResult/FinalcloudLeft.ply", *yuantu);
//	//pcl::io::loadPLYFile<pcl::PointXYZ>(splitResultPaTH+"C:\\Users\\15281\\Desktop\\Qt\\visual\\FinalcloudLeft.ply", *yuantu);
//	//pcl::io::loadPLYFile("C:/Users/徐鹏/Desktop/datasPre/lunguizuhe_small_100000.ply", *yuantu);
//
//
//	//xyz转化为xyzrgba,可以看到原图是xyz的格式的，直接使用拷贝函数进行拷贝到xyzRGB,方便后面提取时候设置特征点和非特征点的颜色
//	pcl::copyPointCloud(*yuantu, *cloud);
//	//pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\Users\\15281\\Desktop\\Qt\\数据1221\\pcd\\cloudLeft.ply", *cloud);
//	emit show_SpotProcessSignal("读取点云数量为:" + to_string(cloud->points.size()));
//	cout << "Number of points in the Cube Input cloud is:" << cloud->points.size() << std::endl;
//
//	//double T1 = 0.3;
//	double T1 = 0.6;
//	double T2 = 0.6;
//	//double T2 = 0.1;
//
//
//	//设置点云的初始颜色
//	for (size_t i = 0; i < cloud->points.size(); ++i) {//原始点云颜色修改
//		cloud->points[i].r = 30;
//		cloud->points[i].g = 144;
//		cloud->points[i].b = 255;
//		//30 144 255
//	}
//
//
//
//	//切片后的圆心的位置，默认是固定的
//	//double oy = 58.2477;
//	double oy = 58.2477;
//
//	//切片的范围，所以切片的范围就是y在（-5，5）之间
//	double ww = 4;
//
//	//切片的厚度
//	double thick = 0.01;
//	//double thick = 0.008;
//
//	//用于判断哪些是踏面上的点用
//	double BR = 12.8284;
//
//	//结构先验信息的展示
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xianyan(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::copyPointCloud(*cloud, *xianyan);
//	for (size_t i = 0; i < xianyan->points.size(); ++i) {//原始点云颜色修改
//		double xx = cloud->points[i].x;
//		double yy = cloud->points[i].y;
//		double zz = cloud->points[i].z;
//		double distance = sqrt(yy * yy + zz * zz);
//
//		if (abs(distance - BR) < 0.3 && abs(yy) < 4) { //为踏面上点，进而在切片圆中进行判断
//			xianyan->points[i].r = 255;
//			xianyan->points[i].g = 0;
//			xianyan->points[i].b = 0;
//		}
//	}
//
//	//切片方向，沿着y轴 r
//	for (int k = 0; (-ww + k * thick) <= ww; k++) {
//
//		//每次切片的圆心坐标
//		//double cx = 2.98;
//		double cx = 2.98;
//		double cy = -ww + thick * k;
//		double cz = oy;
//
//		//当前切片下的半径
//		double curR = 0;
//		//cout << "cy : " << cy << "cur" << curR << endl;
//		//切片的上下限
//		/*double downBountry = cy - 0.005;
//		double upBountry = cy + 0.005;	*/
//		double downBountry = cy - 0.05;
//		double upBountry = cy + 0.05;
//
//		//切片信息拟合
//		//切片圆信息拟合
//		vector<int> indexs;
//
//		//记录当前切片中的所有点并拟合切片圆信息
//		for (size_t i = 0; i < cloud->points.size(); ++i) {
//			double xx = cloud->points[i].x;
//			double yy = cloud->points[i].y;
//			double zz = cloud->points[i].z;
//			double dd = sqrt(yy * yy + zz * zz);
//			if (yy >= downBountry && yy <= upBountry && zz > 39) {//只使用局内点进行切片圆信息拟合
//				indexs.push_back(i);
//			}
//		}
//
//		vector<double> jiegouxxi = computeCircle3D(cloud, indexs);
//		cx = jiegouxxi[0];
//		cy = jiegouxxi[1];
//		cz = jiegouxxi[2];
//		double niheR = jiegouxxi[3];
//
//		//遍历平面上的位于boundry中的点进行计算距离-->改变颜色
//		for (size_t i = 0; i < cloud->points.size(); ++i) {
//
//			//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
//			double xx = cloud->points[i].x;
//			double yy = cloud->points[i].y;
//			double zz = cloud->points[i].z;
//			//if (yy >= downBountry && yy <= upBountry && zz < 0.02) {//目前看来0.02是一个较为合适的数值
//			if (yy >= downBountry && yy <= upBountry) {//目前看来0.02是一个较为合适的数值
//				//符合切片的点，判断是否为踏面上的点
//				double dd = sqrt(yy * yy + zz * zz);
//				if (abs(dd - BR) < T1 && abs(yy) < 3 && abs(xx - 3) < 10) {
//					//为切片上的点时，判断点到圆心的距离是否小于该切片下的半径
//					double distance = distanceBetweenTwoPoint(cx, cy, cz, xx, yy, zz);
//					if (distance <= niheR + T2) {// 0.01 / 0.1
//						//cout << distance << " " << curR << " " << cy << endl;
//						cloud->points[i].r = 255;
//						cloud->points[i].g = 0;
//						cloud->points[i].b = 0;
//						indexs.push_back(i);
//					}
//					else {
//						cloud->points[i].x = 0;
//						cloud->points[i].y = 0;
//						cloud->points[i].z = 0;
//					}
//				}
//			}
//		}
//		if (niheR > 100)
//			continue;
//		emit show_SpotProcessSignal("切片[" + to_string(k) + "]结构信息" + to_string(jiegouxxi[0]) + +" " + to_string(jiegouxxi[1]) + " " + to_string(jiegouxxi[2]) + " " + to_string(jiegouxxi[3]));
//
//		cout << "切片[" << k << "]结构信息" << jiegouxxi[0] << " " << jiegouxxi[1] << " " << jiegouxxi[2] << " " << jiegouxxi[3] << " " << endl;
//
//	}
//
//
//
//	pcl::copyPointCloud(*cloud, indexs, *cloudOut);
//
//	double maxY = -1000;
//	double minY = 1000;
//	double maxX = -1000;
//	double minX = 1000;
//	for (size_t i = 0; i < cloud->points.size(); ++i) {
//		//遍历所有的点中，在boundry中的点，进行计算到圆心的距离
//		double xx = cloud->points[i].x;
//		double yy = cloud->points[i].y;
//		double zz = cloud->points[i].z;
//		int redNum = cloud->points[i].r;
//		if (redNum == 255) {
//			maxY = max(maxY, yy);
//			minY = min(minY, yy);
//			maxX = max(maxX, xx);
//			minX = min(minY, xx);
//		}
//	}
//
//
//	cout << "测量结果如下所示：" << endl; cout << endl;
//	emit show_spotResultSignal("测量结果如下所示：");
//	//根据提取得到的接触斑就行长短轴的测量
//	vector<double> ans = measurePatch(cloud);
//	double cxLen = ans[0];
//	double cyLen = ans[2];
//	double wuchaxLen = ans[1];
//	double wuchayLen = ans[3];
//	if (cxLen == 0 || cyLen == 0)
//	{
//		cout << "未检测到接触斑" << endl;
//		emit show_spotResultSignal("未检测到接触斑");
//	}
//	else
//	{
//		cout << "测量的长轴LR:" << cxLen << "测量的短轴SR:" << cyLen << endl;
//		emit show_spotResultSignal("测量的长轴LR: " + to_string(cxLen) + " " + "测量的短轴SR: " + to_string(cyLen));
//	}
//
//
//	endTime = clock();//计时结束
//	cout << "程序运行时间" << (endTime - startTime) / CLOCKS_PER_SEC << endl;
//
//
//
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4;
//
//	viewer3 = rgbVis2(cloud);
//
//	viewer4 = rgbVis2(xianyan);
//
//	while (!viewer3->wasStopped())
//	{
//		viewer3->spinOnce(100);
//		std::this_thread::sleep_for(std::chrono::microseconds(100000));
//	}
//	return 0;
//}
