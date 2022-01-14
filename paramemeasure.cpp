#include "paramemeasure.h"
#include <QTextCursor>
#include<qstring.h>
#include<qdebug.h>
#include<qfile.h>
#include "measure.h"
#include <qfiledialog.h>
#include"spotMeasure.h"
#include<qstring.h>
#include<string>
#include"NiuRansac.h"
#include <iomanip>

using namespace std;
ParameMeasure::ParameMeasure(QWidget* parent)
	: QMainWindow(parent)
{

	qRegisterMetaType<spotMethod>("spotMethod");
	qRegisterMetaType<spotMethod*>("spotMethod*");
	qRegisterMetaType<string>("string");
	qRegisterMetaType<Measure>("Measure");
	qRegisterMetaType<Measure*>("Measure*");
	measure = new Measure();
	mysopt = new spotMethod();// 接触斑测量类
	spotThread = new myThread();
	niuransac = new NiuRansacAlgo();
	mThread = new measureThread();
	ui.setupUi(this);
	this->setWindowIcon(QIcon("./bitbug_favicon.ico"));

	ui.progressBar->setValue(0);
	ui.progressBar_3->setValue(0);// 实时更新进度条信息
	//connect();
	drawEcharts();// 自动调用初始化饼状图
	connect(ui.StartButton, SIGNAL(clicked()), this, SLOT(startRansac()));// 点击运行按钮开始程序运行
	connect(ui.SelectInputFolder, SIGNAL(clicked()), this, SLOT(setInputPath()));// 点击选择文件按钮获得当前输入文件路径
	connect(ui.SelectOutFolder, SIGNAL(clicked()), this, SLOT(setOutputPath()));// 点击选择文件按钮获得当前输出文件路径
	connect(ui.startSpot, SIGNAL(clicked()), this, SLOT(testMywork()));// 点击选择文件按钮获得当前输出文件路径
	connect(measure, SIGNAL(show_message(string)), this, SLOT(show_message(string)));// 发送信号给measure.cpp 开始测量过程ui 信息修改
	connect(measure, SIGNAL(show_MeaureResultmessage(string)), this, SLOT(show_MeaureResultmessage(string)));// 发送信号给measure.cpp 开始测量结果ui 信息修改
	connect(mysopt, SIGNAL(show_SpotProcessSignal(string)), this, SLOT(show_SpotProcessSignal(string)));// 发送信号给soptMeasure.cpp 开始测量过程ui 信息修改
	connect(mysopt, SIGNAL(changUI()), this, SLOT(changUI()));// 发送信号给soptMeasure.cpp 开始测量过程ui 信息修改
	connect(mysopt, SIGNAL(show_spotResultSignal(string)), this, SLOT(show_spotResultSignal(string)));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	connect(this, SIGNAL(sendSpotPathMessage(spotMethod*, string, string)), spotThread, SLOT(getPath(spotMethod*, string, string)));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	///connect(this, &ParameMeasure::sendSpotPathMessage, spotThread, &myThread::getPath);
	connect(mysopt, SIGNAL(changProgessBar(int)), this, SLOT(changProgessBar(int)));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	connect(measure, SIGNAL(changMeasureProgessBar(int)), this, SLOT(changMeasureProgessBar(int)));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	connect(this, SIGNAL(sendMeasureMessage(NiuRansacAlgo*, Measure*, string, string)), mThread, SLOT(getMeasurePath(NiuRansacAlgo*, Measure*, string, string)));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	connect(mThread, SIGNAL(sendMeasureThreadMessage()), this, SLOT(sendMeasureThreadMessage()));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	connect(spotThread, SIGNAL(sendMeasureThreadMessage2()), this, SLOT(sendMeasureThreadMessage2()));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	// 两个暂停按钮使用
	//connect(ui.stopMeasure, SIGNAL(clicked()), this, SLOT(sendMeasureThreadMessage2()));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
	//connect(ui.stopSpot, SIGNAL(clicked()), this, SLOT(sendMeasureThreadMessage2()));// 发送信号给soptMeasure.cpp 开始测量结果ui 信息修改
}
QString ParameMeasure::openFolder()//获得打开文件路径
{
	QFileDialog* filepath = new QFileDialog(this);
	qDebug() << "开始测试选择文件夹";
	return filepath->getExistingDirectory();
}
void ParameMeasure::setInputPath()//设置输入文件路径
{
	ui.inputDataPath->setText(openFolder());
}void ParameMeasure::setOutputPath()//设置输出文件路径并自动创建文件夹(若存在则不创建)
{
	QString path = openFolder();
	ui.OutPutPath->setText(path);
	//QString x = path + "fengeLunYuanPCD";
	qDebug() << path;
	QDir dir;
	if (!dir.exists(path + "/fengeLunYuanPCD"))
	{
		dir.mkdir(path + "/fengeLunYuanPCD");
	}
	if (!dir.exists(path + "/remainPoints"))
	{
		dir.mkdir(path + "/remainPoints");
	}
	if (!dir.exists(path + "/middleResult"))
	{
		dir.mkdir(path + "/middleResult");
	}if (!dir.exists(path + "/torusResult"))
	{
		dir.mkdir(path + "/torusResult");
	}if (!dir.exists(path + "/splitResult"))
	{
		dir.mkdir(path + "/splitResult");
	}
}



void ParameMeasure::drawEcharts() {// 测试饼状图

	QPieSlice* slice_1 = new QPieSlice(QStringLiteral("正常比例：0.8"), 0.8, this);
	slice_1->setLabelVisible(true); // 显示饼状区对应的数据label
	slice_1->setBrush(Qt::green);
	QPieSlice* slice_2 = new QPieSlice(QStringLiteral("误差比例：0.2"), 0.2, this);
	slice_2->setLabelVisible(true);
	slice_2->setBrush(Qt::red);

	// 将两个饼状分区加入series
	QPieSeries* series = new QPieSeries(this);
	series->append(slice_1);
	series->append(slice_2);
	//series->setPieSize(50);//饼图的大小
	series->setHoleSize(0.1);
	series->setLabelsVisible(true);
	QChart* chart = new QChart();
	chart->addSeries(series);
	chart->setAnimationOptions(QChart::AllAnimations); // 设置显示时的动画效果
	QChartView* chartview = new QChartView(this);
	chartview->show();
	chartview->setChart(chart);


	ui.Echarts->insertWidget(0, chartview);
	/*NiuRansacAlgo my;
	int a = my.measure_wheelrail();
	qDebug() << a;*/
}



void ParameMeasure::show_message(string message) {//测量过程ui 信息修改
	qDebug() << "here\n";
	qDebug() << QString::fromStdString(message);
	ui.outputWindow->append(QString::fromStdString(message));
	ui.outputWindow->repaint();// 实时更新输出窗口信息

}

void ParameMeasure::show_MeaureResultmessage(string message)//开始测量结果ui 信息修改
{
	//qDebug() << "here\n";
	ui.MeasureResultShow->append(QString::fromStdString(message));
	ui.MeasureResultShow->repaint();// 实时更新输出窗口信息
}

void ParameMeasure::show_SpotProcessSignal(string message)
{
	//qDebug() << "here\n";
	ui.spotOutputWindow->append(QString::fromStdString(message));
	ui.spotOutputWindow->repaint();// 实时更新输出窗口信息
}

void ParameMeasure::show_spotResultSignal(string message)
{
	ui.spotResultShow->append(QString::fromStdString(message));
	ui.spotResultShow->repaint();// 实时更新输出窗口信息
}


void ParameMeasure::testMywork()
{/*
	QThread* thread;
	thread = new QThread();*/
	qDebug() << "开始传输数据";
	emit sendSpotPathMessage(mysopt, ui.inputDataPath->text().toLocal8Bit().constData(), ui.OutPutPath->text().toLocal8Bit().constData());
	//ui.progressBar->setValue(30);
	qDebug() << "开始传输数据2";
	/*QTime dieTIme = QTime::currentTime().addMSecs(1000);
	while (QTime::currentTime() < dieTIme) {
		QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
	}*/
	spotThread->start();
}

void ParameMeasure::changProgessBar(int num)
{
	ui.progressBar_3->setValue(num);// 实时更新进度条信息
}
void ParameMeasure::changMeasureProgessBar(int num)
{
	ui.progressBar->setValue(num);// 实时更新进度条信息
}

void ParameMeasure::startRansac() {// 测试Ransac算法

	qDebug() << "开始参数测量";
	qDebug() << "开始计算";
	ui.outputWindow->append("开始基元检测计算···");
	ui.progressBar->setValue(5);
	ui.outputWindow->repaint();// 实时更新输出窗口信息
	string inputpath = ui.inputDataPath->text().toLocal8Bit().constData();
	qDebug() << "输入文件路径" << ui.inputDataPath->text();
	string outputPath = ui.OutPutPath->text().toLocal8Bit().constData();
	qDebug() << "输出文件路径" << ui.OutPutPath->text();
	// 基元检测
	qDebug() << "开始发送基元检测信号";
	//*****************************************************************************
	emit sendMeasureMessage(niuransac, measure, inputpath, outputPath);
	qDebug() << "发送基元检测信号结束";
	mThread->start();
	//*****************************************************************************
	/*NiuRansacAlgo my;
	int a = my.measure_wheelrail(inputpath, outputPath);*/
	//ui.progressBar->setValue(50);// 第一个进度条假装控制
	//ui.outputWindow->append("基元检测完毕，开始参数测算···");
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	//measure->startMeasure(ui.OutPutPath->text().toLocal8Bit().constData());
	//QString s = QString::number(a, 10);
	//ui.outputWindow->append(s);
	//ui.outputWindow->repaint();// 实时更新输出窗口信息

	//ui.outputWindow->append("轮轨参数测量完毕");
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	//ui.outputWindow->append("程序运行结束");
	//ui.progressBar->setValue(100);// 第一个进度条假装控制
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	//qDebug() << a;
}
//void ParameMeasure::startSpot()// 接触斑点测量算法
//{
//	qDebug() << "开始接触斑点测量";
//	// emit重要响应！！！！！
//	mysopt->startTransform(ui.inputDataPath->text().toLocal8Bit().constData(), ui.OutPutPath->text().toLocal8Bit().constData());
//	/*mysopt->startPoint(ui.OutPutPath->text().toLocal8Bit().constData());*/
//}

void ParameMeasure::sendMeasureThreadMessage() {
	//ui.progressBar->setValue(50);// 第一个进度条假装控制
	ui.outputWindow->append("基元检测完毕，开始参数测算···");
	ui.outputWindow->repaint();// 实时更新输出窗口信息
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	//ui.outputWindow->append("轮轨参数测量完毕");
	//ui.outputWindow->repaint();// 实时更新输出窗口信息
	ui.outputWindow->append("轮轨四参数测量程序运行结束");
	ui.progressBar->setValue(100);// 第一个进度条假装控制
	ui.outputWindow->repaint();// 实时更新输出窗口信息
	//drawEcharts();// 自动调用初始化饼状图
	//drawEcharts(8, 2);// 自动调用初始化饼状图
	mThread->quit();// 释放线程资源
	mThread->wait();
	mThread->deleteLater();
}
void ParameMeasure::sendMeasureThreadMessage2() {

	spotThread->quit();// 释放线程资源
	spotThread->wait();
	spotThread->deleteLater();
}
