#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_paramemeasure.h"
#include <string>
#include <QtCharts>
#include <QtCharts/QPieSeries>
#include <QtCharts/QPieSlice>
#include "measure.h"
#include "myThread.h"
#include "spotMeasure.h"
#include<qstring.h>
#include <QMetaType>
using namespace std;
class ParameMeasure : public QMainWindow
{
	Q_OBJECT

public:
	ParameMeasure(QWidget* parent = Q_NULLPTR);
public slots:
	void startRansac(void);// 开始Ransac 算法
	//void startSpot(void);//开始接触斑测量算法
	void drawEcharts();// Echart可视化
	QString openFolder();// 打开文件夹路径
	void setInputPath();// 设置输入文件夹路径
	void setOutputPath();// 设置输出文件夹路径
	void show_message(string);// 显示参数测量的过程
	void show_MeaureResultmessage(string);// 在最下面显示测量的结果
	void show_SpotProcessSignal(string);//显示运算进度的信号
	void show_spotResultSignal(string);// 显示运算结果的输出信号
	void testMywork();// 测试点击开始线程
	void changProgessBar(int);// 发送信号
	void changMeasureProgessBar(int);//更改进度条
	void sendMeasureThreadMessage();// 发送信号告知主程序当前程序运行结束
	void sendMeasureThreadMessage2();// 告知子线程运行结束
signals:
	void sendSpotPathMessage(spotMethod* mysopt, string input, string out);// 发送信号
	void sendMeasureMessage(NiuRansacAlgo* n, Measure* m, string input, string output);// 向轮轨参数测量进程发送参数

public:
	Ui::ParameMeasureClass ui;
	Measure* measure;
	spotMethod* mysopt;
	myThread* spotThread;
	NiuRansacAlgo* niuransac;
	measureThread* mThread;
};
