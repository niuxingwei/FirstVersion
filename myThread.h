#pragma once
#include<qthread.h>
#include <string>
#include"spotMeasure.h"
#include <qdebug.h>
#include"NiuRansac.h"
#include "measure.h"

using namespace std;
class myThread :public QThread// 接触斑测量线程
{
	Q_OBJECT
protected:
	void run();
public slots:
	void getPath(spotMethod*, string, string);// 获得主线程发送的输入输出路径

private:
	string input;// 获得输入路径
	string outPut;// 获得输出路径
	spotMethod* myso;

signals:
	void sendPathMessage();// 发送执行信号
	void sendMeasureThreadMessage2();// 发送线程结束信号
};

class measureThread : public QThread// 轮轨参数测量线程
{
	Q_OBJECT
protected:
	void run();
private:
	string inputPath;
	string outputPath;
	NiuRansacAlgo* niuRansac;
	Measure* mm;
public slots:
	void getMeasurePath(NiuRansacAlgo*, Measure*, string, string);// 获得主线程发送的基元检测以及轮轨参数测量数据
signals:
	void sendMeasureThreadMessage();// 发送信号告知主程序当前程序运行结束


};
//class detectThread : public QThread// 基元检测线程
//{
//	Q_OBJECT
//protected:
//	void run();
//
//};
//class progessBarThread : public QThread// 进度条控制线程
//{
//	Q_OBJECT
//protected:
//	void run();
//
//};