#include "myThread.h"

void myThread::run()
{
	if (myso == NULL) {
		qDebug() << "NULL";
	}

	myso->startTransform(input, outPut);
	emit sendMeasureThreadMessage2();
}

void myThread::getPath(spotMethod* mysopt, string inputPath, string outPutPath)
{
	if (mysopt == NULL) {
		qDebug() << "NULL2";
	}
	else {
		qDebug() << "NULL3";

	}

	input = inputPath;
	qDebug() << "ds";
	outPut = outPutPath;
	myso = mysopt;

}
void measureThread::run() {

	niuRansac->measure_wheelrail(inputPath, outputPath);

	mm->startMeasure(outputPath);
	emit sendMeasureThreadMessage();
}
void measureThread::getMeasurePath(NiuRansacAlgo* n, Measure* m, string input, string output) {
	if (n == NULL) {
		qDebug() << "NiuRansacAlgo NULL";
	}
	else {
		qDebug() << "NiuRansacAlgo不为空 ";

	}
	if (m == NULL) {
		qDebug() << "Measure NULL";
	}
	else {
		qDebug() << "Measure不为空 ";

	}
	niuRansac = n;
	mm = m;
	inputPath = input;
	outputPath = output;
}