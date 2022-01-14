#include "paramemeasure.h"
#include <QtWidgets/QApplication>
#include "vtkAutoInit.h"
#include "measure.h"
#include "spotMeasure.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	QPixmap pixmap("./startScrreen.png");
	QSplashScreen* splash = new QSplashScreen;
	splash->setPixmap(pixmap);
	splash->show();
	a.processEvents();
	QDateTime n = QDateTime::currentDateTime();
	QDateTime now;
	do {
		now = QDateTime::currentDateTime();
	} while (n.secsTo(now) <= 1);//3为需要延时的秒数

	ParameMeasure w;
	w.show();
	splash->finish(&w);
	delete splash;
	return a.exec();
}
