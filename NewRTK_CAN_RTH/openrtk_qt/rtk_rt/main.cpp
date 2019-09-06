#include "rtk_rt.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	rtk_rt w;
	w.show();
	return a.exec();
}
