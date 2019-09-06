#include "openrtk_qt.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	openrtk_qt w;
	w.show();
	return a.exec();
}
