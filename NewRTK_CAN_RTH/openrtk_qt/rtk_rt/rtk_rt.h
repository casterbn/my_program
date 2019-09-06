#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_rtk_rt.h"

class rtk_rt : public QMainWindow
{
	Q_OBJECT

public:
	rtk_rt(QWidget *parent = Q_NULLPTR);

private:
	Ui::rtk_rtClass ui;
};
