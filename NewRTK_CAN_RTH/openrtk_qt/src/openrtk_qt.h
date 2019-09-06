#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_openrtk_qt.h"
#include "NtripClient.h"
#include "ClientModuleUI.h"
#include "LogModuleUI.h"
#include "SatelliteModuleUI.h"

class openrtk_qt : public QMainWindow
{
	Q_OBJECT

public:
	openrtk_qt(QWidget *parent = Q_NULLPTR);
    ~openrtk_qt();
	void removeAllDock();
	void setDefaultDockLayout();
private:
	Ui::openrtk_qtClass ui;

	ClientModuleUI* m_pClientModuleUI;
	LogModuleUI* m_pLogModuleUI;
	SatelliteModuleUI* m_pSatelliteModuleUI;
private slots:
	void OnSomeTestCode();
};
