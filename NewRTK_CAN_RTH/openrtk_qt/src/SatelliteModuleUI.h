#pragma once

#include <QWidget>
#include "ui_SatelliteModuleUI.h"

#include "rtcm.h"

class SatelliteModuleUI : public QWidget
{
	Q_OBJECT

public:
	SatelliteModuleUI(QWidget *parent = Q_NULLPTR);
	~SatelliteModuleUI();

private:
	Ui::SatelliteModuleUI ui;
	QMap<int, QString> m_SatelliteList;

public slots:
	void OnShowSatelliteList(nav_t * nav);
private:

};
