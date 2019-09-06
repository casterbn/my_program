#pragma once

#include <QWidget>
#include "ui_LogModuleUI.h"

class LogModuleUI : public QWidget
{
	Q_OBJECT

public:
	LogModuleUI(QWidget *parent = Q_NULLPTR);
	~LogModuleUI();

private:
	Ui::LogModuleUI ui;
	QFile m_file_gga;
	QFile m_file_sol;
public slots:
	void onAppandLog(QString type, QString log);
	void onOpenLogFile(QString fileName);
	void onCloseLogFile();
};
