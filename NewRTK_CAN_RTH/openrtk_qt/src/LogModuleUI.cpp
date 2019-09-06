#include "LogModuleUI.h"
#include "LogicCenter.h"
#include <QDateTime>

LogModuleUI::LogModuleUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	connect(LogicCenter::getInstance(), SIGNAL(sgnResultLog(QString,QString)),this,SLOT(onAppandLog(QString,QString)));
	connect(LogicCenter::getInstance(), SIGNAL(sgnOpenLogFile(QString)), this, SLOT(onOpenLogFile(QString)));
	connect(LogicCenter::getInstance(), SIGNAL(sgnCloseLogFile()), this, SLOT(onCloseLogFile()));
}

LogModuleUI::~LogModuleUI()
{
}

void LogModuleUI::onOpenLogFile(QString fileName)
{
	QDateTime currentTime = QDateTime::currentDateTime();
	QString strTime = currentTime.toString("yyyy-MM-dd hh-mm-ss");
	m_file_gga.setFileName("gga_"+strTime+".log");
	m_file_gga.open(QIODevice::WriteOnly | QIODevice::Text);

	m_file_sol.setFileName("sol_" + strTime + ".log");
	m_file_sol.open(QIODevice::WriteOnly | QIODevice::Text);
}

void LogModuleUI::onCloseLogFile()
{
	m_file_gga.close();
	m_file_sol.close();
}

void LogModuleUI::onAppandLog(QString type,QString log)
{
	
	if (type == "gga")
	{
		if (m_file_gga.isOpen())
		{
			m_file_gga.write(log.toUtf8() + "\n");
		}
		ui.m_plainTextEditResult->appendPlainText(log.trimmed());
	}

	if (type == "sol")
	{
		if (m_file_sol.isOpen())
		{
			m_file_sol.write(log.toUtf8() + "\n");
		}
	}
}
