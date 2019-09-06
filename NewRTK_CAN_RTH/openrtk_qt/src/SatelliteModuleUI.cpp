#include "SatelliteModuleUI.h"
#include "LogicCenter.h"

SatelliteModuleUI::SatelliteModuleUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	m_SatelliteList.clear();

	ui.m_satelliteTable->setColumnWidth(0, 48);
	ui.m_satelliteTable->horizontalHeader()->setStretchLastSection(true);

	connect(LogicCenter::getInstance(), SIGNAL(sgnShowSatelliteList(nav_t*)), this, SLOT(OnShowSatelliteList(nav_t*)));
}

SatelliteModuleUI::~SatelliteModuleUI()
{
}

void SatelliteModuleUI::OnShowSatelliteList(nav_t * nav)
{
	m_SatelliteList.clear();
	for (int i = 0; i < MAXEPH; i++)
	{
		if (nav->eph[i].sat > 0)
		{
			struct tm Tm = { 0 };
			gmtime_s(&Tm, &(nav->eph[i].ttr.time));

			QString str;
			str.sprintf("%04d/%02d/%02d %02d:%02d:%02d ", Tm.tm_year + 1900, Tm.tm_mon + 1, Tm.tm_mday, Tm.tm_hour + 8, Tm.tm_min, Tm.tm_sec);
			int sat = (int)nav->eph[i].sat;
			if (m_SatelliteList.contains(sat)) continue;
			m_SatelliteList.insert(sat, str);
		}
	}
	for (int i = 0; i < MAXEPH_R; i++)
	{
		if (nav->geph[i].sat > 0)
		{
			struct tm Tm = { 0 };
			gmtime_s(&Tm, &(nav->geph[i].tof.time));

			QString str;
			str.sprintf("%04d/%02d/%02d %02d:%02d:%02d ", Tm.tm_year + 1900, Tm.tm_mon + 1, Tm.tm_mday, Tm.tm_hour + 8, Tm.tm_min, Tm.tm_sec);
			int sat = (int)nav->geph[i].sat;
			if (m_SatelliteList.contains(sat)) continue;
			m_SatelliteList.insert(sat, str);
		}
	}

	ui.m_satelliteTable->clearContents();
	ui.m_satelliteTable->setRowCount(0);
	QMap<int, QString>::iterator it;
	for (it = m_SatelliteList.begin(); it != m_SatelliteList.end(); ++it)
	{
		int nRow = ui.m_satelliteTable->rowCount();
		ui.m_satelliteTable->insertRow(nRow);
		ui.m_satelliteTable->setItem(nRow, 0, new QTableWidgetItem(QString::number(it.key())));
		ui.m_satelliteTable->setItem(nRow, 1, new QTableWidgetItem(it.value()));
	}
}