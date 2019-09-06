#include "openrtk_qt.h"
//#include "ppprtk.h"

openrtk_qt::openrtk_qt(QWidget *parent)
    : QMainWindow(parent)
{
	ui.setupUi(this);

	m_pClientModuleUI = new ClientModuleUI(ui.dockContents_client);
	ui.vertical_client->addWidget(m_pClientModuleUI);

	m_pLogModuleUI = new LogModuleUI(ui.dockContents_log);
	ui.vertical_log->addWidget(m_pLogModuleUI);

	m_pSatelliteModuleUI = new SatelliteModuleUI(ui.dockContents_satellite);
	ui.vertical_satellite->addWidget(m_pSatelliteModuleUI);
	
	QWidget* p = takeCentralWidget();
	if (p)
		delete p;

	setDockNestingEnabled(true);

	removeAllDock();
	setDefaultDockLayout();

	connect(ui.actionTest, SIGNAL(triggered()), this, SLOT(OnSomeTestCode()));
}

openrtk_qt::~openrtk_qt()
{

}

void openrtk_qt::removeAllDock()
{
	removeDockWidget(ui.dockWidget_client);
	removeDockWidget(ui.dockWidget_log);
	removeDockWidget(ui.dockWidget_satellite);
}

void openrtk_qt::setDefaultDockLayout()
{
	addDockWidget(Qt::LeftDockWidgetArea, ui.dockWidget_client);
	splitDockWidget(ui.dockWidget_client, ui.dockWidget_satellite, Qt::Horizontal);
	splitDockWidget(ui.dockWidget_client, ui.dockWidget_log, Qt::Vertical);

	ui.dockWidget_client->show();
	ui.dockWidget_satellite->show();
	ui.dockWidget_log->show();
}

void openrtk_qt::OnSomeTestCode()
{
}

