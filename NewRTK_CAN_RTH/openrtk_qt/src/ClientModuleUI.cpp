#include "ClientModuleUI.h"
#include "LogicCenter.h"
#include "NetManager.h"

ClientModuleUI::ClientModuleUI(QWidget *parent)
	: QWidget(parent)
	, m_nStates(emStop)
{
	ui.setupUi(this);

	m_pSingleClientConfigUI_1 = new SingleClientConfigUI(this);
	m_pSingleClientConfigUI_1->setName("(1)Rover");
	m_pSingleClientConfigUI_2 = new SingleClientConfigUI(this);
	m_pSingleClientConfigUI_2->setName("(2)Base Station");
	m_pSingleClientConfigUI_3 = new SingleClientConfigUI(this);
	m_pSingleClientConfigUI_3->setName("(3)Correction");
	ui.gridLayout->addWidget(m_pSingleClientConfigUI_1, 0, 1);
	ui.gridLayout->addWidget(m_pSingleClientConfigUI_2, 0, 2);
	ui.gridLayout->addWidget(m_pSingleClientConfigUI_3, 0, 3);

	connect(ui.m_pushButtonStart, SIGNAL(clicked()), this, SLOT(OnStartOrStop()));
	connect(ui.m_pushButtonClearData, SIGNAL(clicked()), this, SLOT(OnClearData()));

	connect(LogicCenter::getInstance(), SIGNAL(sgnStreamDataLog(int, QString)), this, SLOT(OnShowDataLog(int, QString)));
	connect(NetManager::getInstance(), SIGNAL(sgnSetUIEnable(int, bool)), this, SLOT(OnUIEnable(int, bool)));
}

ClientModuleUI::~ClientModuleUI()
{
}

void ClientModuleUI::OnStartOrStop()
{
	if (m_nStates == emStop)
	{
		if (m_pSingleClientConfigUI_1->isChecked() == false
			&& m_pSingleClientConfigUI_2->isChecked() == false
			&& m_pSingleClientConfigUI_3->isChecked() == false)
		{
			return;
		}
		if (m_pSingleClientConfigUI_1->isChecked() && m_pSingleClientConfigUI_1->isAllCompleted())
		{
			NetManager::getInstance()->setNtripClientConfig(1, m_pSingleClientConfigUI_1->getIp(), 
				m_pSingleClientConfigUI_1->getPort(), m_pSingleClientConfigUI_1->getMountPoint(), 
				m_pSingleClientConfigUI_1->getUserName(), m_pSingleClientConfigUI_1->getPassword());
			NetManager::getInstance()->startNtripClient(1);
		}
		if (m_pSingleClientConfigUI_2->isChecked() && m_pSingleClientConfigUI_2->isAllCompleted())
		{
			NetManager::getInstance()->setNtripClientConfig(2, m_pSingleClientConfigUI_2->getIp(),
				m_pSingleClientConfigUI_2->getPort(), m_pSingleClientConfigUI_2->getMountPoint(),
				m_pSingleClientConfigUI_2->getUserName(), m_pSingleClientConfigUI_2->getPassword());
			NetManager::getInstance()->startNtripClient(2);
		}
		if (m_pSingleClientConfigUI_3->isChecked() && m_pSingleClientConfigUI_3->isAllCompleted())
		{
			NetManager::getInstance()->setNtripClientConfig(3, m_pSingleClientConfigUI_3->getIp(), 
				m_pSingleClientConfigUI_3->getPort(), m_pSingleClientConfigUI_3->getMountPoint(),
				m_pSingleClientConfigUI_3->getUserName(), m_pSingleClientConfigUI_3->getPassword());
			NetManager::getInstance()->startNtripClient(3);
		}
		m_nStates = emStart;
	}
	else if (m_nStates == emStart)
	{
		NetManager::getInstance()->stopNtripClient(1);
		NetManager::getInstance()->stopNtripClient(2);
		NetManager::getInstance()->stopNtripClient(3);
		m_nStates = emStop;
	}

	if (m_nStates == emStart)
	{
		if (ui.m_radioButton_RTK->isChecked())
		{
			LogicCenter::getInstance()->setModelPPPOrRTK(emModelRTK);
		}
		else if (ui.m_radioButton_PPP->isChecked())
		{
			LogicCenter::getInstance()->setModelPPPOrRTK(emModelPPP);
		}
		ui.m_pushButtonStart->setText("Stop");
		LogicCenter::getInstance()->initAllData();
		LogicCenter::getInstance()->openLogFile("resultlog");
	}
	else
	{
		ui.m_pushButtonStart->setText("Start");
		LogicCenter::getInstance()->closeLogFile();		
	}
}

void ClientModuleUI::OnClearData()
{
	ui.m_plainTextEditData_1->clear();
	ui.m_plainTextEditData_2->clear();
	ui.m_plainTextEditData_3->clear();
}

void ClientModuleUI::OnShowDataLog(int nIndex, QString log)
{
	switch (nIndex)
	{
	case 1:
		ui.m_plainTextEditData_1->appendPlainText(log.trimmed());
		break;
	case 2:
		ui.m_plainTextEditData_2->appendPlainText(log.trimmed());
		break;
	case 3:
		ui.m_plainTextEditData_3->appendPlainText(log.trimmed());
		break;
	}
}

void ClientModuleUI::OnUIEnable(int nIndex, bool enabel)
{
	switch (nIndex)
	{
	case 1:
		m_pSingleClientConfigUI_1->setEnabled(enabel);
		break;
	case 2:
		m_pSingleClientConfigUI_2->setEnabled(enabel);
		break;
	case 3:
		m_pSingleClientConfigUI_3->setEnabled(enabel);
		break;
	}
	if (m_pSingleClientConfigUI_1->isEnabled() && m_pSingleClientConfigUI_2->isEnabled() && m_pSingleClientConfigUI_3->isEnabled())
	{
		ui.m_radioButton_RTK->setEnabled(true);
		ui.m_radioButton_PPP->setEnabled(true);
	}
	else
	{
		ui.m_radioButton_RTK->setEnabled(false);
		ui.m_radioButton_PPP->setEnabled(false);
	}
}
