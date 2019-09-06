#pragma once

#include <QWidget>
#include "ui_ClientModuleUI.h"
#include "SingleClientConfigUI.h"
#include "NtripClient.h"

enum emStates
{
	emStart,
	emStop,
};

class ClientModuleUI : public QWidget
{
	Q_OBJECT

public:
	ClientModuleUI(QWidget *parent = Q_NULLPTR);
	~ClientModuleUI();

private:
	Ui::ClientModuleUI ui;
	emStates m_nStates;

	SingleClientConfigUI* m_pSingleClientConfigUI_1;
	SingleClientConfigUI* m_pSingleClientConfigUI_2;
	SingleClientConfigUI* m_pSingleClientConfigUI_3;

public slots:
	void OnStartOrStop();
	void OnClearData();
	void OnShowDataLog(int nIndex, QString log);
	void OnUIEnable(int nIndex, bool enabel);
};
