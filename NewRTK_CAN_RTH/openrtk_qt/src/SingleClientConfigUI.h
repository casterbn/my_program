#pragma once

#include <QWidget>
#include "ui_SingleClientConfigUI.h"

class SingleClientConfigUI : public QWidget
{
	Q_OBJECT

public:
	SingleClientConfigUI(QWidget *parent = Q_NULLPTR);
	~SingleClientConfigUI();

	void setName(QString name);
	bool isChecked();
	bool isAllCompleted();
	QString getIp();
	int getPort();
	QString getMountPoint();
	QString getUserName();
	QString getPassword();
private:
	Ui::SingleClientConfigUI ui;
};
