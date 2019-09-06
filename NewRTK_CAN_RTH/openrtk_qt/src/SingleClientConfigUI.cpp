#include "SingleClientConfigUI.h"

SingleClientConfigUI::SingleClientConfigUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

SingleClientConfigUI::~SingleClientConfigUI()
{
}

void SingleClientConfigUI::setName(QString name)
{
	ui.m_checkBoxStream->setText(name);
}

bool SingleClientConfigUI::isChecked()
{
	return ui.m_checkBoxStream->isChecked();
}

bool SingleClientConfigUI::isAllCompleted()
{
	if (ui.m_comboBoxHost->currentText().trimmed().isEmpty()
		|| ui.m_lineEditPort->text().trimmed().isEmpty()
		|| ui.m_comboBoxMountpoint->currentText().trimmed().isEmpty())
	{
		return false;
	}
	return true;
}

QString SingleClientConfigUI::getIp()
{
	return  ui.m_comboBoxHost->currentText().trimmed();
}

int SingleClientConfigUI::getPort()
{
	return ui.m_lineEditPort->text().trimmed().toInt();
}

QString SingleClientConfigUI::getMountPoint()
{
	return ui.m_comboBoxMountpoint->currentText().trimmed();
}

QString SingleClientConfigUI::getUserName()
{
	return ui.m_lineEditUser->text().trimmed();
}

QString SingleClientConfigUI::getPassword()
{
	return ui.m_lineEditPassword->text().trimmed();
}