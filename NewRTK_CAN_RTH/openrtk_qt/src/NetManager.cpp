#include "NetManager.h"
#include <iostream>
#include "LogicCenter.h"

NetManager * NetManager::m_instance = NULL;
std::once_flag      NetManager::m_flag;

NetManager::NetManager(QObject *parent)
	: QObject(parent)
{
	m_pNtripClient_1 = new NtripClient(this);
	m_pNtripClient_2 = new NtripClient(this);
	m_pNtripClient_3 = new NtripClient(this);
	m_pNtripClient_1->m_nType = 1;
	m_pNtripClient_2->m_nType = 2;
	m_pNtripClient_3->m_nType = 3;

	connect(m_pNtripClient_1, SIGNAL(connected()), this, SLOT(onClientConnected()));
	connect(m_pNtripClient_1, SIGNAL(readyRead()), this, SLOT(onClientReceiveData()));
	connect(m_pNtripClient_1, SIGNAL(disconnected()), this, SLOT(onClientDisconnect()));

	connect(m_pNtripClient_2, SIGNAL(connected()), this, SLOT(onClientConnected()));
	connect(m_pNtripClient_2, SIGNAL(readyRead()), this, SLOT(onClientReceiveData()));
	connect(m_pNtripClient_2, SIGNAL(disconnected()), this, SLOT(onClientDisconnect()));

	connect(m_pNtripClient_3, SIGNAL(connected()), this, SLOT(onClientConnected()));
	connect(m_pNtripClient_3, SIGNAL(readyRead()), this, SLOT(onClientReceiveData()));
	connect(m_pNtripClient_3, SIGNAL(disconnected()), this, SLOT(onClientDisconnect()));
}

NetManager::~NetManager()
{
}

NetManager* NetManager::getInstance()
{
	if (m_instance == NULL)
	{
		try
		{
			std::call_once(m_flag, createInstance);
		}
		catch (...)
		{
			std::cout << "CreateInstance error\n";
		}
		m_instance = new NetManager(NULL);
	}
	return m_instance;
}

void NetManager::createInstance()
{
	m_instance = new(std::nothrow) NetManager(NULL);//·ÖÅäÊ§°Ü£¬ÊÇ·µ»ØNULL;
	if (NULL == m_instance)
	{
		throw std::exception();
	}
}

void NetManager::onClientConnected()
{
	NtripClient *obj = (NtripClient*)sender();//get who send the signal
	if (obj == m_pNtripClient_1)
	{
		QString NtripData = QString("GET %1 HTTP/1.1\r\nUser-Agent: NTRIP JS Client/0.2\r\nAuthorization: Basic %2\r\n\r\n").arg(obj->m_sMountPoint, obj->getUserNameAndPasswordBase64());
		obj->write(NtripData.toUtf8());
	}
	if (obj == m_pNtripClient_2)
	{
		QString NtripData = QString("GET %1 HTTP/1.1\r\nUser-Agent: NTRIP JS Client/0.2\r\nAuthorization: Basic %2\r\n\r\n").arg(obj->m_sMountPoint, obj->getUserNameAndPasswordBase64());
		obj->write(NtripData.toUtf8());
	}
	if (obj == m_pNtripClient_3)
	{
		QString NtripData = QString("GET %1 HTTP/1.1\r\nUser-Agent: NTRIP JS Client/0.2\r\nAuthorization: Basic %2\r\n\r\n").arg(obj->m_sMountPoint, obj->getUserNameAndPasswordBase64());
		obj->write(NtripData.toUtf8());
	}
	emit sgnSetUIEnable(obj->m_nType, false);
}

void NetManager::setNtripClientConfig(int nIndex, QString sIp, int nPort, QString sMountPoint, QString sUserName, QString sPassword)
{
	switch (nIndex)
	{
	case 1:
	{
		m_pNtripClient_1->m_strIp = sIp;
		m_pNtripClient_1->m_nPort = nPort;
		m_pNtripClient_1->m_sMountPoint = sMountPoint;
		m_pNtripClient_1->m_sUserName = sUserName;
		m_pNtripClient_1->m_sPassword = sPassword;
	}
	break;
	case 2:
	{
		m_pNtripClient_2->m_strIp = sIp;
		m_pNtripClient_2->m_nPort = nPort;
		m_pNtripClient_2->m_sMountPoint = sMountPoint;
		m_pNtripClient_2->m_sUserName = sUserName;
		m_pNtripClient_2->m_sPassword = sPassword;
	}
	break;
	case 3:
	{
		m_pNtripClient_3->m_strIp = sIp;
		m_pNtripClient_3->m_nPort = nPort;
		m_pNtripClient_3->m_sMountPoint = sMountPoint;
		m_pNtripClient_3->m_sUserName = sUserName;
		m_pNtripClient_3->m_sPassword = sPassword;
	}
	break;
	}
}

void NetManager::startNtripClient(int nIndex)
{
	switch (nIndex)
	{
	case 1:
	{
		m_pNtripClient_1->doConnect();
	}
	break;
	case 2:
	{
		m_pNtripClient_2->doConnect();
	}
	break;
	case 3:
	{
		m_pNtripClient_3->doConnect();
	}
	break;
	}
}

void NetManager::stopNtripClient(int nIndex)
{
	switch (nIndex)
	{
	case 1:
	{
		m_pNtripClient_1->close();
	}
	break;
	case 2:
	{
		m_pNtripClient_2->close();
	}
	break;
	case 3:
	{
		m_pNtripClient_3->close();
	}
	break;
	}
}

bool NetManager::isAllPackageCompleted()
{
	if (m_pNtripClient_1->m_nRtcmPackageNum > 0 && m_pNtripClient_2->m_nRtcmPackageNum > 0 && m_pNtripClient_3->m_nRtcmPackageNum > 0)
	{
		return true;
	}
	return false;
}

bool NetManager::isFristRtkPackage()
{
	if (m_pNtripClient_1->m_nRtcmPackageNum > 0 && m_pNtripClient_2->m_nRtcmPackageNum == 0)
	{
		return true;
	}
	return false;
}

void NetManager::sendToRtkServer(QString& gga)
{
	m_pNtripClient_2->write(gga.toUtf8());
}

void NetManager::onClientDisconnect()
{
	NtripClient *obj = (NtripClient*)sender();//get who send the signal
	obj->m_nPackageNum = 0;
	obj->m_nRtcmPackageNum = 0;
	obj->m_receiveData.clear();
	emit sgnSetUIEnable(obj->m_nType, true);
}

void NetManager::onClientReceiveData()
{
	NtripClient *obj = (NtripClient*)sender();
	QByteArray byteArray = obj->readAll();
	QString msg = byteArray;
	obj->m_nPackageNum++;
	if (obj->m_nPackageNum == 1)//Skip first pagkage
	{
		return;
	}
	obj->m_receiveData.append(byteArray);
	if (obj == m_pNtripClient_1)
	{
		if (LogicCenter::getInstance()->transformRtcm(m_pNtripClient_1) == 1)
		{
			LogicCenter::getInstance()->processData();
		}
	}
	if (obj == m_pNtripClient_2)
	{
		if (LogicCenter::getInstance()->getModelPPPOrRTK() == emModelRTK)
		{
			if (LogicCenter::getInstance()->transformRtcm(m_pNtripClient_2))
			{
				LogicCenter::getInstance()->processData();
			}
		}
		else if (LogicCenter::getInstance()->getModelPPPOrRTK() == emModelPPP)
		{
			LogicCenter::getInstance()->transformRtcm(m_pNtripClient_2);
		}
	}
	if (obj == m_pNtripClient_3)
	{
		LogicCenter::getInstance()->transformRtcm(m_pNtripClient_3);
	}
}