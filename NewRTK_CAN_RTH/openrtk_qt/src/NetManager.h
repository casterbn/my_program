#pragma once

#include <QObject>
#include <thread>
#include <mutex>
#include "NtripClient.h"

class NetManager : public QObject
{
	Q_OBJECT

public:
	NetManager(QObject *parent);
	~NetManager();
public:
	static NetManager* getInstance();
	static void createInstance();

private:
	static NetManager* m_instance;
	static std::once_flag m_flag;

	NtripClient* m_pNtripClient_1;
	NtripClient* m_pNtripClient_2;
	NtripClient* m_pNtripClient_3;
public:
	void setNtripClientConfig(int nIndex, QString sIp,int nPort, QString sMountPoint, QString sUserName, QString sPassword);
	void startNtripClient(int nIndex);
	void stopNtripClient(int nIndex);
	bool isAllPackageCompleted();
	bool isFristRtkPackage();
	void sendToRtkServer(QString& gga);
public slots:
	void onClientConnected();
	void onClientDisconnect();
	void onClientReceiveData();
signals:
	void sgnSetUIEnable(int nIndex, bool enable);
};
