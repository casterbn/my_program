#pragma once

#include <QObject>
#include <QTcpSocket>
#include "rtcm.h"

class NtripClient : public QTcpSocket
{
	Q_OBJECT

public:
	NtripClient(QObject *parent);
	~NtripClient();

	void doConnect();
	QString getUserNameAndPasswordBase64();
public:
	int m_nType;
	QString m_strIp;
	int m_nPort;
	QString m_sMountPoint;
	QString m_sUserName;
	QString m_sPassword;

	int m_nPackageNum;//how many packeage received
	QByteArray m_receiveData;
	int m_nRtcmPackageNum;//how many rtcm package decoded
};
