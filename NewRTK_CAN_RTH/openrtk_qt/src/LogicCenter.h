#pragma once

#include <QObject>
#include <thread>
#include <mutex>
#include "ppprtk.h"

enum ProcessModel
{
	emModelRTK = 1,
	emModelPPP = 2,
};

class NtripClient;
//LogicCenter is Singlton
class LogicCenter : public QObject
{
	Q_OBJECT

private:
	LogicCenter(QObject *parent);
	~LogicCenter();
public:
	static LogicCenter* getInstance();
	static void createInstance();
private:
	static LogicCenter* m_instance;
	static std::once_flag m_flag;

	rcv_rtk_t*   m_rtk;
	rcv_ppp_t*   m_ppp;
	ProcessModel m_processModel;

	rtcm_t  m_rtcm_cache[3];
	obs_t m_obs_rov;
	obs_t m_obs_ref;
	nav_t m_nav;
public:
	void initAllData();
	int transformRtcm(NtripClient* client);
	void getRtcmLog(int nIndex, QString & outIog);
	void getObsLog(obs_t & obs, QString & outIog);
	void streamDataLog(int nIndex, QString log);
	void processData();
	void processRtk();
	void processPPP();
	void setModelPPPOrRTK(ProcessModel model);
	ProcessModel getModelPPPOrRTK();
	void openLogFile(QString fileName);
	void closeLogFile();
signals:
	void sgnStreamDataLog(int nIndex, QString log);
	void sgnResultLog(QString type,QString log);
	void sgnShowSatelliteList(nav_t * nav);
	void sgnOpenLogFile(QString fileName);
	void sgnCloseLogFile();
};
