#include "LogicCenter.h"
#include <iostream>
#include "NetManager.h"

LogicCenter * LogicCenter::m_instance = NULL;
std::once_flag      LogicCenter::m_flag;

LogicCenter::LogicCenter(QObject *parent)
	: QObject(parent)
	,m_processModel(emModelRTK)
{
	m_rtk = new rcv_rtk_t();
	m_ppp = new rcv_ppp_t();

	initAllData();
}

LogicCenter::~LogicCenter()
{
	delete m_rtk;
	m_rtk = NULL;

	delete m_ppp;
	m_ppp = NULL;
	//if (m_instance != NULL)
	//{
	//	delete m_instance;
	//	m_instance = NULL;
	//}
}

LogicCenter* LogicCenter::getInstance()
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
 		m_instance = new LogicCenter(NULL);
	}
	return m_instance;
}

void LogicCenter::createInstance()
{
	m_instance = new(std::nothrow) LogicCenter(NULL);//·ÖÅäÊ§°Ü£¬ÊÇ·µ»ØNULL;
	if (NULL == m_instance)
	{
		throw std::exception();
	}
}

void LogicCenter::initAllData()
{
	memset(m_rtk, 0, sizeof(rcv_rtk_t));
	memset(m_ppp, 0, sizeof(rcv_ppp_t));
	memset(m_rtcm_cache, 0, sizeof(rtcm_t) * 3);
	memset(&m_obs_rov, 0, sizeof(obs_t));
	memset(&m_obs_ref, 0, sizeof(obs_t));
	memset(&m_nav, 0, sizeof(nav_t));
}

int LogicCenter::transformRtcm(NtripClient * client)
{
	int ret = 0;

	//while (true)
	//{
		for (int i = 0; i < client->m_receiveData.size(); i++)
		{
			char buff = client->m_receiveData[i];

			switch (client->m_nType)
			{
			case 1:
				ret = input_rtcm3_data(&m_rtcm_cache[0], buff, &m_obs_rov, &m_nav);
				break;
			case 2:
				if (m_processModel == emModelRTK)
				{
					ret = input_rtcm3_data(&m_rtcm_cache[1], buff, &m_obs_ref, &m_nav);
				}
				else if (m_processModel == emModelPPP)
				{
					ret = input_rtcm3_data(&m_rtcm_cache[1], buff, &m_obs_rov, &m_nav);
				}
				break;
			case 3:
				ret = input_rtcm3_data(&m_rtcm_cache[2], buff, &m_obs_rov, &m_nav);
				break;
			}
			
			if (ret == 1 || ret == 5 || ret == 2 || ret == 10)
			{
				//read a full package
				client->m_nRtcmPackageNum++;
				QString outIog;
				getRtcmLog(client->m_nType, outIog);
				//write log
				streamDataLog(client->m_nType, outIog);
				//Delete processed data
				//client->m_receiveData.remove(0, i + 1);

				if (client->m_nType == 1 && ret == 1)
				{
					processData();
				}
				
				//break;
			}
			//else if (ret != 0 && ret != -1)
			//{
			//	break;
			//}
		}
		client->m_receiveData.clear();

		//If you can't read a complete package or the cache is empty then break
		//if ((ret != 1 && ret != 5  && ret != 2 && ret != 10) || client->m_receiveData.size() == 0)
		//{
		//	client->m_receiveData.clear();
		//	break;
		//}
	//}
	return ret;

}

void LogicCenter::getRtcmLog(int nIndex, QString& outIog)
{
	switch (nIndex)
	{
	case 1:
	{
		getObsLog(m_obs_rov, outIog);
	}
	break;
	case 2:
	{
		getObsLog(m_obs_ref, outIog);
	}
	break;
	case 3:
	{
		outIog.sprintf("G:%i, R:%i, E:%i, C:%i, SSR:%i\n", m_nav.n_gps, m_nav.ng, m_nav.n_gal, m_nav.n_bds, m_nav.ns);
		emit sgnShowSatelliteList(&m_nav);
	}
	break;
	}
}

void LogicCenter::getObsLog(obs_t& obs, QString & outIog)
{
	QString temp_sat_str;
	QString time_str;

	struct tm Tm = { 0 };
	gmtime_s(&Tm, &(obs.time.time));
	time_str.sprintf("%04d/%02d/%02d %02d:%02d:%02d ", Tm.tm_year + 1900, Tm.tm_mon + 1, Tm.tm_mday, Tm.tm_hour + 8, Tm.tm_min, Tm.tm_sec);

	outIog.sprintf("%i", obs.n);
	outIog.append("[");
	for (int i = 0; i < MAXOBS; i++)
	{
		temp_sat_str.clear();
		if (obs.data[i].sat == 0)break;
		if (i == obs.n)
		{
			outIog.append("|");
		}
		if (i == 0)
			temp_sat_str.sprintf("%d", obs.data[i].sat);
		else
			temp_sat_str.sprintf(",%d", obs.data[i].sat);

		outIog.append(temp_sat_str);
	}
	outIog.append("] ");
	//outIog = time_str + outIog;
}

void LogicCenter::streamDataLog(int nIndex, QString log)
{
	if (log.trimmed().isEmpty())
	{
		return;
	}
	emit sgnStreamDataLog(nIndex, log);
}

void LogicCenter::processData()
{
	if (m_processModel == emModelRTK)
	{
		processRtk();
	}
	else if (m_processModel == emModelPPP)
	{
		processPPP();
	}
}

void LogicCenter::processRtk()
{
	if (NetManager::getInstance()->isAllPackageCompleted())
	{
		int isPrint = 0, nsd = 0;
		char gga[1024] = { 0 }, sol[1024] = { 0 };
		QString logInfo;

		nsd = rtk_processor(&m_obs_rov, &m_obs_ref, &m_nav, m_rtk, gga, sol, isPrint);

		//ui.m_textMatchedSatellite->appendPlainText(QString::number(nsd));

		if (nsd > 0)
		{
			/*struct tm Tm = { 0 };
			gmtime_s(&Tm, &(m_obs_rov.time.time));*/
			//logInfo.sprintf("%04d/%02d/%02d %02d:%02d:%02d, rov:%d, ref:%d, %f, %f, %f \n", Tm.tm_year + 1900, Tm.tm_mon + 1, Tm.tm_mday, Tm.tm_hour + 8, Tm.tm_min, Tm.tm_sec, m_obs_rov.n, m_obs_ref.n, m_rtk->x[0], m_rtk->x[1], m_rtk->x[2]);
			
			logInfo = gga;	
		}
		else {
			print_nmea_gga(m_obs_rov.time, m_obs_rov.pos, m_obs_rov.n, 1, 1.0, 0.0, gga);
			logInfo = gga;
		}

		if (logInfo.trimmed().isEmpty() == false)
		{
			emit sgnResultLog("gga",logInfo.trimmed());
		}

		/*logInfo = sol;
		if (logInfo.trimmed().isEmpty() == false)
		{
			emit sgnResultLog("sol", logInfo.trimmed());
		}*/

	}
}

void LogicCenter::processPPP()
{
	if (NetManager::getInstance()->isAllPackageCompleted())
	{
		if (m_nav.n == 0) return;

		/*char gga[1024] = { 0 }, sol[1024] = { 0 };
		ppp_processor(&m_obs_rov, &m_nav, m_ppp, gga, sol,0);*/

		struct tm Tm = { 0 };
		gmtime_s(&Tm, &m_obs_rov.time.time);
		QString logInfo;
		logInfo.sprintf("%04d/%02d/%02d %02d:%02d:%02d, rov:%d, %f, %f, %f \n", Tm.tm_year + 1900, Tm.tm_mon + 1, Tm.tm_mday, Tm.tm_hour + 8, Tm.tm_min, Tm.tm_sec, m_obs_rov.n, m_ppp->x[0], m_ppp->x[1], m_ppp->x[2]);

		if (logInfo.trimmed().isEmpty() == false)
		{
			emit sgnResultLog("", logInfo.trimmed());
		}
	}
}

void LogicCenter::setModelPPPOrRTK(ProcessModel model)
{
	m_processModel = model;
}

ProcessModel LogicCenter::getModelPPPOrRTK()
{
	return m_processModel;
}

void LogicCenter::openLogFile(QString fileName)
{
	OpenLogFile();
	emit sgnOpenLogFile(fileName);
}

void LogicCenter::closeLogFile()
{
	CloseLogFile();
	emit sgnCloseLogFile();
}