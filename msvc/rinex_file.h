#pragma once

#include "ppprtk.h"
#include "rinex.h"
#include <vector>
#include <map>
#include <stdio.h>
#include <string>

struct obs_file
{
	double ver_;
	int sys_;
	int tsys_;
	char tobs[NUMSYS][MAXOBSTYPE][4];
	char type_;
	std::string file_name_;
	obs_t obss_;
	FILE* pf_;
	int read_ret;
	time_t time;//The closest second
	obs_file()
	{
		ver_ = 0.0f;
		sys_  = tsys_ = TSYS_GPS;
		type_ = ' ';
		memset(tobs, 0, NUMSYS*MAXOBSTYPE * 4);
		file_name_.clear();
		memset(&obss_, 0, sizeof(obs_t));
		pf_ = NULL;
		read_ret = 0;
		time = 0;
	}
};

class rinex_file
{
public:
	rinex_file();
	~rinex_file();

	void set_navigation_file_name(const char* filename);
	void set_observation_file_name(const char** filenames, int n);

	int read_navigation_rinex_file();
	void open_observation_rinex_files();
	void close_observation_rinex_files();
	int read_navigation_rinex(FILE *fp);
	//return true if have next
	bool read_next_epoch();

	int match_obs_list();
	int match_nav_list();
	void get_matched_ephemerides(nav_t& nav);
	void get_matched_obs_list(std::vector<obs_t*>& obs_list);
	time_t& getTime();
private:
	std::string navigation_file_name_;
	nav_t* pnavs_;								//navigation
	sta_t* psta_;
	time_t mintime_;
	std::vector<int> matched_indexes_;
	std::map<unsigned char, eph_t*> selected_ephs_;
	std::map<unsigned char, geph_t*> selected_gephs_;
	std::vector<obs_file*> obs_file_vector_;
};

