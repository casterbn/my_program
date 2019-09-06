#include "rinex_file.h"

#define SECOND_PER_HOUR 3600

rinex_file::rinex_file()
{
	navigation_file_name_.clear();
	pnavs_ = new nav_t();
	psta_ = new sta_t();
	memset(pnavs_, 0, sizeof(nav_t));
	obs_file_vector_.clear();
	mintime_ = 0;
	matched_indexes_.clear();
}

rinex_file::~rinex_file()
{
	delete pnavs_; pnavs_ = NULL;
	delete psta_; psta_ = NULL;
	for (int i = 0; i < obs_file_vector_.size(); i++)
	{
		delete obs_file_vector_[i];
	}
	obs_file_vector_.clear();
}

void rinex_file::set_navigation_file_name(const char * filename)
{
	navigation_file_name_ = filename;
}

void rinex_file::set_observation_file_name(const char ** filenames, int n)
{
	for (int i = 0; i < n; i++)
	{
		obs_file* obs_file_ = new obs_file();
		obs_file_->file_name_ = filenames[i];
		obs_file_vector_.push_back(obs_file_);
	}
}

int rinex_file::read_navigation_rinex_file()
{
	FILE *fp;
	int stat;

	trace(3, "readrnxfile: file=%s \n", navigation_file_name_.c_str());

	if (!(fp = fopen(navigation_file_name_.c_str(), "r"))) {
		trace(1, "File opening failed,file=%s", navigation_file_name_.c_str());
		return 0;
	}
	/* read rinex file */
	stat = read_navigation_rinex(fp);

	fclose(fp);

	return stat;
}

void rinex_file::open_observation_rinex_files()
{
	//open files
	for (int i = 0; i < obs_file_vector_.size(); i++)
	{
		obs_file* pobsfile = obs_file_vector_[i];
		pobsfile->pf_ = fopen(pobsfile->file_name_.c_str(), "r");
		if (!pobsfile->pf_) {
			trace(1,"File opening failed,file=%s", pobsfile->file_name_.c_str());
			continue;
		}
		/* read rinex header */
		if (!readrnxh(pobsfile->pf_, &pobsfile->ver_, &pobsfile->type_, &pobsfile->sys_, &pobsfile->tsys_, pobsfile->tobs, pnavs_, psta_))
		{
			trace(2, "unsupported rinex type ver=%.2f type=%c\n", pobsfile->ver_, pobsfile->type_);
			break;
		}
	}
}

void rinex_file::close_observation_rinex_files()
{
	//close filess
	for (int i = 0; i < obs_file_vector_.size(); i++)
	{
		obs_file* pobsfile = obs_file_vector_[i];
		fclose(pobsfile->pf_);
	}
}

int rinex_file::read_navigation_rinex(FILE *fp)
{
	double ver;
	int sys, tsys = TSYS_GPS;
	char tobs[NUMSYS][MAXOBSTYPE][4] = { {""} };
	char type = ' ';
	sta_t* sta = NULL;

	/* read rinex header */
	if (!readrnxh(fp, &ver, &type, &sys, &tsys, tobs, pnavs_, sta)) return 0;

	/* read rinex body */
	switch (type)
	{
	case 'N': return readrnxnav(fp, ver, sys, pnavs_);
	case 'G': return readrnxnav(fp, ver, _SYS_GLO_, pnavs_);
	}
	trace(2, "unsupported rinex type ver=%.2f type=%c\n", ver, type);
	return 0;
}

bool rinex_file::read_next_epoch()
{
	int eof_count = 0;
	for (int i = 0; i < obs_file_vector_.size(); i++)
	{
		obs_file* pobsfile = obs_file_vector_[i];
		//trace(3, "readrnxfile: file=%s \n", pobsfile->file_name_.c_str());
		if (pobsfile->read_ret >= 0 && pobsfile->time <= mintime_)
		{
			pobsfile->read_ret = readrnxobs_one(pobsfile->pf_, pobsfile->ver_, &pobsfile->tsys_, pobsfile->tobs, &(pobsfile->obss_), psta_);
		}
		if (pobsfile->read_ret == -1)
		{
			eof_count++;
		}
	}
	if (obs_file_vector_.size() == eof_count)
	{
		return false;
	}
	return true;
}

int rinex_file::match_obs_list()
{
	mintime_ = time(NULL);

	for (int i = 0; i < obs_file_vector_.size(); i++)
	{
		obs_file* pobsfile = obs_file_vector_[i];
		if (pobsfile->obss_.time.sec > 0.5)
		{
			pobsfile->time = pobsfile->obss_.time.time + 1;
		}
		else
		{
			pobsfile->time = pobsfile->obss_.time.time;
		}
		mintime_ = pobsfile->time < mintime_ ? pobsfile->time : mintime_;
	}

	matched_indexes_.clear();

	for (int i = 0; i < obs_file_vector_.size(); i++)
	{
		obs_file* pobsfile = obs_file_vector_[i];
		if (pobsfile->time == mintime_)
		{
			matched_indexes_.push_back(i);
		}
	}

	return matched_indexes_.size();
}

int rinex_file::match_nav_list()
{
	selected_ephs_.clear();
	std::map<unsigned char, eph_t*>::iterator it;
	for (int eph_i = 0; eph_i < MAXEPH; eph_i++)// loop satellite in pnavs_
	{
		unsigned char sat = pnavs_->eph[eph_i].sat;
		if (sat == 0) continue;

		for (int matched_i = 0; matched_i < matched_indexes_.size(); matched_i++)
		{
			int index = matched_indexes_[matched_i];
			obs_file* pobsfile = obs_file_vector_[index];
			if (abs(pobsfile->obss_.time.time - pnavs_->eph[eph_i].toe.time) < SECOND_PER_HOUR * 2)//in two hours
			{
				for (int sat_i = 0; sat_i < MAXOBS; sat_i++)// loop and find the satellite
				{
					if(pobsfile->obss_.data[sat_i].sat == pnavs_->eph[eph_i].sat)// same satellite
					{
						it = selected_ephs_.find(sat); //duplicate removal
						if (it == selected_ephs_.end())
						{
							selected_ephs_[sat] = &(pnavs_->eph[eph_i]);
							break;
						}
					}
				}
			}
		}
	}

	selected_gephs_.clear();
	std::map<unsigned char, geph_t*>::iterator it_g;
	for (int geph_i = 0; geph_i < MAXEPH_R; geph_i++)// loop satellite in pnavs_
	{
		unsigned char sat = pnavs_->geph[geph_i].sat;
		if (sat == 0) continue;

		for (int matched_i = 0; matched_i < matched_indexes_.size(); matched_i++)
		{
			int index = matched_indexes_[matched_i];
			obs_file* pobsfile = obs_file_vector_[index];
			for (int sat_i = 0; sat_i < MAXOBS; sat_i++)// loop and find the satellite
			{
				if (pobsfile->obss_.data[sat_i].sat == sat)// same satellite
				{
					it = selected_ephs_.find(sat); //duplicate removal
					if (it == selected_ephs_.end())
					{
						selected_gephs_[sat] = &(pnavs_->geph[geph_i]);
						break;
					}
				}
			}
		}
	}

	return selected_ephs_.size() + selected_gephs_.size();
}

void rinex_file::get_matched_ephemerides(nav_t& nav)
{
	std::map<unsigned char, eph_t*>::iterator it;
	int index = 0;
	for (it = selected_ephs_.begin(); it != selected_ephs_.end(); it++)
	{
		nav.eph[index] = *(eph_t*)(it->second);
		index++;
	}
	std::map<unsigned char, geph_t*>::iterator it_g;
	index = 0;
	for (it_g = selected_gephs_.begin(); it_g != selected_gephs_.end(); it_g++)
	{
		nav.geph[index] = *(geph_t*)(it_g->second);
		index++;
	}

	nav.n = selected_ephs_.size();
	nav.ng = selected_gephs_.size();
	nav.n_gps = pnavs_->n_gps;
	nav.n_gal = pnavs_->n_gal;
	nav.n_bds = pnavs_->n_bds;
	nav.n_qzs = pnavs_->n_qzs;
	nav.ns = pnavs_->ns;
}

void rinex_file::get_matched_obs_list(std::vector<obs_t*>& obs_list)
{
	for (int i = 0; i < matched_indexes_.size(); i++)
	{
		int index = matched_indexes_[i];
		obs_file* pobsfile = obs_file_vector_[index];
		if (i > 0)
		{
			for (int j = 0; j < 3; j++)
			{
				pobsfile->obss_.pos[j] = psta_->pos[j];
			}
		}
		obs_list.push_back(&pobsfile->obss_);
	}
}

time_t& rinex_file::getTime()
{
	return mintime_;
}
