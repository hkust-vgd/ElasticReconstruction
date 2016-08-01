#pragma once
#include <boost/filesystem.hpp>

#include "stream.h"

#include <iostream>
#include <hash_set>
using namespace std;

//#define IMAGE_VIEWER     // for debug
#ifdef IMAGE_VIEWER
#include <pcl/visualization/image_viewer.h>
#endif

#include "TSDFVolume.h"
#include "ControlGrid.h"

struct SampledScopeTime : public pcl::StopWatch
{          
	enum { EACH = 33 };
	SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
	~SampledScopeTime() { 
		static int i_ = 0; 
		time_ms_ += pcl::StopWatch::getTime ();
		if ( i_ % EACH == 0 && i_ ) {
			cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
			time_ms_ = 0;        
		}
		++i_;
	}
private:    
	int & time_ms_;    
};

class CIntegrateApp : public Increon::DepthStreamListener
{
public:
	std::vector< unsigned short > depth_;
	std::vector< float > scaled_depth_;
	int cols_, rows_;

	bool use_device_;
	bool exit_;
	int frame_id_;
	int time_ms_;

	stdext::hash_set< int > blacklist_;
	std::string blacklist_filename_;

	RGBDTrajectory traj_;
	RGBDTrajectory seg_traj_;
	RGBDTrajectory pose_traj_;

	TSDFVolume volume_;
    double volume_size_;

	std::string traj_filename_;
	std::string pose_filename_;
	std::string seg_filename_;
	std::string camera_filename_;
	std::string pcd_filename_;

	// control grid parameters
	std::vector< ControlGrid > grids_;
	std::string ctr_filename_;
	int ctr_resolution_;
	int ctr_interval_;
	int ctr_num_;
	double ctr_length_;
	int start_from_;
	int end_at_;

private:
	std::vector< unsigned short > depth_buffer_;

#ifdef IMAGE_VIEWER
	pcl::visualization::ImageViewer viewer_depth_;
#endif

public:
	CIntegrateApp();
	~CIntegrateApp(void);

public:
	void Init();

private:
	void Execute( bool has_data );
	void Reproject();

	void onDepthFrame(Increon::DepthFrame &depth);
	void onDepthStreamEnd();
};

