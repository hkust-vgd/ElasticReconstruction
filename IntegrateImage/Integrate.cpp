#include "stream.h"

#include "IntegrateApp.h"

#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <boost/filesystem.hpp>

#include <iostream>
using namespace std;

int print_help ()
{
	cout << "\nApplication parameters:" << endl;
	cout << "    --help, -h                      : print this message" << endl;  
    cout << "    --ref_traj <log_file>           : use a reference trajectory file" << endl;
	cout << "    --pose_traj <log_file>          : use a pose trajectory file to create a reference trajectory" << endl;
	cout << "    --seg_traj <log_file>           : trajectory within each fragment - must have" << endl;
	cout << "    --ctr <ctr_file>                : enables distortion, must specify the following parameters" << endl;
	cout << "    --num <number>                  : number of pieces, important parameter" << endl;
	cout << "    --resolution <resolution>       : default - 8" << endl;
	cout << "    --length <length>               : default - 3.0" << endl;
	cout << "    --interval <interval>           : default - 50" << endl;
	cout << "    --camera <param_file>           : load camera parameters" << endl;
	cout << "    --save_to <pcd_file>            : output file, default - world.pcd" << endl;
	cout << "    --start_from <frame_id>         : frames before frame_id will be skipped" << endl;
	cout << "    --end_at <frame_id>             : frames after frame_id will be skipped" << endl;
	cout << "Valid depth data sources:" << endl; 
	cout << "    --folder <path>" << endl;
	return 0;
}

int main(int argc, char * argv[])
{
	using namespace pcl::console;

	if ( argc == 1 || find_switch( argc, argv, "--help" ) || find_switch( argc, argv, "-h" ) ) {
		return print_help ();
	}

	std::string folder;
	parse_argument(argc, argv, "--folder", folder);		// path that contains the depth folder that contains png files

	Increon::ImageFileStream stream;
	if (!stream.openFolder(folder)) {
		std::cout << "Error: cannot open " << folder << std::endl;
		return 1;
	}

	CIntegrateApp app;
	stream.registerDepthListener(&app);

	parse_argument( argc, argv, "--ref_traj", app.traj_filename_ );
	parse_argument( argc, argv, "--pose_traj", app.pose_filename_ );
	parse_argument( argc, argv, "--seg_traj", app.seg_filename_ );
	parse_argument( argc, argv, "--camera", app.camera_filename_ );
	parse_argument( argc, argv, "--save_to", app.pcd_filename_ );
	parse_argument(argc, argv,  "--blacklist", app.blacklist_filename_);
	parse_argument( argc, argv, "--start_from", app.start_from_ );
	parse_argument( argc, argv, "--end_at", app.end_at_ );
    parse_argument( argc, argv, "--volume_size", app.volume_size_);

	parse_argument( argc, argv, "--ctr", app.ctr_filename_ );	
	parse_argument( argc, argv, "--num", app.ctr_num_ );	
	parse_argument( argc, argv, "--resolution", app.ctr_resolution_ );	
	parse_argument( argc, argv, "--length", app.ctr_length_ );	
	parse_argument( argc, argv, "--interval", app.ctr_interval_ );	

	app.Init();

	stream.start();
	return 0;
}
