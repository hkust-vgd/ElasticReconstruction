#include "helper.h"

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
using namespace std;

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;


std::string dir_name = "";

int fragment;
RGBDTrajectory segment_traj;        // 100-0.log
RGBDTrajectory init_traj;           // init.log, per frame
RGBDTrajectory pose_traj;           // per fragment
RGBDTrajectory align_traj;          // pairwise
RGBDTrajectory odometry_traj;
RGBDInformation odometry_info;
vector<bool> visited;

void visualize( int num )
{
	Configuration config;

	RGBDTrajectory traj;
	RGBDInformation info;

    vector<PointCloudT::Ptr> fragments;
    for ( int i = 0; i < num; i++ ) {
        PointCloudT::Ptr object (new PointCloudT);
		
		pcl::console::print_highlight ("Loading point clouds...\n");

		char filename[ 1024 ];
		sprintf( filename, "%scloud_bin_%d.pcd", dir_name.c_str(), i );
		pcl::io::loadPCDFile<PointT>( filename, *object );

        const float leaf = config.resample_leaf_;

		// Downsample
		pcl::console::print_highlight ("Downsampling...\n");
		pcl::VoxelGrid<PointT> grid;
		grid.setLeafSize (leaf, leaf, leaf);
		grid.setInputCloud (object);
		grid.filter (*object);

        fragments.push_back(object);
    }

    // Display
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    double color[][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255} };
    for ( int i = 0; i < num; i++ ) {
        ostringstream oss;
        oss << "cloud " << i;
        string id = oss.str();
        
        if (visited[i] == false) continue;      // has no pose

        Eigen::Matrix4d pose = pose_traj.data_[i].transformation_;      // local to world

        PointCloudT::Ptr world_fragment (new PointCloudT);
        pcl::transformPointCloud (*fragments[i], *world_fragment, pose);

        viewer->addPointCloud(world_fragment, id);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);
        double *rgb = color[i % 6];
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], id); 
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, id);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void create_init_traj()
{
	init_traj.data_.clear();
	Eigen::Matrix4d base_mat = Eigen::Matrix4d::Identity();
	for ( size_t i = 0; i < segment_traj.data_.size(); i++ ) {
		if ( i % fragment == 0 && i > 0 ) {
			base_mat = init_traj.data_[ i - 1 ].transformation_ * segment_traj.data_[ i ].transformation_.inverse();
		}
		init_traj.data_.push_back( FramedTransformation( i, i, i + 1, base_mat * segment_traj.data_[ i ].transformation_ ) );
	}
}

void create_pose_traj( int num_frags )
{
    // build world pose of each fragment based on pairwise alignment
	// no optimization is considered
    pose_traj.data_.clear();
    visited.resize(num_frags);
    for (int i = 0; i < num_frags; ++i) {
        visited[i] = false;
        pose_traj.data_.push_back( FramedTransformation( i, i, num_frags, Eigen::Matrix4d::Identity() ) );
    }
    int index = 0;
    bool first = true;
    while (index < num_frags) {
        
        for ( size_t k = 0; k < align_traj.data_.size(); ++k) {
            FramedTransformation &frame = align_traj.data_[k];
            if (frame.id1_ == index) {
                if (first) {
                    first = false;
                    visited[frame.id1_] = true;
                }
                
                if (visited[frame.id2_] == false) {
                    // frame.transformation_ transform point cloud id2 to id1
                    
                    visited[frame.id2_] = true;

                    pose_traj.data_[frame.id2_].transformation_ = 
                        pose_traj.data_[frame.id1_].transformation_ * frame.transformation_;
                    visited[frame.id2_] = true;
                }
            }
        }

        index++;
    }
}

struct FramedTransformationPredicate {
    bool operator()(FramedTransformation &a, FramedTransformation &b) {
        return (a.id1_ < b.id1_ || (a.id1_ == b.id1_ && a.id2_ < b.id2_));
    }
};

void create_align_traj() {
    align_traj.LoadFromFile( "result.txt" );

    std::sort(align_traj.data_.begin(), align_traj.data_.end(), FramedTransformationPredicate());
}

int main(int argc, char * argv[])
{
	if ( argc < 2 ) {
		cout << "Usage : " << endl;
		cout << "    FragmentVisualizer.exe <dir>" << endl;
		cout << "    FragmentVisualizer.exe <dir> <100-0.log> <segment_length>" << endl;
		return 0;
	}
	dir_name = std::string( argv[ 1 ] );
	int num_of_pcds =  std::count_if( boost::filesystem::directory_iterator( boost::filesystem::path( dir_name ) ),
		boost::filesystem::directory_iterator(), 
		[](const boost::filesystem::directory_entry& e) {
			return e.path().extension() == ".pcd";  }
	);
	cout << num_of_pcds << " detected." << endl << endl;

	if ( argc == 2 ) {
        create_align_traj();
        create_pose_traj( num_of_pcds );
		visualize( num_of_pcds );
	}

	return 0;
}

