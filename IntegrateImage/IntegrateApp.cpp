
#include "IntegrateApp.h"

#include <iostream>
using namespace std;

CIntegrateApp::CIntegrateApp( )
	: cols_( 640 ), rows_( 480 )
	, volume_( cols_, rows_ )
    , volume_size_(3.0)
	, exit_( false )
	, time_ms_( 0 )
	, frame_id_( 0 )
	, traj_filename_( "" )
	, pose_filename_( "" )
	, seg_filename_( "" )
	, camera_filename_( "" )
	, ctr_filename_( "" )
	, pcd_filename_( "world.pcd" )
	, blacklist_filename_("")
	, ctr_num_( 0 )
	, ctr_resolution_( 8 )
	, ctr_interval_( 50 )
	, ctr_length_( 3.0 )
	, start_from_( -1 )
	, end_at_( 100000000 )
{
	
	depth_.resize( cols_ * rows_ );
	scaled_depth_.resize( cols_ * rows_ );

#ifdef IMAGE_VIEWER
	viewer_depth_.setWindowTitle( "Depth stream" );
	viewer_depth_.setPosition( 0, 0 );
#endif
}

CIntegrateApp::~CIntegrateApp(void)
{
}

void CIntegrateApp::Init()
{
	if ( boost::filesystem::exists( camera_filename_ ) ) {
		volume_.camera_.LoadFromFile( camera_filename_ );
	}
    volume_.unit_length_ = volume_size_ / 512;

	if ( ctr_num_ > 0 && boost::filesystem::exists( ctr_filename_ ) && boost::filesystem::exists( seg_filename_ ) ) {
		grids_.resize( ctr_num_ );
		FILE * f = fopen( ctr_filename_.c_str(), "r" );
		for ( int i = 0; i < ctr_num_; i++ ) {
			grids_[ i ].Load( f, ctr_resolution_, ctr_length_ );
		}
		fclose( f );
	} else {
		ctr_num_ = 0;
	}

	if ( boost::filesystem::exists( traj_filename_ ) ) {
		traj_.LoadFromFile( traj_filename_ );
	}

	if (boost::filesystem::exists(blacklist_filename_)) {
		blacklist_.clear();

		FILE * f = fopen(blacklist_filename_.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			int id;
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d", &id);
					blacklist_.insert(id);
				}
			}
			fclose(f);
		}
	}

	if ( boost::filesystem::exists( seg_filename_ ) ) {
		seg_traj_.LoadFromFile( seg_filename_ );

		if ( boost::filesystem::exists( pose_filename_ ) ) {
			pose_traj_.LoadFromFile( pose_filename_ );

            // compute the world transformation for each frame
			traj_.data_.clear();
			for ( int i = 0; i < ( int )pose_traj_.data_.size(); i++ ) {
				if (blacklist_.find(i) != blacklist_.end() || pose_traj_.data_[i].frame_ == -1) {
					for (int j = 0; j < ctr_interval_; j++) {
						int idx = i * ctr_interval_ + j;
						// mark the pose as invalid by the frame index -1
						traj_.data_.push_back(FramedTransformation(idx, idx, -1, Eigen::Matrix4d::Zero()));
					}
				}
				else {
					for (int j = 0; j < ctr_interval_; j++) {
						int idx = i * ctr_interval_ + j;

						if (seg_traj_.data_[idx].frame_ == -1) {
							traj_.data_.push_back(FramedTransformation(idx, idx, -1, Eigen::Matrix4d::Zero()));
						}
						else {
							traj_.data_.push_back(FramedTransformation(idx, idx, seg_traj_.data_[idx].frame_, pose_traj_.data_[i].transformation_ * seg_traj_.data_[idx].transformation_));
						}
					}
				}
			}
			PCL_WARN( "Trajectory created from pose and segment trajectories.\n" );

			// save the final per-frame camera pose to trajectory.log
			// this can be used as input for mesh segmentation application
			traj_.SaveToFile("trajectory.log");
		}
	}
}

void CIntegrateApp::onDepthFrame(Increon::DepthFrame &depth) {

	frame_id_++;

	cols_ = depth.buffer.width();
	rows_ = depth.buffer.height();
	depth_.resize(cols_ * rows_);
	scaled_depth_.resize(cols_ * rows_);
	// SH: remember to change volume size (e.g., when resolution is not standard like 320x240)
	volume_.cols_ = cols_;
	volume_.rows_ = rows_;

	// copy data to depth_
	for (int y = 0; y < rows_; ++y) {
		for (int x = 0; x < cols_; ++x) {
			depth_[y * cols_ + x] = depth.buffer(x, y) * 1000;		// our depth is in meter while the buffer requires raw
		}
	}

	try {
		this->Execute(true);

#ifdef IMAGE_VIEWER
		viewer_depth_.showShortImage(&depth_[0], cols_, rows_, 0, 3000, true);
		viewer_depth_.spinOnce(3);
#endif
	}
	catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
	catch (const std::exception& /*e*/) { cout << "Exception" << endl; }

}

void CIntegrateApp::onDepthStreamEnd() {
	std::cout << "Saving world to " << pcd_filename_ << std::endl;
	volume_.SaveWorld(pcd_filename_);
	std::cout << "Done." << std::endl;
}

void CIntegrateApp::Execute( bool has_data )
{
	if ( !has_data ) {
		return;
	}

	if ( frame_id_ >= traj_.data_.size() ) {
		exit_ = true;
		return;
	}

	if (traj_.data_[frame_id_ - 1].frame_ == -1) {
		return;
	}

	if ( frame_id_ % 100 == 0 ) {
		PCL_WARN( "Frames processed : %d / %d\n", frame_id_, traj_.data_.size() );
	}

	if ( frame_id_ < start_from_ || frame_id_ > end_at_ ) {
		if ( frame_id_ > end_at_ ) {
			PCL_WARN( "Reaching the specified end point.\n" );
			exit_ = true;
		}
		return;
	}

    // SH: also disable reproject() as it alters original depth data
	if ( ctr_num_ > 0 ) {
		Reproject();                // TODO: handle chunk in reproject later
		if ( exit_ ) {
			return;
		}
	}

    // if we have frame lost in ONI file, then looking up trajectory with frame_id_ - 1 is wrong!
	volume_.ScaleDepth( depth_, scaled_depth_ );
	volume_.Integrate( depth_, scaled_depth_, traj_.data_[ frame_id_ - 1 ].transformation_ );
    /*
    // SH: disable scale depth (set factor to 1)
    volume_.ScaleDepth1( depth_, scaled_depth_ );
    volume_.Integrate( depth_, scaled_depth_, traj_.data_[ frame_id_ - 1 ].transformation_ );
    */
}

void CIntegrateApp::Reproject()
{
	if ( frame_id_ > ctr_interval_ * ctr_num_ ) {
		exit_ = true;
		return;
	}

	depth_buffer_.resize( depth_.size() );
	for ( int i = 0; i < cols_ * rows_; i++ ) {
		depth_buffer_[ i ] = depth_[ i ];
		depth_[ i ] = 0;
	}

    // chunk ID is only correct if there are not many frame drops. 
	int chunk = ( traj_.data_[ frame_id_ - 1 ].frame_ - 1 ) / ctr_interval_;
	Eigen::Matrix4d TiT0Ai_adj = traj_.data_[ frame_id_ - 1 ].transformation_.inverse() * traj_.data_[ 0 ].transformation_ * seg_traj_.data_[ 0 ].transformation_.inverse();

	int uu, vv;
	unsigned short dd;
	double x, y, z;
	for ( int v = 0; v < rows_; v += 1 ) {
		for ( int u = 0; u < cols_; u += 1 ) {
			unsigned short d = depth_buffer_[ v * cols_ + u ];
			if ( volume_.UVD2XYZ( u, v, d, x, y, z ) ) {
				Eigen::Vector4d dummy = seg_traj_.data_[ frame_id_ - 1 ].transformation_ * Eigen::Vector4d( x, y, z, 1 );
				Coordinate coo;
				Eigen::Vector3f pos;

				if ( grids_[ chunk ].GetCoordinate( Eigen::Vector3f( dummy( 0 ), dummy( 1 ), dummy( 2 ) ), coo ) ) {		// in the box, thus has the right coo
					grids_[ chunk ].GetPosition( coo, pos );
					Eigen::Vector4d reproj_pos = TiT0Ai_adj * Eigen::Vector4d( pos( 0 ), pos( 1 ), pos( 2 ), 1.0 );

					if ( volume_.XYZ2UVD( reproj_pos( 0 ), reproj_pos( 1 ), reproj_pos( 2 ), uu, vv, dd ) ) {
						unsigned short ddd = depth_[ vv * cols_ + uu ];
						if ( ddd == 0 || ddd > dd ) {
							depth_[ vv * cols_ + uu ] = dd;
						}
					}
				}
			}
		}
	}
}