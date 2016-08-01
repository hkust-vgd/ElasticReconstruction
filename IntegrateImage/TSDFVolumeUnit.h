#pragma once
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>

#include <pcl/console/print.h>
#include <pcl/common/time.h>

struct FramedTransformation {
	int id1_;
	int id2_;
	int frame_;
	Eigen::Matrix4d transformation_;
	FramedTransformation( int id1, int id2, int f, Eigen::Matrix4d t )
		: id1_( id1 ), id2_( id2 ), frame_( f ), transformation_( t ) 
	{}
};

struct RGBDTrajectory {
	std::vector< FramedTransformation > data_;
	int index_;

	void LoadFromFile( std::string filename ) {
		data_.clear();
		index_ = 0;
		int id1, id2, frame;
		Eigen::Matrix4d trans;
		FILE * f = fopen( filename.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[1024];
			while ( fgets( buffer, 1024, f ) != NULL ) {
				if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
					sscanf( buffer, "%d %d %d", &id1, &id2, &frame);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(0,0), &trans(0,1), &trans(0,2), &trans(0,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(1,0), &trans(1,1), &trans(1,2), &trans(1,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(2,0), &trans(2,1), &trans(2,2), &trans(2,3) );
					fgets( buffer, 1024, f );
					sscanf( buffer, "%lf %lf %lf %lf", &trans(3,0), &trans(3,1), &trans(3,2), &trans(3,3) );
					data_.push_back( FramedTransformation( id1, id2, frame, trans ) );
				}
			}
			fclose( f );
		}
	}
	void SaveToFile( std::string filename ) {
		FILE * f = fopen( filename.c_str(), "w" );
		for ( int i = 0; i < ( int )data_.size(); i++ ) {
			Eigen::Matrix4d & trans = data_[ i ].transformation_;
			fprintf( f, "%d\t%d\t%d\n", data_[ i ].id1_, data_[ i ].id2_, data_[ i ].frame_ );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(0,0), trans(0,1), trans(0,2), trans(0,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(1,0), trans(1,1), trans(1,2), trans(1,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(2,0), trans(2,1), trans(2,2), trans(2,3) );
			fprintf( f, "%.8f %.8f %.8f %.8f\n", trans(3,0), trans(3,1), trans(3,2), trans(3,3) );
		}
		fclose( f );
	}
};

struct CameraParam {
public:
	float fx_, fy_, cx_, cy_, ICP_trunc_, integration_trunc_;

	CameraParam() : fx_( 525.0f ), fy_( 525.0f ), cx_( 319.5f ), cy_( 239.5f ), ICP_trunc_( 2.5f ), integration_trunc_( 2.5f )  {
	}

	inline void parseIni(std::string fn, std::map<std::string, std::string> &param) {
		std::ifstream ifile;
		ifile.open(fn.c_str(), std::ios::in);

		char line[256];
		while (ifile.getline(line, 256)) {
			if (strlen(line) == 0 || line[0] == '\0' || line[0] == '#')
				continue;

			std::string s1, s2;
			s1 = strtok(line, " \t\r\n");
			s2 = strtok(NULL, " \t\r\n");

			param.insert(std::make_pair(s1, s2));
		}

		ifile.close();
	}

	void LoadFromFile( std::string filename ) {
		// extract from INI file
		std::map<std::string, std::string> params;
		parseIni(filename, params);
		std::cout << "Loading config: " << filename << std::endl;

		if (params.find("fx") != params.end())		fx_ = atof(params["fx"].c_str());
		if (params.find("fy") != params.end())		fy_ = atof(params["fy"].c_str());
		if (params.find("cx") != params.end())		cx_ = atof(params["cx"].c_str());
		if (params.find("cx") != params.end())		cy_ = atof(params["cy"].c_str());
		if (params.find("icp_truncation") != params.end())				ICP_trunc_ = atof(params["icp_truncation"].c_str());
		if (params.find("integration_truncation") != params.end())		integration_trunc_ = atof(params["integration_truncation"].c_str());

		printf("Camera model set to (fx, fy, cx, cy, icp_trunc, int_trunc):\n\t%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
			fx_, fy_, cx_, cy_, ICP_trunc_, integration_trunc_);

		/*
		FILE * f = fopen( filename.c_str(), "r" );
		if ( f != NULL ) {
			char buffer[1024];
			while ( fgets( buffer, 1024, f ) != NULL ) {
				if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
					sscanf( buffer, "%f", &fx_);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%f", &fy_);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%f", &cx_);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%f", &cy_);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%f", &ICP_trunc_);
					fgets( buffer, 1024, f );
					sscanf( buffer, "%f", &integration_trunc_);
				}
			}
			fclose ( f );
			PCL_WARN( "Camera model set to (fx, fy, cx, cy, icp_trunc, int_trunc):\n\t%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
				fx_, fy_, cx_, cy_, ICP_trunc_, integration_trunc_ );
		}
		*/ 


	}
};

class TSDFVolumeUnit
{
public:
	TSDFVolumeUnit( int resolution, int xi, int yi, int zi );
	~TSDFVolumeUnit(void);

public:
	typedef boost::shared_ptr< TSDFVolumeUnit > Ptr;

public:
	float * sdf_;
	float * weight_;

	const int resolution_;
	int xi_, yi_, zi_;
};

