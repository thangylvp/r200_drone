/////////////////////////////////////////////////////////////////////////////
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.
//
//
//
/////////////////////////////////////////////////////////////////////////////
// Authors
// * Rudy Cazabon
// * Rick Blacker
//
// Dependencies
// * LibRealSense
// * OpenCV
//
/////////////////////////////////////////////////////////////////////////////
// This code sample shows how you can use LibRealSense and OpenCV to display
// both an RGB stream as well as Depth stream into two separate OpenCV
// created windows.
//
/////////////////////////////////////////////////////////////////////////////

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

using namespace std;
using namespace rs;


// Window size and frame rate
int const INPUT_WIDTH 	= 640;
int const INPUT_HEIGHT 	= 480;
int const FRAMERATE 	= 60;

// Named windows
char* const WINDOW_DEPTH = "Depth Image";
char* const WINDOW_RGB	 = "RGB Image";

cv::viz::Viz3d viewer;



context 	_rs_ctx;
device* 	_rs_camera = NULL;
intrinsics 	_depth_intrin;
intrinsics  _color_intrin;
bool 		_loop = true;

void convertToXYZ(const cv::Mat & depthMat, intrinsics & camInfo, cv::Mat & xyzMat, double scale)
{
	cv::Mat depthImgF;
	depthMat.convertTo(depthImgF, CV_32F); // convert the image data to float type 
	depthImgF /= scale;
	std::cerr << "sample " << depthImgF.at<float>(240, 320) << std::endl;
	const float qnan = std::numeric_limits<float>::quiet_NaN();

	int idx = 0;
	for (int i = 0; i < depthImgF.rows; i++)
	{
		for (int j = 0; j < depthImgF.cols; j++)
		{
			float d = depthImgF.at<float>(i, j);
			

			if (d < 0.5 || d > 1.5)
				xyzMat.at<cv::Point3f>(i, j) = cv::Point3f(qnan, qnan, qnan);
			else
				xyzMat.at<cv::Point3f>(i, j) = cv::Point3f((j - (camInfo.width / 2.f - 0.5f)) * d / camInfo.fx, (i - (camInfo.height / 2.f - 0.5f)) * d / camInfo.fy, d);
			idx++;
		}
	}
}

// Initialize the application state. Upon success will return the static app_state vars address
bool initialize_streaming( )
{
	bool success = false;
	if( _rs_ctx.get_device_count( ) > 0 )
	{
		_rs_camera = _rs_ctx.get_device( 0 );

		_rs_camera->enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
		_rs_camera->enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );

		_rs_camera->start( );

		success = true;
	}
	viewer = cv::viz::Viz3d( "Point Cloud" );
	return success;
}




/////////////////////////////////////////////////////////////////////////////
// Create the depth and RGB windows, set their mouse callbacks.
// Required if we want to create a window and have the ability to use it in
// different functions
/////////////////////////////////////////////////////////////////////////////
void setup_windows( )
{
	cv::namedWindow( WINDOW_DEPTH, 0 );
	cv::namedWindow( WINDOW_RGB, 0 );

	
}



/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using OpenCV.
/////////////////////////////////////////////////////////////////////////////
bool display_next_frame( )
{
	// Get current frames intrinsic data.
	// _depth_intrin 	= _rs_camera->get_stream_intrinsics( rs::stream::depth );
	_depth_intrin 	= _rs_camera->get_stream_intrinsics(rs::stream::depth_aligned_to_color);
	_color_intrin 	= _rs_camera->get_stream_intrinsics( rs::stream::color );

	// Create depth image
	cv::Mat xyzMat = cv::Mat(INPUT_HEIGHT, INPUT_WIDTH, CV_32FC3);


	cv::Mat depth16( _depth_intrin.height,
					 _depth_intrin.width,
					 CV_16U,
					 (uchar *)_rs_camera->get_frame_data( rs::stream::depth_aligned_to_color ) );

	std::cerr << "depth : " << _depth_intrin.height << " " << _depth_intrin.width << " " << _depth_intrin.fx << " " << _depth_intrin.fy << std::endl;
	
	convertToXYZ(depth16, _depth_intrin, xyzMat, 1000.f);
	// Create color image
	cv::Mat rgb( _color_intrin.height,
				 _color_intrin.width,
				 CV_8UC3,
				 (uchar *)_rs_camera->get_frame_data( rs::stream::color ) );

	std::cerr << "color : " << _color_intrin.height << " " << _color_intrin.width << " " << _color_intrin.fx << " " << _color_intrin.fy << std::endl;
	cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	// < 800
	// cv::Mat depth8u = depth16;
	// depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );
	
	cv::viz::WCloud cloud( xyzMat, rgb );
	viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
	viewer.showWidget( "Cloud", cloud );
    viewer.spinOnce();
	
	depth16 *= 5;
	imshow( WINDOW_DEPTH, depth16 );
	
	

	
	imshow( WINDOW_RGB, rgb );
	
	if (cv::waitKey(5) == 'q') return false;

	return true;
}



/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main( ) try
{
	rs::log_to_console( rs::log_severity::warn );

	if( !initialize_streaming( ) )
	{
		std::cout << "Unable to locate a camera" << std::endl;
		rs::log_to_console( rs::log_severity::fatal );
		return EXIT_FAILURE;
	}

	setup_windows( );

	// Loop until someone left clicks on either of the images in either window.
	while( _loop )
	{
		if( _rs_camera->is_streaming( ) )
			_rs_camera->wait_for_frames( );

		if (!display_next_frame( )) break;
	}

	_rs_camera->stop( );
	cv::destroyAllWindows( );

	return EXIT_SUCCESS;
}
catch( const rs::error & e )
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch( const std::exception & e )
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
