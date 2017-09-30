/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   
   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM
   
   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "ocam_functions.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/camera_subscriber.h>
#include <camera_info_manager/camera_info_manager.h>
#include <iostream>
namespace enc = sensor_msgs::image_encodings;
class ImageConverter{

protected:
     ros::NodeHandle nh;
     // Node handle in the private namespace
     ros::NodeHandle priv_nh_;
     image_transport::ImageTransport it;
     //image_transport::CameraSubscriber sub;
     ros::Subscriber sub;
     image_transport::Publisher pub;
     //ros::Publisher pub_info;
     std::string fpath;
   //sensor_msgs::CameraInfo info_camera; //

public:
    ImageConverter(ros::NodeHandle &nh) : nh(nh),it(nh), priv_nh_("~")
    {
      priv_nh_.param<std::string>("f", fpath, "");
      sub = nh.subscribe("usb_cam/image_raw", 1, &ImageConverter::input,this);
      pub = it.advertise("ocam/image_raw", 1);
      //pub_info= nh.advertise<sensor_msgs::CameraInfo>("ocam/camera_info", 1);

}


~ImageConverter()
{
    cvDestroyAllWindows();

}



void input(const sensor_msgs::ImageConstPtr& msg)//, const sensor_msgs::CameraInfoConstPtr& info_msg)

{ 

//sensor_msgs::CameraInfo info = *info_msg;
    ros::WallTime start_time = ros::WallTime::now();
    cv_bridge::CvImageConstPtr src11;



    try{
    src11 = cv_bridge::toCvShare(msg, enc::MONO8);

   //src1 = bridge.imgMsgToCv(msg,"mono8");
            //cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
}
    catch(cv_bridge::Exception& e)
    {ROS_ERROR("error %s",e.what());

}
 if (src11){

	float div=2;
        /* std::istringstream video_sourceCmd(argv[1]);
         int video_source;
         if(!(video_sourceCmd >> video_source)) return 1;
         cvCapture* cap=cvCaptureFromCAM(video_source);
         if(!cap.isOpened()) return 1;
         //cv::Mat src1;
         sensor_msgs::ImagePtr msg;
         ros::Rate loop_rate(5);
     */
         /* --------------------------------------------------------------------*/
         /* Read the parameters of the omnidirectional camera from the TXT file */
         /* --------------------------------------------------------------------*/
         struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras
         get_ocam_model(&o, "./calib_results.txt");
         /* --------------------------------------------------------------------*/
         /* Print ocam_model parameters                                         */
         /* --------------------------------------------------------------------*/
         int i;
         //printf("pol =\n");    for (i=0; i<o.length_pol; i++){    printf("\t%e\n",o.pol[i]); };    printf("\n");
         //printf("invpol =\n"); for (i=0; i<o.length_invpol; i++){ printf("\t%e\n",o.invpol[i]); }; printf("\n");
         //printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o.xc,o.yc,o.width,o.height);

         /* --------------------------------------------------------------------*/
         /* WORLD2CAM projects 3D point into the image                          */
         /* NOTE!!! The coordinates are expressed according the C convention,   */
         /* that is, from the origin (0,0) instead than from 1 (MATLAB).        */
         /* --------------------------------------------------------------------*/
         double point3D[3] = { 100 , 200 , -300 };       // a sample 3D point
         double point2D[2];                              // the image point in pixel coordinates
         world2cam(point2D, point3D, &o); // The behaviour of this function is the same as in MATLAB

         /* --------------------------------------------------------------------*/
         /* Display re-projected coordinates                                    */
         /* --------------------------------------------------------------------*/
       //  printf("\nworld2cam: pixel coordinates reprojected onto the image\n");
       //  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);

         /* --------------------------------------------------------------------*/
         /* CAM2WORLD back-projects pixel points on to the unit sphere          */
         /* The behaviour of this function is the same as in MATLAB             */
         /* --------------------------------------------------------------------*/

         cam2world(point3D, point2D, &o);
         /* --------------------------------------------------------------------*/
         /* Display back-projected normalized coordinates (on the unit sphere)  */
         /* --------------------------------------------------------------------*/
         //printf("\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
         //printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);

         IplImage cvImageRef, *src1,*src2;
         CvSize size;
         cvImageRef = IplImage(src11->image);
         src2 = &cvImageRef;
	 src1=cvCreateImage(cvSize(1280/div,720/div),src2->depth,src2->nChannels);
         cvResize(src2,src1,CV_INTER_LINEAR);
         size = cvGetSize(src1);
         IplImage *dst_persp   = cvCreateImage( size, 8, 1 );   // undistorted perspective and panoramic image
         CvMat* mapx_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);
         CvMat* mapy_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);

             /* --------------------------------------------------------------------  */
             /* Create Look-Up-Table for perspective undistortion                     */
             /* SF is kind of distance from the undistorted image to the camera       */
             /* (it is not meters, it is justa zoom fator)                            */
             /* Try to change SF to see how it affects the result                     */
             /* The undistortion is done on a  plane perpendicular to the camera axis */
             /* --------------------------------------------------------------------  */
             float sf = 3;
             create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o, sf );

             /* --------------------------------------------------------------------*/
             /* Undistort using specified interpolation method                      */
             /* Other possible values are (see OpenCV doc):                         */
             /* CV_INTER_NN - nearest-neighbor interpolation,                       */
             /* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
             /* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
             /* CV_INTER_CUBIC - bicubic interpolation.                             */
             /* --------------------------------------------------------------------*/


                   //cvNamedWindow( "Undistorted Perspective Image", 1 );
                  // cvShowImage( "Un", src1);

             cvRemap( src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );

             cvFlip(dst_persp,dst_persp,1);
             cv::Mat mat_image=cv::cvarrToMat(dst_persp);

             //mat_image = mat_image(cv::Rect(0,100,1280,400)); //Recorte de la imagen
             //mat_image = mat_image(cv::Rect(120,0,300,287)); //Recorte de la imagen

             sensor_msgs::ImagePtr msg = cv_bridge::CvImage(src11->header, "mono8", mat_image).toImageMsg();

            // cv_bridge::CvImage out_msg;
             //out_msg.header   = src11->header; // Same timestamp and tf frame as input image
             //out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
            // out_msg.image    = mat_image;// Your cv::Mat
//		info.header.frame_id="ocam";

//		info.height =src1->height;
//                info.width =src1->width;
//		info.K[0]=info.K[0]/div;
//		info.K[2]=info.K[2]/div;
//		info.K[4]=info.K[4]/div;
//		info.K[5]=info.K[5]/div;
//		info.P[0]=info.P[0]/div;
//		info.P[2]=info.P[2]/div;
//		info.P[5]=info.P[5]/div;
//		info.P[6]=info.P[6]/div;
//		info.roi.do_rectify= true;
//    info.header.stamp = ros::Time::now();
		//msg->header.stamp=info.header.stamp;
		msg->header.frame_id="ocam";
             try
                {
                   //pub.publish(bridge.cvToImgMsg(dst_persp, "bgr8"));
                    pub.publish(msg);
		//	pub_info.publish(info_camera);
		//	info.header.stamp = ros::Time::now();
			//pub_info.publish(info);
 	                }
                 catch(cv_bridge::Exception& e)
                  {
                    ROS_ERROR("error");
                }
            
             //cvShowImage( "Undistorted Perspective Image", dst_persp);
             //cvSaveImage("frame_cap5.jpg",dst_persp);
             //cvWaitKey(1);
             cvReleaseImage(&dst_persp);
             cvReleaseMat(&mapx_persp);
             cvReleaseMat(&mapy_persp);
 }








       }





 };
int main(int argc, char** argv)
 {
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;
  ImageConverter ic(nh);
  ros::Rate rate(1); //100 Hz, every .01 second
  while(ros::ok())
  {
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}
