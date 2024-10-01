#include <mutex>
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "moveit_msgs/PlanningScene.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "sensor_msgs/image_encodings.h"

static sensor_msgs::CameraInfo gCameraInfo;

class SincPublisher{
    ros::NodeHandle mNh_;
    ros::Publisher mImageDepthPub_;
    ros::Publisher mImageRgbPub_;
    ros::Publisher mImageParamsPub_;
    ros::Subscriber mPointSub_;
    sensor_msgs::CameraInfo mCi_;

    public:

    void pc2Callback(sensor_msgs::PointCloud2 pc2){
        sensor_msgs::Image depthImage;
        sensor_msgs::Image rgbImage;
        depthImage.width = pc2.width;
        depthImage.height = pc2.height;
        depthImage.encoding = "32FC1";
        depthImage.step = depthImage.width * sizeof(float);
        depthImage.data.resize(depthImage.step * depthImage.height);

        for (int row = 0; row < pc2.height; row++)
        {
            for (int col = 0; col < pc2.width; col++)
            {
                size_t pc2Index = pc2.row_step * row + pc2.point_step * col + pc2.fields[2].offset;
                size_t imgIndex = row * depthImage.step + col * sizeof(float);
                memcpy(&depthImage.data[imgIndex], &pc2.data[pc2Index], sizeof(float));
            }
        }

        cv::Mat testImg;
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
        cvPtr->image.copyTo(testImg);
        cv::imshow("gatto", testImg);
        cv::waitKey(100);

        pcl::toROSMsg(pc2, rgbImage);

        ros::Time syncTimestamp = ros::Time::now();

        depthImage.header.frame_id = pc2.header.frame_id;
        rgbImage.header.frame_id = pc2.header.frame_id;
        mCi_.header.frame_id = pc2.header.frame_id;

        depthImage.header.stamp = syncTimestamp;
        rgbImage.header.stamp = syncTimestamp;
        mCi_.header.stamp = syncTimestamp;

        mImageDepthPub_.publish(depthImage);
        mImageRgbPub_.publish(rgbImage);
        mImageParamsPub_.publish(mCi_);
    }

    SincPublisher(ros::NodeHandle& nh, sensor_msgs::CameraInfo& ci){
        mNh_ = nh;
        mCi_ = ci;
        mImageDepthPub_ = mNh_.advertise<sensor_msgs::Image>("/tomato_sync/image_depth", 5);
        mImageRgbPub_ = mNh_.advertise<sensor_msgs::Image>("/tomato_sync/image_rgb", 5);
        mImageParamsPub_ = mNh_.advertise<sensor_msgs::CameraInfo>("/tomato_sync/image_params", 5);
        mPointSub_ = mNh_.subscribe("/throttle_filtering_points/filtered_points", 3, &SincPublisher::pc2Callback, this);
    }
};


static void getCamerainfo(sensor_msgs::CameraInfo ci){
    gCameraInfo = ci;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sync_publisher");
    ros::NodeHandle nh;

    ros::Subscriber cameraInfo = nh.subscribe("/xtion/depth_registered/camera_info", 1, getCamerainfo);
    ros::spinOnce();
    cameraInfo.shutdown();

    SincPublisher ou(nh, gCameraInfo);
    ros::spin();
}
