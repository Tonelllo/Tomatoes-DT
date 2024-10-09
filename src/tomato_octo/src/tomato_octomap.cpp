#include "octomap_msgs/Octomap.h"
#include "moveit_msgs/PlanningScene.h"
#include "ros/node_handle.h"

class OctoUpdater{
    ros::NodeHandle mNh;
    ros::Publisher mPub;
    ros::Subscriber mOctoSub;

    public:

    void octomapCallback(octomap_msgs::Octomap octo){
        if(mPub.getNumSubscribers() < 1){
            ROS_WARN("No one is interested in your octomap");
            return;
        }

        geometry_msgs::Pose pose;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        moveit_msgs::PlanningScene ps;
        ps.world.octomap.header.frame_id = octo.header.frame_id;
        ps.world.octomap.octomap = octo;
        ps.world.octomap.origin = pose;
        ps.is_diff = true;

        mPub.publish(ps);
    }

    OctoUpdater(ros::NodeHandle& nh){
        mNh = nh;
        mPub = mNh.advertise<moveit_msgs::PlanningScene>("planning_scene", 5);
        mOctoSub = mNh.subscribe("/octomap_full", 5, &OctoUpdater::octomapCallback, this);
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "octomap_adder");
    ros::NodeHandle nh;
    OctoUpdater ou(nh);
    ros::spin();
}
