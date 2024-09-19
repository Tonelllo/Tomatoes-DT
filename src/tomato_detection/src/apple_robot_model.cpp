#include "agri_control/apple_robot_model.h"

#include <vector>

namespace rml {

AppleRobotModel::AppleRobotModel(std::shared_ptr<ros::NodeHandle> nh, const std::string id)
    : ArmModel(id)
{
    int numJoints = 8; // arm 7 DOFs + torso 1 DOF
    std::vector<double> minJoint(numJoints, 0.0);
    std::vector<double> maxJoint(numJoints, 0.0);
    std::vector<double> tx(numJoints, 0.0);
    std::vector<double> ty(numJoints, 0.0);
    std::vector<double> tz(numJoints, 0.0);
    jointOffsetsSim.resize(numJoints, 0.0);
    jointDirectionSwapSim.resize(numJoints, 1.0);
    nh_ = nh;
    nh_->getParam("/numJoints", numJoints);
    std::string jointParametersString;
    if (id.find("Left") != std::string::npos) {
        jointParametersString = "/joint_parameters_left";
    }
    else {
        jointParametersString = "/joint_parameters_right";
    }

    for (auto jointIdx = 0; jointIdx < numJoints; jointIdx++) {
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/min_angle", minJoint.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/max_angle", maxJoint.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/distance_vector/x", tx.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/distance_vector/y", ty.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/distance_vector/z", tz.at(jointIdx));
    }
    std::vector<Eigen::TransformationMatrix> biTri(numJoints);
    
    Eigen::RowVectorXd jMin(numJoints), jMax(numJoints);
    jMin << minJoint.at(0), minJoint.at(1), minJoint.at(2), minJoint.at(3), minJoint.at(4), minJoint.at(5), minJoint.at(6), minJoint.at(7);
    jMax << maxJoint.at(0), maxJoint.at(1), maxJoint.at(2), maxJoint.at(3), maxJoint.at(4), maxJoint.at(5), maxJoint.at(6), maxJoint.at(7);

    jMin = jMin * M_PI / 180.0;
    jMax = jMax * M_PI / 180.0;

    biTri.at(0)(0,0) = 1; biTri.at(0)(0,1) = 0;  biTri.at(0)(0,2) = 0; biTri.at(0)(0,3) = tx.at(0);
    biTri.at(0)(1,0) = 0; biTri.at(0)(1,1) = 1;  biTri.at(0)(1,2) = 0; biTri.at(0)(1,3) = ty.at(0);
    biTri.at(0)(2,0) = 0; biTri.at(0)(2,1) = 0;  biTri.at(0)(2,2) = 1; biTri.at(0)(2,3) = tz.at(0);
    biTri.at(0)(3,0) = 0; biTri.at(0)(3,1) = 0;  biTri.at(0)(3,2) = 0; biTri.at(0)(3,3) = 1;

    biTri.at(1)(0,0) = 0; biTri.at(1)(0,1) = 0;  biTri.at(1)(0,2) = 1;  biTri.at(1)(0,3) = tx.at(1);
    biTri.at(1)(1,0) = 0; biTri.at(1)(1,1) = -1;  biTri.at(1)(1,2) = 0; biTri.at(1)(1,3) = ty.at(1);
    biTri.at(1)(2,0) = 1; biTri.at(1)(2,1) = 0;  biTri.at(1)(2,2) = 0;  biTri.at(1)(2,3) = tz.at(1); 
    biTri.at(1)(3,0) = 0; biTri.at(1)(3,1) = 0;  biTri.at(1)(3,2) = 0;  biTri.at(1)(3,3) = 1;

    biTri.at(2)(0,0) = 1; biTri.at(2)(0,1) = 0;  biTri.at(2)(0,2) = 0; biTri.at(2)(0,3) = tx.at(2);
    biTri.at(2)(1,0) = 0; biTri.at(2)(1,1) = -1;  biTri.at(2)(1,2) = 0; biTri.at(2)(1,3) = ty.at(2);
    biTri.at(2)(2,0) = 0; biTri.at(2)(2,1) = 0;  biTri.at(2)(2,2) = -1; biTri.at(2)(2,3) = tz.at(2);
    biTri.at(2)(3,0) = 0; biTri.at(2)(3,1) = 0;  biTri.at(2)(3,2) = 0; biTri.at(2)(3,3) = 1;

    biTri.at(3)(0,0) = -1; biTri.at(3)(0,1) = 0;  biTri.at(3)(0,2) = 0; biTri.at(3)(0,3) = tx.at(3);
    biTri.at(3)(1,0) = 0; biTri.at(3)(1,1) = 1;  biTri.at(3)(1,2) = 0; biTri.at(3)(1,3) = ty.at(3);
    biTri.at(3)(2,0) = 0; biTri.at(3)(2,1) = 0;  biTri.at(3)(2,2) = -1; biTri.at(3)(2,3) = tz.at(3); 
    biTri.at(3)(3,0) = 0; biTri.at(3)(3,1) = 0;  biTri.at(3)(3,2) = 0; biTri.at(3)(3,3) = 1;

    biTri.at(4)(0,0) = 0; biTri.at(4)(0,1) = 0;  biTri.at(4)(0,2) = -1; biTri.at(4)(0,3) = tx.at(4);
    biTri.at(4)(1,0) = 0; biTri.at(4)(1,1) = 1;  biTri.at(4)(1,2) = 0; biTri.at(4)(1,3) = ty.at(4);
    biTri.at(4)(2,0) = 1; biTri.at(4)(2,1) = 0;  biTri.at(4)(2,2) = 0; biTri.at(4)(2,3) = tz.at(4);
    biTri.at(4)(3,0) = 0; biTri.at(4)(3,1) = 0;  biTri.at(4)(3,2) = 0; biTri.at(4)(3,3) = 1;

    biTri.at(5)(0,0) = 0; biTri.at(5)(0,1) = 0;  biTri.at(5)(0,2) = 1; biTri.at(5)(0,3) = tx.at(5);
    biTri.at(5)(1,0) = 0; biTri.at(5)(1,1) = 1;  biTri.at(5)(1,2) = 0; biTri.at(5)(1,3) = ty.at(5);
    biTri.at(5)(2,0) = -1; biTri.at(5)(2,1) = 0;  biTri.at(5)(2,2) = 0; biTri.at(5)(2,3) = tz.at(5);
    biTri.at(5)(3,0) = 0; biTri.at(5)(3,1) = 0;  biTri.at(5)(3,2) = 0; biTri.at(5)(3,3) = 1;

    biTri.at(6)(0,0) = 0; biTri.at(6)(0,1) = 0;  biTri.at(6)(0,2) = 1; biTri.at(6)(0,3) = tx.at(6);
    biTri.at(6)(1,0) = 0; biTri.at(6)(1,1) = 1;  biTri.at(6)(1,2) = 0; biTri.at(6)(1,3) = ty.at(6);
    biTri.at(6)(2,0) = -1; biTri.at(6)(2,1) = 0;  biTri.at(6)(2,2) = 0; biTri.at(6)(2,3) = tz.at(6);
    biTri.at(6)(3,0) = 0; biTri.at(6)(3,1) = 0;  biTri.at(6)(3,2) = 0; biTri.at(6)(3,3) = 1;

    biTri.at(7)(0,0) = 0; biTri.at(7)(0,1) = 0;  biTri.at(7)(0,2) = 1; biTri.at(7)(0,3) = tx.at(7);
    biTri.at(7)(1,0) = 0; biTri.at(7)(1,1) = 1;  biTri.at(7)(1,2) = 0; biTri.at(7)(1,3) = ty.at(7);
    biTri.at(7)(2,0) = -1; biTri.at(7)(2,1) = 0;  biTri.at(7)(2,2) = 0; biTri.at(7)(2,3) = tz.at(7);
    biTri.at(7)(3,0) = 0; biTri.at(7)(3,1) = 0;  biTri.at(7)(3,2) = 0; biTri.at(7)(3,3) = 1;


    for (unsigned int i = 0; i < numJoints; ++i) {
        //std::cout << "Adding link " << i << std::endl;
        AddJointLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), biTri.at(i), jMin(i), jMax(i));
    }

  /*  jointOffsetsSim[1] = M_PI_2;
    jointOffsetsSim[3] = -M_PI_2;
    jointDirectionSwapSim[1] = -1;*/

    for (auto mat : biTri) {
        std::cout << mat << std::endl;
    }
}

AppleRobotModel::~AppleRobotModel()
{
    // TODO Auto-generated destructor stub
}
}
