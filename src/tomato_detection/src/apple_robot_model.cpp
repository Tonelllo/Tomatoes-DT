#include "agri_control/apple_robot_model.h"

#include <vector>

namespace rml {

AppleRobotModel::AppleRobotModel(std::shared_ptr<ros::NodeHandle> nh, const std::string id)
    : ArmModel(id)
{
    int numJoints = 8; // arm 7 DOFs + torso 1 DOF
    std::vector<double> minJoint(numJoints, 0.0);
    std::vector<double> maxJoint(numJoints, 0.0);
    std::vector<double> roll(numJoints, 0.0);
    std::vector<double> pitch(numJoints, 0.0);
    std::vector<double> yaw(numJoints, 0.0);
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
        std::cerr << "minJoint.at(jointIdx) = " << minJoint.at(jointIdx) << " with jointIdx = " << jointIdx << std::endl;
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/max_angle", maxJoint.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/distance_vector/x", tx.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/distance_vector/y", ty.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/distance_vector/z", tz.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/roll", roll.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/pitch", pitch.at(jointIdx));
        nh_->getParam(jointParametersString + "/joint" + std::to_string(jointIdx) + "/yaw", yaw.at(jointIdx));
    }
    std::vector<Eigen::TransformationMatrix> biTri(numJoints);
    
    Eigen::RowVectorXd jMin(numJoints), jMax(numJoints);
    jMin << minJoint.at(0), minJoint.at(1), minJoint.at(2), minJoint.at(3), minJoint.at(4), minJoint.at(5), minJoint.at(6), minJoint.at(7);
    jMax << maxJoint.at(0), maxJoint.at(1), maxJoint.at(2), maxJoint.at(3), maxJoint.at(4), maxJoint.at(5), maxJoint.at(6), maxJoint.at(7);

    jMin = jMin * M_PI / 180.0;
    jMax = jMax * M_PI / 180.0;

    for (auto i = 0; i < numJoints; i++) {
        biTri.at(i).RotationMatrix(rml::EulerRPY(roll.at(i),pitch.at(i),yaw.at(i)).ToRotationMatrix());
        biTri.at(i).TranslationVector(Eigen::Vector3d(tx.at(i),ty.at(i),tz.at(i)));
        std::cerr << "biTri.at(" << i << ") = " << std::endl << biTri.at(i) << std::endl;
    }

    for (unsigned int i = 0; i < numJoints; ++i) {
        AddJointLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), biTri.at(i), jMin(i), jMax(i));
    }

    Eigen::TransformationMatrix torso_T_armj2 = biTri.at(0) * biTri.at(1);
    Eigen::TransformationMatrix torso_T_armj7 = biTri.at(0) * biTri.at(1) * biTri.at(2) * biTri.at(3) * biTri.at(4) * biTri.at(5) * biTri.at(6) * biTri.at(7);
    std::cerr << "0_d_1 = " << biTri.at(0).TranslationVector().transpose() << std::endl;
    std::cerr << "1_d_2 = " << biTri.at(1).TranslationVector().transpose() << std::endl;
    std::cerr << "0_d_2 = " << torso_T_armj2.TranslationVector().transpose() << std::endl;
    std::cerr << "0_d_7 = " << torso_T_armj7.TranslationVector().transpose() << std::endl;

    std::cerr << "0_rpy_1 = " << biTri.at(0).RotationMatrix().ToEulerRPY() << std::endl;
    std::cerr << "1_rpy_2 = " << biTri.at(1).RotationMatrix().ToEulerRPY() << std::endl;
    std::cerr << "0_rpy_2 = " << torso_T_armj2.RotationMatrix().ToEulerRPY() << std::endl;
    std::cerr << "0_rpy_7 = " << torso_T_armj7.RotationMatrix().ToEulerRPY() << std::endl;
}

AppleRobotModel::~AppleRobotModel()
{
    // TODO Auto-generated destructor stub
}
}
