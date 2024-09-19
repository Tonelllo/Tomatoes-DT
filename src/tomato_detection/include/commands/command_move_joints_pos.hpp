#ifndef UNDROID_CTRL_COMMANDMOVEJOINTSPOS_HPP
#define UNDROID_CTRL_COMMANDMOVEJOINTSPOS_HPP

#include "commands/generic_command.hpp"
#include "states/state_move_joints_pos.hpp"


namespace commands {

    class MoveJointsPos : public GenericCommand {

    private:
        std::shared_ptr<states::MoveJointsPos> stateMoveJointsPos_;
        std::vector<double> jointsPos_;
        std::string moveType_;
        int jointIndex_;
        bool singleJointMove;

    public:
        MoveJointsPos();
        virtual ~MoveJointsPos() override;
        virtual fsm::retval Execute() override;

        void SetState(std::shared_ptr<states::GenericState> state) override;
        void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;

        void SetJointsGoal(std::vector<double> &joints_pos, std::string &move_type);
        void SetSingleJointGoal(std::vector<double> &joints_pos, int joint_index, std::string &move_type);
        
        bool enableObstacleAvoidance;

    };
}

#endif // UNDROID_CTRL_COMMANDMOVEJOINTSPOS_HPP
