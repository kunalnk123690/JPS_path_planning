#include "global_planner.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "jps_path_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(nh_);

    ros::Rate lr(1000);
    while (ros::ok()){
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
