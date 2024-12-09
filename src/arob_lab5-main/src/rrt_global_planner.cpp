#include <pluginlib/class_list_macros.h>
#include "AROB_lab5/rrt_global_planner.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace rrt_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

visualization_msgs::Marker createLineMarker(std::string frame_id){
    
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;  // Change this frame_id according to your setup
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "path";
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.05;  // Line width
    line_marker.color.a = 1.0;
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.0;
    line_marker.pose.orientation.w = 1.0;
    line_marker.lifetime = ros::Duration();
    return line_marker;

}

RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false), tree_initialized_(false),
                            max_samples_(0.0), tree_(nullptr){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");
        node_marker_ = nh.advertise<visualization_msgs::Marker>("/rrt_marker", 10);

        nh.param("maxsamples", max_samples_, 0.0);
        nh.param("treshold", treshold_, 0.5);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/3.0);  //or any other distance within local costmap

        nh_global.param("resolution", resolution_, 0.05);
        

        // std::cout << "Parameters: " << max_samples_ << ", " << dist_th_ << ", " << visualize_markers_ << ", " << max_dist_ << std::endl;
        // std::cout << "Local costmap size: " << width << ", " << height << std::endl;
        // std::cout << "Global costmap resolution: " << resolution_ << std::endl;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    // std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    
    if (!tree_initialized_){
        bool computed = computeRRT(point_start, point_goal, solRRT);
        std::cout << "computed" << computed << std::endl;
        if (computed){        
            getPlan(solRRT, plan);
            // add goal
            plan.push_back(goal);
            tree_initialized_ = true;
        }else{
            ROS_WARN("No plan computed");
            return false;
        }
    } else {
        getPlan(solRRT, plan);
    }

    return true;
}

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    
    bool finished = false;
    int iter = 0;

    //Initialize random number generator
    srand(time(NULL));
        
    // Initialize the tree with the starting point in map coordinates
    TreeNode *itr_node = new TreeNode(start); 

    auto treeMarker = createLineMarker(global_frame_id_);

    while ( iter < max_samples_ ){
    
        iter++;

        //Generate a random point inside the map
        int x_rand = rand() % costmap_->getSizeInCellsX(); 
        int y_rand = rand() % costmap_->getSizeInCellsY(); 
        
        
        if (costmap_->getCost(x_rand, y_rand) != costmap_2d::FREE_SPACE){
            continue;
        }

        std::vector<int> point{(int)x_rand, (int)y_rand};
        TreeNode *new_node = new TreeNode(point);

        //Find the nearest point in the tree
        TreeNode *nearest = new_node->neast(itr_node);
        unsigned int x0 = nearest->getNode()[0];
        unsigned int y0 = nearest->getNode()[1];

        // Verify if the path is free of obstacles
        if(!obstacleFree( x0, y0, x_rand, y_rand )){
            delete new_node;
            continue;
        }
         

        // Adjust the distance to the random point to the maximum distance in the same direction
       std::vector <int> near_to_rand = {x_rand - x0, y_rand - y0};
       double dist = sqrt(near_to_rand[0] * near_to_rand[0] + near_to_rand[1] * near_to_rand[1]);

      
       if (dist > max_dist_ / resolution_){
           std::vector <int> ntr_direction = {
               static_cast<int>(static_cast<double>(near_to_rand[0]) * (max_dist_ / resolution_) / dist),
               static_cast<int>(static_cast<double>(near_to_rand[1]) * (max_dist_ / resolution_) / dist)
           };

           x_rand = x0 + ntr_direction[0];
           y_rand = y0 + ntr_direction[1];
           std::vector <int> x_new_coord = {x_rand, y_rand};
           new_node = new TreeNode(x_new_coord); // Updates the random node
       }


        // Create a new node and add it to the tree 
        nearest->appendChild(new_node);
        // Draw the new node and the path to the nearest node.
        drawMarker_(treeMarker, new_node);

        double disgoal = distance(x_rand, y_rand, goal[0], goal[1]);
        
        // Comprobar si se alcanza la meta
        if ((disgoal*resolution_ <= treshold_) && (obstacleFree(x_rand, y_rand, goal[0], goal[1]))) {
            // Reconstruir el camino desde la meta al inicio
            TreeNode *final_node = new TreeNode(goal);
            new_node->appendChild(final_node);
            drawMarker_(treeMarker, final_node);
            ROS_INFO("Goal reached");
            sol = new_node->returnSolution();  
            finished = true;
            break;
        }
        
    }
    if (!finished){
        ROS_WARN("RRT failed to find a path to the goal");
    }

    // implement RRT here!
    itr_node->~TreeNode();

    return finished;
}

void RRTPlanner::drawMarker_(visualization_msgs::Marker& line_marker, TreeNode *node) {

    line_marker.header.stamp = ros::Time::now();
    line_marker.lifetime = ros::Duration();

    // leafNode 
    // Get coordinates of the nodes
    unsigned int x_leaf = node->getNode()[0];
    unsigned int y_leaf = node->getNode()[1];
    TreeNode *parent = node->getParent();
    unsigned int x_parent = parent->getNode()[0];
    unsigned int y_parent = parent->getNode()[1];

    geometry_msgs::Point p;

    // leaf Node
    costmap_->mapToWorld(x_leaf, y_leaf, p.x, p.y);
    p.z = 0.0;
    line_marker.points.push_back(p);

    // parent node
    costmap_->mapToWorld(x_parent, y_parent, p.x, p.y);
    line_marker.points.push_back(p);

    // publish the line
    node_marker_.publish(line_marker);
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1;
        plan.push_back(pose);

    }
}


};
