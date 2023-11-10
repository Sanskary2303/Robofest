#include <gnc_functions.hpp>
//include API 
#include <geometry_msgs/Polygon.h>

std::vector<gnc_api_waypoint> goals;
geometry_msgs::Polygon polygon_points;
void polygon_callback(const geometry_msgs::Polygon::ConstPtr& msg)
{
	polygon_points = *msg;
	ROS_INFO("Polygon received wirh %lu points", polygon_points.points.size());
    // for(const auto& point : msg->points)
    // {
    //     gnc_api_waypoint wp;
    //     wp.x = point.x;
    //     wp.y = point.y;
    //     wp.z = point.z;
    //     wp.psi = 0;
    //     goals.push_back(wp);
    // }
}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	set_mode("GUIDED");

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	ros::Rate rate(2.0);

    // subscribe to the polygon topic
    ros::Subscriber polygon_subscriber = gnc_node.subscribe("/polygon", 10, polygon_callback);


	//specify some waypoints 
	// std::vector<gnc_api_waypoint> waypointList;
	// gnc_api_waypoint nextWayPoint;
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 5;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = -90;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 5;
	// nextWayPoint.y = 5;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 5;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 90;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 180;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	// ros::Rate rate(2.0);
	

	int counter = 0;
	int flag =0;
	int c = 0;
	while(ros::ok())
	{	
		if(polygon_points.points.size()!=0 && flag == 0){
			for(const auto& point : polygon_points.points)
			{
				gnc_api_waypoint wp;
				wp.x = point.x;
				wp.y = point.y;
				wp.z = point.z;
				wp.psi = -90+(90*c);
				goals.push_back(wp);
				c++;
			}
			flag = 1;
		}

		ros::spinOnce();
		rate.sleep();
		if(goals.size()!=0){
			if(check_waypoint_reached(.3) == 1)
			{	
				ROS_INFO("Waypoint %lu", goals.size());
				set_destination(0,0,5,0);
				if (counter < goals.size())	
				{	
					ROS_INFO("Going to waypoint %d", counter);
					set_destination(goals[counter].x/10, goals[counter].y/10, 5, goals[counter].psi);
					counter++;
						
				}
				else{
					//land after all waypoints are reached
					land();
				
				}	
			}
		}
		
		
	}
	return 0;
}