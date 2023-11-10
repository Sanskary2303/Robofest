#include <gnc_functions.hpp>
//include API 
#include <geometry_msgs/Polygon.h>
#include <cmath>

std::vector<gnc_api_waypoint> goals;
gnc_api_waypoint A;
std::vector<gnc_api_waypoint> B(3);
geometry_msgs::Polygon polygon_points;
sensor_msgs::NavSatFix global_position_1;
sensor_msgs::NavSatFix global_position;

struct Coordinate {
    double latitude;
    double longitude;
};

const double EARTH_RADIUS = 6371000; // Radius of the Earth in meters

double toRadians(double degree) {
    return degree * (M_PI / 180.0);
}

void polygon_callback(const geometry_msgs::Polygon::ConstPtr& msg)
{
	// polygon_points = *msg;
	// polygon_points.points
	// ROS_INFO("Polygon received wirh %lu points", polygon_points.points.size());
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

void global_position_callback_1(const sensor_msgs::NavSatFix::ConstPtr& msg){
	global_position_1 = *msg;
}

void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
	global_position = *msg;
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
    // ros::Subscriber polygon_subscriber = gnc_node.subscribe("/polygon", 10, polygon_callback);

	int random_state;
	gnc_node.getParam("random_state", random_state);

	ros::Subscriber global_position_sub_1 = gnc_node.subscribe("/drone1/mavros/global_position/global", 10, global_position_callback_1);
	ros::Subscriber global_position_sub = gnc_node.subscribe("/drone"+ std::to_string(random_state + 1) +"/mavros/global_position/global", 10, global_position_callback);

	geometry_msgs::Point32 v1;
	v1.x = 0;
	v1.y = 0;
	v1.z = 0;
	
	polygon_points.points.push_back(v1);
	geometry_msgs::Point32 v2;
	v2.x = 5;
	v2.y = 5;
	v2.z = 0;
	
	polygon_points.points.push_back(v2);
	geometry_msgs::Point32 v3 ;
	v3.x = 10;
	v3.y = 0;
	v3.z = 0;
	polygon_points.points.push_back(v3);

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
				wp.psi = 0; 	//-90+(90*c);
				goals.push_back(wp);
				c++;
			}
		for(int i = 0; i<3; i++){
			ROS_INFO("Initial%d : %f %f",random_state+1, goals[i].x, goals[i].y);
		}

		// // for(int i = 0; i<3; i++){
		// 	gnc_api_waypoint temp;
			// A.x = (global_position_1.longitude - global_position.longitude)* EARTH_RADIUS * std::cos(toRadians((global_position_1.latitude + global_position.latitude) / 2));
			// A.y = (global_position_1.latitude - global_position.latitude)* EARTH_RADIUS;
			if (random_state == 1){
			// A.x = (-35.3632621 + 35.3632622) * EARTH_RADIUS * std::cos(toRadians((149.1652374 + 149.1652925) / 2));
			// A.y = (149.1652374 - 149.1652925) * EARTH_RADIUS;
			A.x = 0;
			A.y = 5;
			}
			else if (random_state == 2){
			// A.x = (-35.3632621 + 35.3632172) * EARTH_RADIUS * std::cos(toRadians((149.1652374 + 149.1652924) / 2));	
			// A.y = (149.1652374 - 149.1652924) * EARTH_RADIUS;
			A.x = -5;
			A.y = 5;
			}
			else if (random_state == 3){
			}
			else if (random_state == 0){
				A.x = 0;
				A.y = 0;
			}
			A.z = 0;
			A.psi = 0;
		// 	A.push_back(temp);
		// }

		

		for(int i = 0; i<3; i++){
			
			goals[i].x = goals[i].x + A.x;
			goals[i].y = goals[i].y + A.y;
		}
//request takeoff
	takeoff(3);
		for(int i = 0; i<3; i++){
			B[i].x = goals[(i + random_state)%3].x;
			B[i].y = goals[(i + random_state)%3].y;
			B[i].z = 0;
			B[i].psi = 0;
		}
		flag = 1;

		}
		ROS_INFO("A%d : %f %f",random_state+1, A.x, A.y);
		for(int i = 0; i<3; i++){
			ROS_INFO("Final%d : %f %f",random_state+1, goals[i].x, goals[i].y);
		}	
		if(goals.size()!=0){
			if(check_waypoint_reached(.3) == 1)
			{	
				ROS_INFO("DRONE1 : %f %f", global_position_1.latitude, global_position_1.longitude);
				ROS_INFO("DRONE%d : %f %f", random_state+1, global_position.latitude, global_position.longitude);
				ROS_INFO("Waypoint : %lu", goals.size());
				// set_destination(0,0,5,0);
				if (counter < goals.size())	
				{	
					ROS_INFO("Drone%d : Going to waypoint %d", random_state+1, counter+1);
					set_destination(B[counter].x, B[counter].y, 5, goals[counter].psi);
					counter++;
						
				}
				else{
					//land after all waypoints are reached
					land();
				
				}	
			}
		}
		
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

// #include <gnc_functions.hpp>
// //include API 

// int main(int argc, char** argv)
// {
// 	//initialize ros 
// 	ros::init(argc, argv, "gnc_node");
// 	ros::NodeHandle gnc_node("~");
	
// 	//initialize control publisher/subscribers
// 	init_publisher_subscriber(gnc_node);

//   	// wait for FCU connection
// 	wait4connect();

// 	//wait for used to switch to mode GUIDED
// 	set_mode("GUIDED");

// 	//create local reference frame 
// 	initialize_local_frame();

// 	//request takeoff
// 	takeoff(3);

// 	//specify some waypoints 
// 	std::vector<gnc_api_waypoint> waypointList;
// 	gnc_api_waypoint nextWayPoint;
// 	nextWayPoint.x = 0;
// 	nextWayPoint.y = 0;
// 	nextWayPoint.z = 3;
// 	nextWayPoint.psi = 0;
// 	waypointList.push_back(nextWayPoint);
// 	nextWayPoint.x = 5;
// 	nextWayPoint.y = 0;
// 	nextWayPoint.z = 3;
// 	nextWayPoint.psi = -90;
// 	waypointList.push_back(nextWayPoint);
// 	nextWayPoint.x = 5;
// 	nextWayPoint.y = 5;
// 	nextWayPoint.z = 3;
// 	nextWayPoint.psi = 0;
// 	waypointList.push_back(nextWayPoint);
// 	nextWayPoint.x = 0;
// 	nextWayPoint.y = 5;
// 	nextWayPoint.z = 3;
// 	nextWayPoint.psi = 90;
// 	waypointList.push_back(nextWayPoint);
// 	nextWayPoint.x = 0;
// 	nextWayPoint.y = 0;
// 	nextWayPoint.z = 3;
// 	nextWayPoint.psi = 180;
// 	waypointList.push_back(nextWayPoint);
// 	nextWayPoint.x = 0;
// 	nextWayPoint.y = 0;
// 	nextWayPoint.z = 3;
// 	nextWayPoint.psi = 0;
// 	waypointList.push_back(nextWayPoint);


// 	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
// 	ros::Rate rate(2.0);
// 	int counter = 0;
// 	while(ros::ok())
// 	{
// 		ros::spinOnce();
// 		rate.sleep();
// 		if(check_waypoint_reached(.3) == 1)
// 		{
// 			if (counter < waypointList.size())
// 			{
// 				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
// 				counter++;	
// 			}else{
// 				//land after all waypoints are reached
// 				land();
// 			}	
// 		}	
		
// 	}
// 	return 0;
// }