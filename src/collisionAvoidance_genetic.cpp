/*
Collision Avoidance Node

This node controls the collision avoidance algorithm; in our case, an artificial potential field approach was used.  For more information about the algorithm, visit https://sites.google.com/site/auburn2011uav/.  It is highly recommended that you gain some background about the artificial field approach before delving into the code.

The node contains a map which stores information about the UAVs in the airspace, which is updated upon the receipt of a new telemetry update.  From these updates, a new force acting on each UAV is calculated.  Based on that new force, a new waypoint is forwarded to the coordinator for the UAV to begin traveling towards.
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <queue>
#include <map>
#include <cmath>
#include <vector>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/standardDefs.h"

//our headers
#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/force.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/dubins.h"

//parameter file
#define PARAMETERS "/ros_workspace/AU_UAV_ROS/src/parameters.txt"

/* ROS service clients for calling services from the coordinator */
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestWaypointInfoClient;

int count; /* keeps count of the number of goToWaypoint services requested */
AU_UAV_ROS::flockVariables flockVars;		// stores the flock variables from a document of parameters
AU_UAV_ROS::forceVariables forceVars;		// stores the force variables from a document of parameters

/* constants */
const double EPSILON = 1e-6; /* used to check floating point numbers for equality */

std::map<int, AU_UAV_ROS::PlaneObject> pobjects;		/* map of planes in the airspace.  The key is the plane id of the aircraft */
std::vector<std::vector<AU_UAV_ROS::PlaneObject*> > flocks;	// keeps track of UAVs that are in flocks; inner vectors contain specific flocks

/* 
This function is run everytime new telemetry information from any plane is recieved.  With the new telemetry update, 
information about the plane is updated, including bearing, speed, current location, etc.  Additionally, we check to see
if the UAV has reached it's current destination, and, if so, update the destination of the UAV.
After updating, the calculateForces function is called to find a the new force acting on the UAV; from this new force,
a next waypoint is found and forwarded to the coordinator.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg);

/*
Returns a GoToWaypoint service to call the coordiantor with.  The service contains a new waypoint for the UAV to navigate to
based on the current location information contained within the Telemetry update and the direction of the force vector.  The new
waypoint is a second away from the current location.
*/
AU_UAV_ROS::GoToWaypoint updatePath(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg, AU_UAV_ROS::mathVector forceVector);

/*
This function checks to see if any planes have been removed from the airspace due to a collision.
To achieve this, the timestamp of the last update from each UAV is compared to the current time.
If the two times differ by more than three seconds, the UAV is deleted.  This function is meant for use with the 
evaluator, and may not be advantageous to use in the live testing due to packet loss, network latency, etc.
*/
void checkCollisions(void);

/*
This function checks to see if any planes can be clustered together from their distance apart and their waypoint
positions. It will shift the planes from one vector of non flocked planes to another one of flocked planes.
*/
void formFlocks(void);

/*
This separates the leader of a flock from the rest of the flock when it gets close to its waypoint.
*/
void deflock(int planeID);

/*
Checks for planes in flocks and groups them together. Searches and constructs vectors of flocks using checkBackward().
*/
void checkFlocks(void);

/*
Recursive function used in checkFlocks() to trace back along a series of planes
*/
void checkBackward(AU_UAV_ROS::PlaneObject* pobj1, std::vector<AU_UAV_ROS::PlaneObject*> &flock);

/*
Takes in a latitude and longitude and gives an x,y coordinate for that lat/long.
*/
double* findXYCoordinate(double longitude, double latitude);

/*
Takes in two plane object pointers and sets up a Dubins Path between them
*/
DubinsPath* setupDubins(AU_UAV_ROS::PlaneObject* pobj1, AU_UAV_ROS::PlaneObject* pobj2);

int main(int argc, char **argv)
{
	//set up variables
	FILE *fp;
	char filename [256];
	strcpy (filename, std::getenv("HOME"));
	strcat (filename, PARAMETERS);
	
	fp = fopen(filename, "r");
	
	if(fp != NULL){
		char buffer[256];
		//while we have something in the file
		int varCount = 0;
		while(fgets(buffer, sizeof(buffer), fp))
		{			
			//check for useless lines
			if(buffer[0] == '#' || isBlankLine(buffer)){
				//this line is a comment, ignore it
			}
			else{
				varCount++;

				if (varCount == 1) flockVars.clusterDistance = atof(buffer);
				if (varCount == 2) flockVars.flockTime = atof(buffer);
				if (varCount == 3) forceVars.maxForce = atof(buffer);
				if (varCount == 4) forceVars.alpha = atof(buffer);
				if (varCount == 5) forceVars.beta = atof(buffer);
				if (varCount == 6) forceVars.gamma = atof(buffer);
				if (varCount == 7) forceVars.alphaTop = atof(buffer);
				if (varCount == 8) forceVars.betaTop = atof(buffer);
				if (varCount == 9) forceVars.betaBot = atof(buffer);
				if (varCount == 10) forceVars.alphaTopF = atof(buffer);
				if (varCount == 11) forceVars.betaTopF = atof(buffer);
			}
		}
		
		fclose(fp);
	}
	
	else{
		std::cout<<"ERROR: file pointer is null" << std::endl;
	}
	
	forceVars.alphaBot = forceVars.alphaTop;
	forceVars.alphaBotF = forceVars.alphaTopF;
	forceVars.betaBotF = forceVars.betaBot;
	
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create clients for the goToWaypoint and requestWaypointInfo services
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	requestWaypointInfoClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//initialize counting
	count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();
		
	return 0;
}

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg)
{    
//	std::cout<<flockVars.clusterDistance<<std::endl;
//	std::cout<<flockVars.flockTime<<std::endl;	
	
	AU_UAV_ROS::GoToWaypoint goToWaypointSrv;
	AU_UAV_ROS::RequestWaypointInfo requestWaypointInfoSrv;
	int planeID = msg->planeID;

	/* 
	Remove any planes that have been involved in a collision.
	Note: This function is made for use with the evaluator node, and may not work optimally in the field.
	To check for collisions, it compares the current time with the last update time of each of the UAVs.  
	If the values differ by more than three seconds, it is assumed the plane has been deleted.  However, 
	packet losses / network latency may render issues in the real world.
	*/
	checkCollisions();

	/* request this plane's current normal destination */
	requestWaypointInfoSrv.request.planeID = planeID;
	requestWaypointInfoSrv.request.isAvoidanceWaypoint = false;
	requestWaypointInfoSrv.request.positionInQueue = 0;

	if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
		ROS_ERROR("Did not receive a response from the coordinator");
		return;
	}

	/* 
	if (plane is 2 seconds from current destination waypoint)
		if (plane is in a flock)
			deflock
	*/
/*
	if (findDistance(msg->currentLatitude, msg->currentLongitude, 
					requestWaypointInfoSrv.response.latitude, 
					requestWaypointInfoSrv.response.longitude) < 1.5*COLLISION_THRESHOLD){
		

		// this plane is a flock leader (it doesn't have a leader, but it does have a follower
		if (pobjects[planeID].getLeader() == -1 && pobjects[planeID].getFollower() != -1){
			
			int followerID = pobjects[planeID].getFollower();
			//std::cout << "UAVs " << planeID << " and " << followerID << " are deflocking." << std::endl;

			pobjects[pobjects[planeID].getFollower()].setLeader(-1);			// deflock the follower
			pobjects[planeID].setFollower(-1);									// deflock itself
		}
	}
	*/	
	
	/* 
	if (plane has reached current destination waypoint)
		move on to next normal destination waypoint in queue
	*/
	if (findDistance(msg->currentLatitude, msg->currentLongitude, 
					requestWaypointInfoSrv.response.latitude, 
					requestWaypointInfoSrv.response.longitude) < COLLISION_THRESHOLD){

		/* request next normal destination */
		requestWaypointInfoSrv.request.positionInQueue = 1;

		if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
			ROS_ERROR("Did not recieve a response from the coordinator");
			return;
		}
	}

	/* 
	Pseudocode for the following code lines.
	if (this plane is not in the map of planes and the telemetry update indicates that the plane is or has been moving towards a destination)
		if (the plane had been flying but we previously detected a collision and removed it)
			return - the plane is dead; the simulator is lagging behind 
		else
			the plane has registered a new TelemetryUpdate 
	else 
		return - the plane has just been initialized but it is not moving torwards a waypoint as of now 
	*/
	if (pobjects.find(planeID) == pobjects.end() && msg->currentWaypointIndex != -1){ 
		
		if (msg->currentWaypointIndex > 0){
			/* This plane is dead; it had previously been moving but was in a collision.
			The simulator is lagging behind and still reporting a telemetry update */
			return;
		}
		else{
			/* 
			a new plane has registered a TelemetryUpdate where the destination is not (0, 0)
			create new PlaneObject, collision radius is set to the distance traveled in one second
			*/
			AU_UAV_ROS::PlaneObject newPlane(MPS_SPEED, *msg); /* */
			pobjects[planeID] = newPlane; /* put the new plane into the map */

			/* update the destination of the PlaneObject with the value found with the requestWaypointInfoSrv call */
			AU_UAV_ROS::waypoint newDest; 

			newDest.latitude = requestWaypointInfoSrv.response.latitude;
			newDest.longitude = requestWaypointInfoSrv.response.longitude;
			newDest.altitude = requestWaypointInfoSrv.response.altitude;

			pobjects[planeID].setDestination(newDest);
		}
	}
	else if (pobjects.find(planeID) == pobjects.end()) /* new plane without waypoint set */
		return; 
	
	/* 
	Note: The requestWaypointInfo service returns a waypoint of -1000, -1000 when the UAV it cannot retrieve a destination from
	queue.

	Pseudocode:
	if (the plane has no destination){
		- for simulations, silence any force from this UAV so it does not affect flight paths by giving it an impossible location
		- update with the new time of latest update to avoid a false detection of a collision
	}
	else{
		update the plane with the new telemetry information

		if (the plane's destination has changed)
			update the map entry of the plane with this information
	}
	*/
	if (requestWaypointInfoSrv.response.latitude == -1000){ /* plane has no waypoints to go to */
		/* 
		useful for simulations, remove in real flights;		
		set location of finished planes to -1000, -1000 so no repulsive force is felt from this plane
		*/
		pobjects[planeID].setLatitude(-1000);
		pobjects[planeID].setLongitude(-1000);

		pobjects[planeID].update(); /* update the time of last update for this plane to acknowledge it is still in the air */
		return; 
	}
	else{
		pobjects[planeID].update(*msg); /* update plane with new position */

		/* if (destination has changed)
			update pobjects[planeID] with new destination */
		if (((pobjects[planeID].getDestination().latitude - requestWaypointInfoSrv.response.latitude) > EPSILON)
				|| ((pobjects[planeID].getDestination().longitude - requestWaypointInfoSrv.response.longitude) > EPSILON)
				|| ((pobjects[planeID].getDestination().latitude - requestWaypointInfoSrv.response.latitude) < EPSILON)
				|| ((pobjects[planeID].getDestination().longitude - requestWaypointInfoSrv.response.longitude) < EPSILON)){
			AU_UAV_ROS::waypoint newDest;

			newDest.latitude = requestWaypointInfoSrv.response.latitude;
			newDest.longitude = requestWaypointInfoSrv.response.longitude;
			newDest.altitude = requestWaypointInfoSrv.response.altitude;

			pobjects[planeID].setDestination(newDest);
		}
	}

	//form flocks	
	formFlocks();
	
	//organize flock information
	checkFlocks();
	
	// give the UAVs their flock numbers
	for (unsigned int i = 0; i < flocks.size(); i += 1)
	{
		for (unsigned int j = 0; j < flocks[i].size(); j += 1)
		{
			flocks[i][j]->setFlock(i);
			//std::cout << flocks[i][j]->getID();
		}
	}

	/* Most important line: Find the force acting on this UAV.  The plane is attracted to its waypoint, and repelled from other UAVs */
	AU_UAV_ROS::mathVector force = calculateForces(pobjects[planeID], pobjects, flocks, forceVars);

	/* 
	Create a goToWaypoint service to send to the coordinator based on the force vector just calculated.  The destination will be one
	second away from the current position.
	*/
	goToWaypointSrv = updatePath(msg, force);

	goToWaypointSrv.request.isAvoidanceManeuver = true; 
	goToWaypointSrv.request.isNewQueue = true;

	if (goToWaypointClient.call(goToWaypointSrv)){
		count++;
		//ROS_INFO("Received response from service request %d", (count-1));
	}
	else{
		ROS_ERROR("Did not receive response");
	}
	
	//std::cout << planeID << "                             " << pobjects[planeID].getID() << std::endl;
}

AU_UAV_ROS::GoToWaypoint updatePath(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg, AU_UAV_ROS::mathVector forceVector){
	AU_UAV_ROS::GoToWaypoint goToWaypointSrv; /* destination to go to */
	double forceTheta = forceVector.getDirection(); /* angle of the force vector with respect to North bearing */
	double d = MPS_SPEED/EARTH_RADIUS; /* angular distance traveled in one second */

	AU_UAV_ROS::waypoint currentPosition, newPosition;
	currentPosition.latitude = msg->currentLatitude;
	currentPosition.longitude = msg->currentLongitude;
	currentPosition.altitude = msg->currentAltitude;

	newPosition = calculateCoordinate(currentPosition, forceTheta, d);	/* find new position one second away based on direction of force */

	/* set up new goToWaypoint service */
	goToWaypointSrv.request.planeID = msg->planeID;

	goToWaypointSrv.request.latitude = newPosition.latitude;
	goToWaypointSrv.request.longitude = newPosition.longitude;
	goToWaypointSrv.request.altitude = newPosition.altitude;

	return goToWaypointSrv;
}

void checkCollisions(void){
	std::queue<int> planesToDelete;

	/* Iterate through the list of planes to check for planes that have been removed due to collisions */
	for (std::map<int, AU_UAV_ROS::PlaneObject>::iterator iter = pobjects.begin(); iter != pobjects.end(); iter++){
		/*
		if (we have not heard from this plane for more than three seconds)
			add this plane's ID to the queue of planes to delete 
		*/
		if ((ros::Time::now().toSec() - (iter->second).getLastUpdateTime()) > 3.0){
			planesToDelete.push(iter->first);
		}
	}

	while (!planesToDelete.empty()){
		/* 
		if the plane to delete still exists
			remove it
		*/
		if (pobjects.find(planesToDelete.front()) != pobjects.end())
			pobjects.erase(planesToDelete.front());

		planesToDelete.pop();
	}
}

// form flocks based on their trajectories and waypoints
void formFlocks(void) {

	for (int i = 0; i < (int)pobjects.size(); i++){
		// stop UAVs from following themselves
		if (pobjects[i].getLeader() == pobjects[i].getID()){
			std::cout << "UAV " << pobjects[i].getID() << " tried to follow itself. Stopping it now." << std::endl;
			pobjects[i].setLeader(-1);
		}
		
		// deflock when a leader gets close to its waypoint
		if (findDistance(pobjects[i].getDestination().latitude, pobjects[i].getDestination().longitude,
		pobjects[i].getLatitude(), pobjects[i].getLongitude()) < 2*COLLISION_THRESHOLD) {
			deflock(i);
		}
		
		else{
	
			// detect clusters, check Dubins Paths, and form flocks
			for (int j = 0; j < (int)pobjects.size(); j++){
			
				// if we're not comparing the plane to itself
				if(i != j) {
							
					// stop UAVs from following the same plane
					if (pobjects[i].getLeader() == pobjects[j].getLeader() &&pobjects[i].getLeader() != -1){
						if (pobjects[pobjects[i].getLeader()].getFollower() == pobjects[i].getLeader()){
							pobjects[j].setLeader(-1);
						}
						else{
							pobjects[i].setLeader(-1);
						}
					}
				
					// FLOCKING CASE 1: both planes are independent
					if(pobjects[i].getLeader() == -1 && pobjects[i].getFollower() == -1 &&
						pobjects[j].getLeader() == -1 && pobjects[j].getFollower() == -1) {
				
						// check their waypoints' distances
						if (findDistance(pobjects[i].getDestination().latitude, pobjects[i].getDestination().longitude,
						pobjects[j].getDestination().latitude, pobjects[j].getDestination().longitude) < flockVars.clusterDistance) {
						
							// generate Dubins Paths and divide by speed to get time required to get to correct vector position
							DubinsPath* dubinsPathij = setupDubins(&pobjects[i],&pobjects[j]);
							double ijDubinsTime = dubins_path_length(dubinsPathij)/MPS_SPEED;
				
							DubinsPath* dubinsPathji = setupDubins(&pobjects[j],&pobjects[i]);
							double jiDubinsTime = dubins_path_length(dubinsPathji)/MPS_SPEED;
						
							int leadingUAV = -1;
							int followingUAV= -1;

							// if both Dubins Paths are short enough, decide leader based on distance to waypoint
							if (ijDubinsTime < flockVars.flockTime && jiDubinsTime < flockVars.flockTime){
					
								// find out which member is closer and make it the leader
								// if i is closer than j, make i the leader
								if (AU_UAV_ROS::cmpDistToDest(pobjects[i], pobjects[j])){		
									leadingUAV = pobjects[i].getID();
									followingUAV = pobjects[j].getID();
									if (leadingUAV == followingUAV) {					// stop rings
										break;
									}
									else {
										pobjects[leadingUAV].setFollower(followingUAV);
										pobjects[followingUAV].setLeader(leadingUAV);
										std::cout << "case 1 flocking " << leadingUAV << " and " << followingUAV << std::endl;
									}
								}
								else {
									leadingUAV = pobjects[j].getID();
									followingUAV = pobjects[i].getID();
									if (leadingUAV == followingUAV) {					// stop rings
										break;
									}
									else {
										pobjects[leadingUAV].setFollower(followingUAV);
										pobjects[followingUAV].setLeader(leadingUAV);
										std::cout << "case 1 flocking " << leadingUAV << " and " << followingUAV << std::endl;
									}
								}
							}
						
							
							// if only the Dubins Path of i is short enough, make j the leader				
							else if (ijDubinsTime < flockVars.flockTime){
								leadingUAV = pobjects[j].getID();
								followingUAV = pobjects[i].getID();
								if (leadingUAV == followingUAV) {					// stop rings
									break;
								}
								else {
									pobjects[leadingUAV].setFollower(followingUAV);
									pobjects[followingUAV].setLeader(leadingUAV);
								}
							}
						
							// if only the Dubins Path of j is short enough, make i the leader
							else if (jiDubinsTime < flockVars.flockTime){
								leadingUAV = pobjects[i].getID();
								followingUAV = pobjects[j].getID();
								if (leadingUAV == followingUAV) {					// stop rings
									break;
								}
								else {
									pobjects[leadingUAV].setFollower(followingUAV);
									pobjects[followingUAV].setLeader(leadingUAV);
									//std::cout << "case 1 flocking" << std::endl;
								}
							}
							
						}
					}
				
					// FLOCKING CASE 2: independent or tail plane leads a flock
					// i is independent or a tail; j is a flock leader (doesn't have a leader, but has a follower)
					else if (pobjects[i].getFollower() == -1 &&	pobjects[j].getLeader() == -1 && pobjects[j].getFollower() != -1){
					
						// if the waypoints are close
						if (findDistance(pobjects[i].getDestination().latitude, pobjects[i].getDestination().longitude,
						pobjects[j].getDestination().latitude, pobjects[j].getDestination().longitude) < flockVars.clusterDistance) {
						
							// only need one Dubins Path here- check from flock to independent/tail
							DubinsPath* dubinsPathji = setupDubins(&pobjects[j],&pobjects[i]);
							double jiDubinsTime = dubins_path_length(dubinsPathji)/MPS_SPEED;
						
							if (jiDubinsTime < flockVars.flockTime){
								if (pobjects[i].getLeader() != pobjects[j].getID()) {		// stop rings
									pobjects[j].setLeader(pobjects[i].getID());
									pobjects[i].setFollower(pobjects[j].getID());
									std::cout << "case 2 flocking " << i << " and " << j << std::endl;
								}
							}
						}
					}
				
					// FLOCKING CASE 3: independent or lead plane follows a flock
					// i is independent; j is the end of a flock (has a leader, but doesn't have a follower)
					else if (pobjects[i].getLeader() == -1 && pobjects[j].getLeader() != -1 && pobjects[j].getFollower() == -1){
					
						// if the waypoints are close
						if (findDistance(pobjects[i].getDestination().latitude, pobjects[i].getDestination().longitude,
						pobjects[j].getDestination().latitude, pobjects[j].getDestination().longitude) < flockVars.clusterDistance) {
						
							// only need one Dubins Path here- check from independent/lead to flock
							DubinsPath* dubinsPathij = setupDubins(&pobjects[i],&pobjects[j]);
							double ijDubinsTime = dubins_path_length(dubinsPathij)/MPS_SPEED;
						
							if (ijDubinsTime < flockVars.flockTime){
								if (pobjects[j].getLeader() != pobjects[i].getID()){		// stop rings
									pobjects[i].setLeader(pobjects[j].getID());
									pobjects[j].setFollower(pobjects[i].getID());
									std::cout << "case 3 flocking " << j << " and " << i << std::endl;
								}
							}
						}
					}
				}
			}
		}
	}	
}

void deflock(int planeID){
	if (pobjects[planeID].getLeader() == -1 && pobjects[planeID].getFollower() != -1){
	
		int followerID = pobjects[planeID].getFollower();
		pobjects[pobjects[planeID].getFollower()].setLeader(-1);			// deflock the follower
		pobjects[planeID].setFollower(-1);									// deflock itself
		std::cout << "UAVs " << planeID << " and " << followerID << " are deflocking" << std::endl;
	}
}

// organizes a 2D vector of flocks for use in force.cpp
void checkFlocks(void) {
	// set all the check states to false
	for (unsigned int i = 0; i < (int)pobjects.size(); i++){
		pobjects[i].setCheckState(false);
	}
	
	// clear the flock tracker
	flocks.clear();
	
	// look for leaders and trace them backward to group them into flocks
	for (unsigned int i = 0; i < (int)pobjects.size(); i++){
		// check forward
		if (!pobjects[i].getCheckState() && pobjects[i].getLeader() == -1){	// if this plane has not been checked yet and it is a leader
			std::vector<AU_UAV_ROS::PlaneObject*> newFlock ;
			checkBackward(&pobjects[i], newFlock);
			flocks.push_back(newFlock);
		}
	}
}

// recursive flock checking function used with checkFlocks()
void checkBackward(AU_UAV_ROS::PlaneObject* pobj1, std::vector<AU_UAV_ROS::PlaneObject*> &flock){
	pobj1->setCheckState(true);
	flock.push_back(pobj1);				// add this member to the flock vector

	if (pobj1->getFollower() == -1){
		// base case: do nothing, this is the very end of the flock
	}
	else if (pobj1->getFollower() != -1){
		checkBackward(&pobjects[pobj1->getFollower()], flock);
	}
	else std::cout << "checkBackward() error" << std::endl;
}

/*	takes in a latitude and longitude and gives an x,y coordinate
	for that lat/long.
*/
double* findXYCoordinate(double longitude, double latitude) {
    double lat1 = latitude * DEGREES_TO_RADIANS;
    double lat2 = NORTH_MOST_LATITUDE*DEGREES_TO_RADIANS;
    double long1 = longitude * DEGREES_TO_RADIANS;
    double long2 = WEST_MOST_LONGITUDE*DEGREES_TO_RADIANS;
        
    double deltaLat = lat2-lat1;
    double deltaLong= long2-long1;
    
    double x = pow(sin(0 / 2.0),2);
    x = x + cos(lat2)*cos(lat2)*pow(sin(deltaLong/2.0),2);
    x = 2.0 * asin(sqrt(x));
    
    double y = pow(sin(deltaLat/2.0),2);
    y = y + cos(lat1)*cos(lat2)*pow(sin(0/2.0),2);
    y = 2.0 * asin(sqrt(y));
    
    double result[2] = {EARTH_RADIUS*x,EARTH_RADIUS*y};
    double* ptr = new double[2];
    ptr[0] = result[0];
    if(latitude>NORTH_MOST_LATITUDE) {
        ptr[1] = result[1];
   }
    else {
        ptr[1] = -result[1];
    }
    return ptr;
}

DubinsPath* setupDubins(AU_UAV_ROS::PlaneObject* pobj1, AU_UAV_ROS::PlaneObject* pobj2) {
    DubinsPath* dubinsPath = new DubinsPath;
    
    double q0[3];
    double q1[3];
	
    double* xyposition1 = findXYCoordinate(pobj1->getLongitude(),pobj1->getLatitude());
	q0[0]=xyposition1[0];
	q0[1]=xyposition1[1];
    q0[2]=pobj1->getBearing()*DEGREES_TO_RADIANS;

    double* xyposition2 = findXYCoordinate(pobj2->getLongitude(),pobj2->getLatitude());
	q1[0]=xyposition2[0];
	q1[1]=xyposition2[1];
    q1[2]=pobj2->getBearing()*DEGREES_TO_RADIANS;

    dubins_init(q0,q1,RHO,dubinsPath);
    return dubinsPath;
}
