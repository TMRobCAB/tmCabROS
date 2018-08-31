#include "ros/ros.h"
#include "std_msgs/String.h"

//#include "geometry_msgs/Vector3.h"

#include <geometry_msgs/WrenchStamped.h>

#include "ForceSensor/tm_force_sensor.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "msg_shared.hpp"

using namespace boost::interprocess;

#include <sstream>

#define DB_FORCE_DEFAULT 0.5

#define FORCEFILTFILTER_TAP_NUM 21

typedef struct {
	float history[FORCEFILTFILTER_TAP_NUM];
	unsigned int last_index;
} forceFiltFilter;

void forceFiltFilter_init(forceFiltFilter* f);
void forceFiltFilter_put(forceFiltFilter* f, float input);
float forceFiltFilter_get(forceFiltFilter* f);

static float filter_taps[FORCEFILTFILTER_TAP_NUM] = { 0.000617, 0.002290,
		0.005456, 0.010481, 0.017452, 0.026066, 0.035586, 0.044926, 0.052832,
		0.058133, 0.060000, 0.058133, 0.052832, 0.044926, 0.035586, 0.026066,
		0.017452, 0.010481, 0.005456, 0.002290, 0.000617 };

void forceFiltFilter_init(forceFiltFilter* f) {
	int i;
	for (i = 0; i < FORCEFILTFILTER_TAP_NUM; ++i)
		f->history[i] = 0;
	f->last_index = 0;
}

void forceFiltFilter_put(forceFiltFilter* f, float input) {
	f->history[f->last_index++] = input;
	if (f->last_index == FORCEFILTFILTER_TAP_NUM)
		f->last_index = 0;
}

float forceFiltFilter_get(forceFiltFilter* f) {
	float acc = 0;
	int index = f->last_index, i;
	for (i = 0; i < FORCEFILTFILTER_TAP_NUM; ++i) {
		index = index != 0 ? index - 1 : FORCEFILTFILTER_TAP_NUM - 1;
		acc += f->history[index] * filter_taps[i];
	};
	return acc;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "force_sensor");

	ros::NodeHandle n;

	std::string fNodeName = ros::this_node::getName();

	fNodeName = fNodeName.substr(1, fNodeName.size() - 1);

	ForceSample fVal;

	TmForceSensor fSensor;

	float deadBand;

	if (fSensor.Init() < 0) {

		ROS_ERROR("Force sensor not init");
		ROS_INFO("Force sensor shutting down");

		ros::shutdown();
	}

	ros::Publisher forcePub = n.advertise<geometry_msgs::WrenchStamped>(
			"force_cart", 1000);

	if (!n.getParam(fNodeName + "/dbForce", deadBand)) {

		ROS_ERROR("Parameter /dbForce not found");

		deadBand = DB_FORCE_DEFAULT;
	}

	geometry_msgs::WrenchStamped forceMsg;

	forceMsg.header.frame_id = "sensor_tip";

	forceMsg.wrench.torque.x = 0;
	forceMsg.wrench.torque.y = 0;
	forceMsg.wrench.torque.z = 0;

	ros::Rate loop_rate(100);

	ROS_INFO("Force sensor publishing");

	std::stringstream shmName;

	shmName.str("");
	shmName << "/force_cart_shared";

	shared_memory_object::remove(shmName.str().data());

//Create a shared memory object.
	shared_memory_object forceShm(create_only, shmName.str().data(),
			read_write);

//Set size
	forceShm.truncate(sizeof(force_shared));

//Map the whole shared memory in this process
	mapped_region regionForce(forceShm, read_write);

//Get the address of the mapped region
	void * forceAddr = regionForce.get_address();

//Construct the shared structure in memory
	force_shared * forceShmMsg = new (forceAddr) force_shared;

	forceFiltFilter fFiltX, fFiltY, fFiltZ;

	forceFiltFilter_init(&fFiltX);
	forceFiltFilter_init(&fFiltY);
	forceFiltFilter_init(&fFiltZ);

	while (ros::ok()) {

		fSensor.GetForce(fVal);

		forceMsg.wrench.force.x = -fVal.getFZ();
		forceMsg.wrench.force.y = fVal.getFX();
		forceMsg.wrench.force.z = -fVal.getFY();

//		forceFiltFilter_put(&fFiltX, forceMsg.wrench.force.x);
//		forceMsg.wrench.force.x = forceFiltFilter_get(&fFiltX);
//
//		forceFiltFilter_put(&fFiltY, forceMsg.wrench.force.y);
//		forceMsg.wrench.force.y = forceFiltFilter_get(&fFiltY);
//
//		forceFiltFilter_put(&fFiltZ, forceMsg.wrench.force.z);
//		forceMsg.wrench.force.z = forceFiltFilter_get(&fFiltZ);

		if (fabs(forceMsg.wrench.force.x) <= deadBand)
			forceMsg.wrench.force.x = 0;
		else if (forceMsg.wrench.force.x > deadBand)
			forceMsg.wrench.force.x -= deadBand;
		else
			forceMsg.wrench.force.x += deadBand;

		if (fabs(forceMsg.wrench.force.y) <= deadBand)
			forceMsg.wrench.force.y = 0;
		else if (forceMsg.wrench.force.y > deadBand)
			forceMsg.wrench.force.y -= deadBand;
		else
			forceMsg.wrench.force.y += deadBand;

		if (fabs(forceMsg.wrench.force.z) <= deadBand)
			forceMsg.wrench.force.z = 0;
		else if (forceMsg.wrench.force.z > deadBand)
			forceMsg.wrench.force.z -= deadBand;
		else
			forceMsg.wrench.force.z += deadBand;

		forceShmMsg->mutex.lock();

		forceShmMsg->newSample = true;
		forceShmMsg->fX = forceMsg.wrench.force.x;
		forceShmMsg->fY = forceMsg.wrench.force.y;
		forceShmMsg->fZ = forceMsg.wrench.force.z;

		forceShmMsg->mutex.unlock();

		forcePub.publish(forceMsg);

		//ros::spinOnce();

		loop_rate.sleep();
	}

	ROS_INFO("Force sensor shutting down");

	return 0;
}
