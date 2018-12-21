#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <string> 
#include <vector>
#include <map>
using namespace std;
#define Tolerancia 0.01
#define DBL_MAX 1.7976931348623158e+308 

// model parameters
double k = 0.3; // constant-speed error factor
double ka = 1.0; // constant factor in eq3, original value 1.0
double kv = 0.58; //constant factor in eq3, original value 0.58
double kd = 0.1; // constant factor in eq3, paper's value 0.1
double tsys = 0.5; // system response time setting for autonomous vehicles in eq6				//LC This assumes all vehicles are CAV (other value for non CAV)
double rmin = 2.0; // minimum allowed distance 2 meters in eq4


// data structures:
map<int, double> idToAcc; // store vehId - acceleration map, because the CAV model requires previous vehicle's acceleration
map<int, int> idToVehType; // vehId - vehicle type map, no used in this full-CAV model

bool isActive = false;

void print(string str) {
	// print something in the console
	AKIPrintString(str.c_str());
}

void testSDKActive() {
	if (!isActive) {
		print("SDK is Working!");
		isActive = true;
	}
}

void setAVState(int id, int state) {
	int GKid = ANGConnVehGetGKSimVehicleId(id);
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::vehTypeState")), GKid, state);
	idToVehType[id] = state;
}

int getAVState(int id) {
	return 2;
	if (idToVehType[id] != 0) return idToVehType[id];
	int GKid = ANGConnVehGetGKSimVehicleId(id);
	int state = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::vehTypeState")), GKid);
	idToVehType[id] = state;
	return state;
}


double get_distance_to_leader(A2SimVehicle * Veh, A2SimVehicle * Lveh, double shift) {
	// get the real distance between Veh and Lveh (front bumper to front bumper, or rear bumper to rear bumper)
	double Xup, Vup, Xdw, Vdw;
	double r = Veh->getGap(0.0, Lveh, shift, Xup, Vup, Xdw, Vdw);
	print(to_string(r));
	return r;
}


double get_deceleration(A2SimVehicle *vehicle, double v, A2SimVehicle *Lveh, double lv, double gap) {
	// calculate the acceleration (if it is deceleration, it will be negative)
	int idveh = vehicle->getId();

	double aref, acc, aref_v, aref_d, vehMaxAcc, vehMaxDec;

	double v_intend = vehicle->getFreeFlowSpeed();

	aref_v = k * (v_intend - v);

	double dec = vehicle->getDeceleration();
	double rsafe = pow(v, 2) / 2 * (1.0 / (-Lveh->getDeceleration()) - 1.0 / (-vehicle->getDeceleration()));
	double rsys = tsys * v;
	double rref = max(rsafe, max(rsys, rmin));																//LC whats the point of two max functions?
	double lacc = idToAcc[Lveh->getId()];
	double r = gap + rmin;
	aref_d = ka * lacc + kv * (lv - v) + kd * (r - rref);
	aref = min(aref_v, aref_d);

	vehMaxAcc = vehicle->getAcceleration(); // Veh's max acceleration
	vehMaxDec = vehicle->getDeceleration(); // Veh's max deceleration
	acc = max(min(aref, vehMaxAcc), vehMaxDec);
	return acc;
}

double get_acceleration(A2SimVehicle *vehicle, double v, double v_intend) {
	// calculate the acceleration (if it is deceleration, it will be negative)
	double shift;
	A2SimVehicle * Lveh;
	Lveh = vehicle->getLeader(shift); // or use getRealLeader

	int idveh = vehicle->getId();

	double aref, acc, aref_v, aref_d, vehMaxAcc, vehMaxDec, r;
	double r_shift;
	aref_v = k * (v_intend - v);
	if (Lveh != NULL) {
		double Xup, Vup, Xdw, Vdw;
		double dec = vehicle->getDeceleration();
		r = vehicle->getGap(0.0, Lveh, shift, Xup, Vup, Xdw, Vdw);
		double rsafe = pow(v, 2) / 2 * (1.0 / (-Lveh->getDeceleration()) - 1.0 / (-vehicle->getDeceleration()));
		double rsys = tsys * v;
		double rref = max(rsafe, max(rsys, rmin));																//LC whats the point of two max functions?
		double lacc = idToAcc[Lveh->getId()];
		double lv = Lveh->getSpeed(Lveh->isUpdated());  // Lveh's velocity
		aref_d = ka * lacc + kv * (lv - v) + kd * (r - rref);
		aref = min(aref_v, aref_d);
	}
	else {
		aref = aref_v;
	}
	vehMaxAcc = vehicle->getAcceleration(); // Veh's max acceleration
	vehMaxDec = vehicle->getDeceleration(); // Veh's max deceleration
	acc = max(min(aref, vehMaxAcc), vehMaxDec);
	return acc;
}


behavioralModelParticular::behavioralModelParticular() : A2BehavioralModel()
{
	const unsigned short *randomSeedString = AKIConvertFromAsciiString("GKReplication::randomSeedAtt");
	seed = ANGConnGetAttributeValueInt(ANGConnGetAttribute(randomSeedString), ANGConnGetReplicationId());
	const unsigned short *param0 = AKIConvertFromAsciiString("GKExperiment::p_distance");
	p_distance = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(param0), ANGConnGetExperimentId());
	time = 0;
}

behavioralModelParticular::~behavioralModelParticular() {}

simVehicleParticular * behavioralModelParticular::arrivalNewVehicle(void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh) {
	simVehicleParticular * res = new simVehicleParticular(handlerVehicle, idHandler, isFictitiousVeh);
	if (!isFictitiousVeh) {
		res->setnewAttribute(2);
	}
	return res;
}
void behavioralModelParticular::removedVehicle(void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh)
{

}

void assignVehParas(int idveh, int AVState) {
	//set reaction time for HV AV CAV
	if (AVState == 2) {
		StaticInfVeh parameters = AKIVehGetStaticInf(idveh);
		parameters.reactionTime = 0.45;
		parameters.reactionTimeAtStop = 0.45;
		parameters.reactionTimeAtTrafficLight = 0.45;
		parameters.minDistanceVeh = rmin;
		AKIVehSetStaticInf(idveh, parameters);
	}
	else {
		StaticInfVeh parameters = AKIVehGetStaticInf(idveh);
		parameters.reactionTime = 0.9;
		parameters.reactionTimeAtStop = 0.9;
		parameters.reactionTimeAtTrafficLight = 0.9;
		AKIVehSetStaticInf(idveh, parameters);
	}
}


bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed)
{
	testSDKActive();

	int idveh = vehicle->getId();
	int AVState = getAVState(idveh);

	newspeed = vehicle->getAimsunCarFollowingSpeed();
	double currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	double increment = 0.0;
	double realAcc = (newspeed - currentSpeed) / getSimStep();
	idToAcc[idveh] = realAcc;

	if (newspeed >= currentSpeed) {
		increment = newspeed * getSimStep();
	}
	else {
		increment = 0.5*(newspeed + currentSpeed)*getSimStep();
	}
	newpos = vehicle->getPosition(vehicle->isUpdated()) + increment;
	return true;
}

bool behavioralModelParticular::evaluateLaneChanging(A2SimVehicle *vehicle, int threadId)
{
	bool res = vehicle->isValidLane(1);

	return false;
}

int behavioralModelParticular::evaluateHasTime2CrossYellowState(A2SimVehicle *vehicle, double distance2StopLine)
{
	return -1;
}

int behavioralModelParticular::evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle, bool LeftLanePossible, bool RightLanePossible)
{
	return -10;
}

bool behavioralModelParticular::isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield)
{
	return false;
}

double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double currentSpeed, double targetSpeed, double simStep)
{
	int idveh = vehicle->getId();
	int AVState = getAVState(idveh);

	if (AVState != 2) return -1;

	double acc = get_acceleration(vehicle, currentSpeed, targetSpeed);

	double newspeed = max(0., currentSpeed + simStep * acc);
	newspeed = min(targetSpeed, newspeed);

	return newspeed;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	return -1;
}
double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeedCore(A2SimVehicle *vehicle, double currentSpeed, A2SimVehicle *vehicleLeader, double leaderSpeed, double gap, double leaderDecelerationEstimated)
{
	int idveh = vehicle->getId();
	int AVState = getAVState(idveh);

	if (AVState != 2) return -1;

	double acc = get_deceleration(vehicle, currentSpeed, vehicleLeader, leaderSpeed, gap);

	double newspeed = max(0., currentSpeed + getSimStep() * acc);

	return newspeed;
}
double behavioralModelParticular::computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, bool VehicleIspVehDw, int time)
{
	return -1;
}


bool behavioralModelParticular::avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre)
{
	return false;
}

