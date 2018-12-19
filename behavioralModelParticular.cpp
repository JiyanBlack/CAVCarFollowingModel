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
	if (idToVehType.count(id) == 1) return idToVehType[id];
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


double get_acc(A2SimVehicle * Veh) {
	// calculate the acceleration (if it is deceleration, it will be negative)

	double shift;
	A2SimVehicle * Lveh;
	Lveh = Veh->getLeader(shift); // or use getRealLeader

	int idveh = Veh->getId();
	int idLveh = Lveh->getId();

	double aref, acc, aref_v, aref_d, vehMaxAcc, vehMaxDec, r;
	//double v_intend = Veh->getFreeFlowSpeed();																	//LC is getFreeFlowSpeed a good option?																								
	double freeFlowSpeed = Veh->getFreeFlowSpeed();
	int turnId = Veh->getIdNextTurning();
	void * turnSpeedAtt = ANGConnGetAttribute(AKIConvertFromAsciiString("GKTurning::speedAtt"));			//investigate memory leak issue
	double nextTurnSpeed = ANGConnGetAttributeValueDouble(turnSpeedAtt, turnId) / 3.6;						//Divided by 3.6 to convert to m/s
	double v_intend;
	if (turnId == -1) {
		v_intend = freeFlowSpeed;
	}
	else {
		v_intend = min(freeFlowSpeed, nextTurnSpeed);
	}


	double v = Veh->getSpeed(Veh->isUpdated());
	aref_v = k * (v_intend - v);
	if (Lveh != NULL) {
		double Xup, Vup, Xdw, Vdw;
		double dec = Veh->getDeceleration();
		r = Veh->getGap(0.0, Lveh, shift, Xup, Vup, Xdw, Vdw);
		double rsafe = pow(v, 2) / 2 * (1.0 / (-Lveh->getDeceleration()) - 1.0 / (-Veh->getDeceleration()));
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
	vehMaxAcc = Veh->getAcceleration(); // Veh's max acceleration
	vehMaxDec = Veh->getDeceleration(); // Veh's max deceleration
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

bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed)
{
	testSDKActive();

	int idveh = vehicle->getId();


	int AVState = getAVState(idveh);

// default values:
	//bool leftLCR = false;
	//bool rightLCR = false;
	//int applyCoopWithVeh = 0;
	//int receiveCoopFromVeh = 0;
	//int tryLCBehindVeh = 0;

	//Vehicle attributes
	int GKid = ANGConnVehGetGKSimVehicleId(idveh);				//Can use Yans getAVState() function or simply write code here
	bool leftLCR = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::leftLaneChangingRequestedForCooperation")), GKid);
	bool rightLCR = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::rightLaneChangingRequestedForCooperation")), GKid);
	int applyCoopWithVeh = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::applyingCooperationWithVehicle")), GKid);
	int receiveCoopFromVeh = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::receivingCooperationFromVehicle")), GKid);
	int tryLCBehindVeh = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::tryingToChangeLaneBehindVehicle")), GKid);
	//Need to consider similiar method to handle stop signs 

	double acc = get_acc(vehicle);
	double old_acc = 0;
	if (idToAcc[idveh] != 0) {
		old_acc = idToAcc[idveh];
	}
	idToAcc[idveh] = acc;

	double speed = vehicle->getSpeed(vehicle->isUpdated());

	// address the stopped vehicle issue:
	// if speed is low, use default model
	if (speed < 1.0) return false;

	//Test whether AV needs to become HV and apply "getAimsunCarFollowingSpeed()"
	//If applying/recieving lane changing cooperation attributes
	bool AV2HV = false;
	if (leftLCR == true || rightLCR == true || applyCoopWithVeh != 0 || receiveCoopFromVeh != 0 || tryLCBehindVeh != 0) {
		AV2HV = true;
	}

	//If on an on-ramp and less than 250m till end
	int obstacle = vehicle->getObstacleType();
	double distObst = vehicle->getDistance2Obstacle();
	bool onRamp = false;
	if (obstacle == 9 && distObst <= 250) {											//250 m is arbitrary value
		onRamp = true;
	}

	//Apply CAV model or Aimsun Gipps model
	if (AVState == 2 && AV2HV == false && onRamp == false) {
		StaticInfVeh parameters = AKIVehGetStaticInf(idveh);
		parameters.reactionTime = 0.45;
		AKIVehSetStaticInf(idveh, parameters);
		newspeed = max(0., speed + (getSimStep() / 2) * (old_acc + acc));
	}
	else {
		StaticInfVeh parameters = AKIVehGetStaticInf(idveh);
		parameters.reactionTime = 0.9;
		AKIVehSetStaticInf(idveh, parameters);
		newspeed = vehicle->getAimsunCarFollowingSpeed();
	}

	double increment = 0;
	if (newspeed >= vehicle->getSpeed(vehicle->isUpdated())) {
		increment = newspeed * getSimStep();
	}
	else {
		increment = 0.5*(newspeed + vehicle->getSpeed(vehicle->isUpdated()))*getSimStep();
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

double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double CurrentSpeed, double TargetSpeed, double deltaRT)
{
	return -1;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	return -1;
}
double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeedCore(A2SimVehicle *vehicle, double speedVehicle, A2SimVehicle *vehicleLeader, double speedLeader, double gap, double leaderDecelerationEstimated)
{
	return -1;
}
double behavioralModelParticular::computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, bool VehicleIspVehDw, int time)
{
	return -1;
}


bool behavioralModelParticular::avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre)
{
	return false;
}

