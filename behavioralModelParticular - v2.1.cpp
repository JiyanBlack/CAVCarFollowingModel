#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string> 

#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <cassert>
using namespace std;
#define Tolerancia 0.01
#define DBL_MAX 1.7976931348623158e+308 

// To-do:
// new leader in the traffic intersection, so there's a lag when veh crossing the intersection
// try to take Vehicle class into this model.

double k = 0.3; // constant-speed error factor
double ka = 1.0; // constant factor in eq3, original value 1.0
double kv = 0.58; //constant factor in eq3, original value 0.58
double kd = 0.1; // constant factor in eq3, paper's value 0.1
double tsys = 0.5; // system response time setting for autonomous vehicles in eq6
double rmin = 2.0; // minimum allowed distance 2 meters in eq4

map<int, double> vehidToAcc; // map Veh to its latest acceleration (acceleration calculated by last simulation loop)

map<int, A2SimVehicle * > vehToLeader; // map veh's id to its leader
map<int, bool> hasEnterNode;
map<int, bool> hasStopped;

// sim logic:
// a Veh find a leader vehicle (Lveh).
// follows the vehicle, until it cross the intersection
// force setting the leader until it cross the intersection
// logic: once Veh's leader is stopped & start to move again, set the vehToLeader map to current leader vehicle.
//        Veh follow according to vehToLeader map
//        once Veh it self is entered a node before & now is in a section, reset the vehToLeader map, reset the hasEnterNode.

void print(string str) {
	AKIPrintString(str.c_str());
}

void setLeaderState(A2SimVehicle * vehicle, int state) {
	int id = vehicle->getId();
	int GKid = ANGConnVehGetGKSimVehicleId(id);
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::isLeader")), GKid, state);
}


int getLeaderState(A2SimVehicle * vehicle) {
	int id = vehicle->getId();
	int GKid = ANGConnVehGetGKSimVehicleId(id);
	return ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::isLeader")), GKid);
}

double get_aref_v(A2SimVehicle * Veh) {
	// Veh is the vehicle that about to update speed & position
	double v_intend = Veh->getFreeFlowSpeed();
	double v = Veh->getSpeed(Veh->isUpdated());
	return k * (v_intend - v);
}

double get_rref(A2SimVehicle * Veh, A2SimVehicle * Lveh) {
	// Lveh is the leader vehicle of Veh.
	double v = Veh->getSpeed(Veh->isUpdated());
	double rsafe = pow(v, 2) / 2 * (1.0 / (-Lveh->getDeceleration()) - 1.0 / (-Veh->getDeceleration()));
	double rsys = tsys * v;
	double rref = max(rsafe, max(rsys, rmin));
	return rref;
}

bool hasRealLeader(A2SimVehicle * Veh) { // check if Veh has a leader or not
	double lshift;
	A2SimVehicle * Lveh = Veh->getRealLeader(lshift);
	return Lveh != NULL;
}

A2SimVehicle * getRealLeader(A2SimVehicle * Veh) {
	double lshift;
	A2SimVehicle * Lveh = Veh->getRealLeader(lshift);
	return Lveh;
}

A2SimVehicle * getLeader(A2SimVehicle * Veh) {
	double lshift;
	A2SimVehicle * Lveh = Veh->getRealLeader(lshift);
	return Lveh;
}


bool isInNode(A2SimVehicle * Veh) {
	return Veh->isCurrentLaneInNode();
}

double isStopped(A2SimVehicle * Veh) {
	return Veh->getSpeed(Veh->isUpdated()) == 0.0;
}

bool isLeaderTrafficLight(A2SimVehicle * Veh) {
	double lshift;
	A2SimVehicle * Lveh = Veh->getLeader(lshift);
	return  Lveh->isTrafficLight();
}



double get_distance_to_leader(A2SimVehicle * Veh, A2SimVehicle * Lveh) {
	// get the real distance between Veh and Lveh (front bumper to front bumper, or rear bumper to rear bumper)
	// double r = Lveh->getPosition(Lveh->isUpdated()) - Veh->getPosition(Veh->isUpdated()) - Lveh->getLength();
	double r = - Veh->getPositionReferenceVeh(Veh->isUpdated(), Lveh , Lveh->isUpdated()) - Lveh->getLength();
	if (isStopped(Veh)) print("v1==============" + to_string(r));
	return r;
}

double get_distance_to_leader_v2(A2SimVehicle * Veh, A2SimVehicle * Lveh) {
	// get the real distance between Veh and Lveh (front bumper to front bumper, or rear bumper to rear bumper)
	double Xup, Vup, Xdw, Vdw;
	double r = Veh->getGap(0.0,Lveh, 2.0, Xup, Vup, Xdw, Vdw);
	if (isStopped(Veh)) print("v2==============" + to_string(r));
	return r;
}

double get_aref_d(A2SimVehicle * Veh, A2SimVehicle * Lveh) {
	double r = get_distance_to_leader_v2(Veh, Lveh);
	double rref = get_rref(Veh, Lveh);
	double lacc = vehidToAcc[Lveh->getId()];
	double lv = Lveh->getSpeed(Lveh->isUpdated());  // Lveh's velocity
	double v = Veh->getSpeed(Veh->isUpdated()); // Veh's velocity
	return ka * lacc + kv * (lv - v) + kd * (r - rref);
}

double get_acc_from_Lveh(A2SimVehicle * Veh, A2SimVehicle * Lveh) { // calculate the acceleration (if it is deceleration, it will be negative)
	double aref, acc, aref_v, aref_d, vehMaxAcc, vehMaxDec;
	aref_v = get_aref_v(Veh);
	if (Lveh != NULL) {
		aref_d = get_aref_d(Veh, Lveh);
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



double get_acc(A2SimVehicle * vehicle, A2SimVehicle * Lveh) {
	int id = vehicle->getId();
	if (vehToLeader[id] == NULL) {
		return get_acc_from_Lveh(vehicle, Lveh);
	}
	return get_acc_from_Lveh(vehicle, vehToLeader[id]);
}

bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle* vehicle, double& newpos, double& newspeed)
{
	// ========================== section for decide weather to use new model ==============
	A2SimVehicle * Lveh = getRealLeader(vehicle);
	double speed = vehicle->getSpeed(vehicle->isUpdated());
	int id = vehicle->getId();
	if (!hasRealLeader(vehicle) && isLeaderTrafficLight(vehicle)) {
		setLeaderState(vehicle, -1);
		return false;
	}

	// ======================= decide to use new model ==============
	
	if (isStopped(vehicle)) hasStopped[id] = true;

	if (hasRealLeader(vehicle) && !isStopped(vehicle) && hasStopped[id]) {
		// if has leader and it has stopped and it now start to move, set vehToLeader Map
		vehToLeader[id] = Lveh;
		hasStopped[id] = false;
	}

	if (isInNode(vehicle)) {
		// if veh is in node, set is hasEnterNode to true
		hasEnterNode[id] = true;
	}
	else {
		// else if veh is in section & has passed a node, reset all things
		if (hasEnterNode[id]) {
			vehToLeader.erase(id);
			hasEnterNode[id] = false;
		}
	}

	setLeaderState(vehicle, 2);	
	double acc = get_acc(vehicle, Lveh);
	vehidToAcc[id] = acc;
	newspeed = vehicle->getSpeed(vehicle->isUpdated()) + acc;
	// to make speed cannot be extreme small value: 0.000001, otherwise all vehicles cannot stop
	if (newspeed < 0.01) newspeed = 0.0;
	double increment = newspeed * getSimStep();
	newpos = vehicle->getPosition(vehicle->isUpdated()) + increment;
	print(to_string(id) + " acc is " + to_string(acc) + " , new speed is " + to_string(newspeed));
	return true;
}

double getRandNum() {
	int curNum = rand() % 100;
	return (double)curNum / 100.0;
}

behavioralModelParticular::behavioralModelParticular(): A2BehavioralModel()
{
   const unsigned short *randomSeedString = AKIConvertFromAsciiString( "GKReplication::randomSeedAtt" );
   seed = ANGConnGetAttributeValueInt( ANGConnGetAttribute( randomSeedString ), ANGConnGetReplicationId() );
   const unsigned short *param0= AKIConvertFromAsciiString( "GKExperiment::p_distance" );
   p_distance = ANGConnGetAttributeValueDouble( ANGConnGetAttribute( param0 ),ANGConnGetExperimentId()); 
}
 
behavioralModelParticular::~behavioralModelParticular(){}

simVehicleParticular * behavioralModelParticular::arrivalNewVehicle( void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh){
	simVehicleParticular * res = new simVehicleParticular( handlerVehicle, idHandler, isFictitiousVeh );
	if (!isFictitiousVeh) {
		res->setnewAttribute(2);
	}
	return res;
}


void behavioralModelParticular::removedVehicle( void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh )
{

}




bool behavioralModelParticular::evaluateLaneChanging(A2SimVehicle *vehicle,int threadId)
{		
	return false;
}

int behavioralModelParticular::evaluateHasTime2CrossYellowState(A2SimVehicle *vehicle,double distance2StopLine)
{		
	return -1;
}

int behavioralModelParticular::evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle,bool LeftLanePossible,bool RightLanePossible)
{
	return -1;
}

bool behavioralModelParticular::isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield)
{	
	return false;
}

// spain : vel = speed, actual = current, Deseada = desired, Resto = rest / remainder, Ciclo = Cycle

double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle,double VelActual,double VelDeseada, double RestoCiclo)
{
	return -1;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed (A2SimVehicle *vehicle,double Shift,A2SimVehicle *vehicleLeader,double ShiftLeader,bool controlDecelMax, bool aside,int time)
{
	return -1;
}

double behavioralModelParticular::computeMinimumGap(A2SimVehicle *vehicleUp,A2SimVehicle *vehicleDown,bool ImprudentCase,bool VehicleIspVehDw, int time)
{
	return -1;
}

bool behavioralModelParticular::avoidCollision(A2SimVehicle *vehicle,A2SimVehicle *vehiclePre,double ShiftPre)
{
	return false;
}
