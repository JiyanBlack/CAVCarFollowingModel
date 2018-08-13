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
#include <fstream>
#include <iostream>
#include <valarray>
using namespace std;
#define Tolerancia 0.01
#define DBL_MAX 1.7976931348623158e+308 

double k = 0.3; // constant-speed error factor
double ka = 1.0; // constant factor in eq3, original value 1.0
double kv = 0.58; //constant factor in eq3, original value 0.58
double kd = 0.1; // constant factor in eq3, paper's value 0.1
double tsys = 0.5; // system response time setting for autonomous vehicles in eq6
double rmin = 2.0; // minimum allowed distance 2 meters in eq4

map<int, double> idToAcc;
map<int, int> idToVehType;
// void * attr = ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::vehTypeState"));

void print(string str) {
	AKIPrintString(str.c_str());
}

void setAVState(int id, int state) {
	int GKid = ANGConnVehGetGKSimVehicleId(id);
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::vehTypeState")), GKid, state);
}

int getAVState(int id) {
	int GKid = ANGConnVehGetGKSimVehicleId(id);
	int state = ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::vehTypeState")), GKid);
	return state;
}


double get_distance_to_leader(A2SimVehicle * Veh, A2SimVehicle * Lveh, double shift) {
	// get the real distance between Veh and Lveh (front bumper to front bumper, or rear bumper to rear bumper)
	double Xup, Vup, Xdw, Vdw;
	double r = Veh->getGap(0.0, Lveh, shift, Xup, Vup, Xdw, Vdw);
	return r;
}


double get_acc(A2SimVehicle * Veh) { // calculate the acceleration (if it is deceleration, it will be negative)
	double shift;
	A2SimVehicle * Lveh;
	Lveh = Veh->getLeader(shift); // or use getRealLeader
	double aref, acc, aref_v, aref_d, vehMaxAcc, vehMaxDec, r;
	double v_intend = Veh->getFreeFlowSpeed();
	double v = Veh->getSpeed(Veh->isUpdated());
	aref_v = k * (v_intend - v);
	if (Lveh != NULL) {
		double Xup, Vup, Xdw, Vdw;
		double r = Veh->getGap(0.0, Lveh, shift, Xup, Vup, Xdw, Vdw);
		double rsafe = pow(v, 2) / 2 * (1.0 / (-Lveh->getDeceleration()) - 1.0 / (-Veh->getDeceleration()));
		double rsys = tsys * v;
		double rref = max(rsafe, max(rsys, rmin));
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

bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle* vehicle, double& newpos, double& newspeed)
{
	double speed;
	int vehId;
	if (vehicle == NULL || vehicle->isFictitious()) return false;
	double simStep = getSimStep();
	// choose vehicle type
	speed = vehicle->getSpeed(vehicle->isUpdated());
	double acc = get_acc(vehicle);
	double old_acc = 0;
	if (idToAcc[vehId] != 0) {
		old_acc = idToAcc[vehId];
	}
	idToAcc[vehId] = acc;
	newspeed = speed + simStep / 2 * (old_acc + acc);
	newpos = vehicle->getPosition(vehicle->isUpdated()) + simStep / 2 * (newspeed + speed);
	return true;
}

behavioralModelParticular::behavioralModelParticular() : A2BehavioralModel()
{

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



bool behavioralModelParticular::evaluateLaneChanging(A2SimVehicle *vehicle, int threadId)
{
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

// spain : vel = speed, actual = current, Deseada = desired, Resto = rest / remainder, Ciclo = Cycle

double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double speed, double desiredSpeed, double simStep)
{
	return -1;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	return -1;
}

double behavioralModelParticular::computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, bool ImprudentCase, bool VehicleIspVehDw, int time)
{
	return -1;
}

bool behavioralModelParticular::avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre)
{
	return false;
}