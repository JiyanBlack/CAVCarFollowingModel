#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <cassert>
using namespace std;
#define Tolerancia 0.01
#define DBL_MAX 1.7976931348623158e+308 

map<int, vector<int>> leaderPlatoonMap;


class Vehicle {
public:
	int leaderId, frontVehicleId;
	int id;
	A2SimVehicle * A2V;
	Vehicle *leaderVehicle;
	bool hasFollowedLeader;
	// leader state description:
	// -1 means a leader
	// 0 means uninitialised
	// >0 records the current leader's id

	Vehicle() {
		A2V = NULL;
		id = 0;
		hasFollowedLeader = false;
		leaderVehicle = NULL;
	}


	Vehicle(A2SimVehicle * A2SV) {
		A2V = A2SV;
		id = A2SV->getId();
		hasFollowedLeader = false;
		leaderVehicle = NULL;
	}

	void setLeaderState(int state) {
		int id = A2V->getId();
		int GKid = ANGConnVehGetGKSimVehicleId(id);
		ANGConnSetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::isLeader")), GKid, state);
	}

	int getLeaderState() {
		int id = A2V->getId();
		int GKid = ANGConnVehGetGKSimVehicleId(id);
		return ANGConnGetAttributeValueInt(ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::isLeader")), GKid);
	}

	void changeLeaderId(int targetId) {
		setLeaderState(targetId);
		leaderId = targetId;
	}

	void resetLeaderState() {
		setLeaderState(0);
		leaderId = 0;
		hasFollowedLeader = false;
	}
};

map<int, Vehicle *> idVehMap;


double getRandNum() {
	int curNum = rand() % 100;
	return (double)curNum / 100.0;
}

void print(char *string) {
	AKIPrintString(string);
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


void removePlatoon(int leaderId) {
	vector<int> followers = leaderPlatoonMap[leaderId];
	idVehMap[leaderId]->resetLeaderState();
	for (vector<int>::const_iterator curid = followers.begin(); curid != followers.end(); ++curid) {
		idVehMap[*curid]->resetLeaderState();
	}
	leaderPlatoonMap.erase(leaderId);
}


void removeVehicle(int id) {
	if (leaderPlatoonMap.count(id) > 0) {
		// disassemble the platoon
		removePlatoon(id);
	}
	idVehMap.erase(id);
}


void updateLeader(Vehicle * veh) {
	A2SimVehicle * curVeh = veh->A2V;
	double shift;
	int curLeaderId = curVeh->getId();

	while (curVeh->getRealLeader(shift) != NULL) {
		curVeh = curVeh->getRealLeader(shift);
		curLeaderId = getLeaderState(curVeh);
		if (curLeaderId > 0) {
			veh->changeLeaderId(curLeaderId);
			veh->leaderVehicle = idVehMap[curVeh->getId()]->leaderVehicle;
			return;
		}

		if (curLeaderId == -1) {
			veh->changeLeaderId(curVeh->getId());
			veh->leaderVehicle = idVehMap[curVeh->getId()];
			return;
		}
	}

	veh->changeLeaderId(-1);
}



bool evaluateCarFollow(double& newpos, double& newspeed, double& simStep, A2SimVehicle * A2V) {
	double increment = 0;
	Vehicle * V = idVehMap[A2V->getId()];
	double speed;
	speed = A2V->getSpeed(A2V->isUpdated());
	double curPosition = A2V->getPosition(A2V->isUpdated());
	int leaderId = V->leaderId;
	bool hasFollowedLeader = V->hasFollowedLeader;
	int id = V->id;
	Vehicle * leaderVehicle = V->leaderVehicle;
	// update hasFollowedLeader:
	// if speed > 0 and has a leader id != 0, means it is a leader or following a leader:
	if (speed > 0 && V->leaderId != 0)
		V->hasFollowedLeader = true;


	// decide if remove platoon
	// if vehicle was a leader once and now it stopped again, disassemble the platoon
	if (speed == 0 && leaderId == -1 && hasFollowedLeader) {
		removePlatoon(id);
	}

	// decide if update leader
	// if the vehicle has never followed a vehicle (speed = 0 & do not have a leader), find a leader
	// if the vehicle has followed other vehicle once but now is stopped again, find a leader
	bool shouldFindLeader = (speed == 0.0 && (leaderId == 0 || hasFollowedLeader));
	if (shouldFindLeader) { // when stopped, update the leader state
		updateLeader(V);
		if (leaderId > 0) {
			leaderPlatoonMap[leaderId].push_back(id);
		}
	}


	// if current vehicle is leader and leave the intersection more than 100m, then remove the platoon
	//if (curLeaderState == -1 && curPosition > 200.0)
	//{
	//	removePlatoon(idveh);
	//}

	// if the vehicle has a leader , then follow the acceleration
	if (leaderId > 0 && leaderVehicle->A2V != NULL)
	{
		newspeed = leaderVehicle->A2V->getSpeed(leaderVehicle->A2V->isUpdated());
		if (newspeed >= leaderVehicle->A2V->getSpeed(leaderVehicle->A2V->isUpdated())) {
			increment = newspeed * simStep;
		}
		else {
			increment = 0.5*(newspeed + A2V->getSpeed(A2V->isUpdated()))*simStep;
		}
		newpos = A2V->getPosition(A2V->isUpdated()) + increment;
		return true;
	}
	// else use the default value
	return false;
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
	int idveh = a2simVeh->getId();
	// remove the vehicle from the id-vehicle map
	removeVehicle(idveh);
}


bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle* vehicle, double& newpos, double& newspeed)
{
	int idveh = vehicle->getId();
	// test if the vehicle is recorded:
	if (idVehMap.count(idveh) == 0)
		idVehMap[idveh] = new Vehicle(vehicle);

	double simStep = getSimStep();

	return evaluateCarFollow(newpos, newspeed, simStep, vehicle);
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

