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
map<int, int> vehLeaderMap;
map<int, Vehicle> idVehMap;

class Vehicle {
public:
	int leader, frontVehicle;
	A2SimVehicle * V;

	// leader state description:
	// -1 means a leader
	// 0 means uninitialised
	// >0 records the current leader's id

	Vehicle() {
		V = NULL;
		id = 0;
		hasFollowedLeader = false;
	}


	Vehicle(A2SimVehicle * A2SV) {
		V = A2SV;
		id = A2SV->getId();
		hasFollowedLeader = false;
	}

	void removeVehicle() {
		if (leaderPlatoonMap.count(id) > 0) {
			// disassemble the platoon
			removePlatoon(id);
		}
		idVehMap.erase(id);
		vehLeaderMap.erase(id);
	}

	void updateLeader() {
		A2SimVehicle * curLeader = V;
		double shift;
		int curLeaderId = id;

		while (curLeader->getRealLeader(shift) != NULL) {
			curLeader = curLeader->getRealLeader(shift);
			curLeaderId = vehLeaderMap[curLeader->getId()];
			if (curLeaderId > 0 ) {
				leader = curLeaderId;
				::setLeaderState(V, leader);
				return;
			}
		}

		::setLeaderState(curLeader, -1);
		vehLeaderMap[curLeaderId, -1];
		leader = -1;
	}

	bool evaluateCarFollowing(double& newpos, double& newspeed, double& simStep) {
		double increment = 0;
		// test is the leader valid;
		double speed;
		speed = V->getSpeed(V->isUpdated());
		double curPosition = V->getPosition(V->isUpdated());

		// update hasFollowedLeader:
		// if speed > 0 and has a leader id != 0, means it is a leader or following a leader:
		if (speed > 0 && leader != 0)
			hasFollowedLeader = true;

		// decide if remove platoon
		// if vehicle was a leader once and now it stopped again, disassemble the platoon
		if (speed == 0 && leader == -1 && hasFollowedLeader) {
			removePlatoon(id);
		}

		// decide if update leader
		// if the vehicle has never followed a vehicle (speed = 0 & do not have a leader), find a leader
		// if the vehicle has followed other vehicle once but now is stopped again, find a leader
		bool shouldFindLeader = (speed == 0.0 && (leader == 0 || hasFollowedLeader));
		if (shouldFindLeader) { // when stopped, update the leader state
			updateLeader();
			if (leader > 0) {
				leaderPlatoonMap[leader].push_back(id);
			}
		}


		// if current vehicle is leader and leave the intersection more than 100m, then remove the platoon
		//if (curLeaderState == -1 && curPosition > 200.0)
		//{
		//	removePlatoon(idveh);
		//}

		// if the vehicle has a leader , then follow the acceleration
		if (leader > 0)
		{
			newspeed = leaderVehicle->getSpeed(leaderVehicle->isUpdated());
			if (newspeed >= leaderVehicle->getSpeed(leaderVehicle->isUpdated())) {
				increment = newspeed * simStep;
			}
			else {
				increment = 0.5*(newspeed + V->getSpeed(V->isUpdated()))*simStep;
			}
			newpos = V->getPosition(V->isUpdated()) + increment;
			return true;
		}
		// else use the default value
		return false;
	}

private:
	int id;
	bool hasFollowedLeader;

};





void setLeaderState(A2SimVehicle * vehicle , int state) {
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
	(*idVehMap[leaderId]).hasFollowedLeader = false;
	for (vector<int>::const_iterator curid = followers.begin(); curid != followers.end(); ++curid) {
		idVehMap[*curid]->setLeaderState(0);
		(*idVehMap[*curid]).hasFollowedLeader = false;
	}
	map<int, vector<int>>::iterator iter = leaderPlatoonMap.find(leaderId);
	if (iter != leaderPlatoonMap.end())
		leaderPlatoonMap.erase(iter);
}


double getRandNum() {
	int curNum = rand() % 100;
	return (double)curNum / 100.0;
}

void print(char *string) {
	AKIPrintString(string);
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
	idVehMap[idveh]->removeVehicle();
}


bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle* vehicle, double& newpos, double& newspeed)
{
	int idveh = vehicle->getId();
	// test if the vehicle is recorded:
	if (idVehMap.count(idveh) == 0)
		idVehMap[idveh] = new Vehicle(vehicle);

	double simStep = getSimStep();

	return idVehMap[idveh]->evaluateCarFollowing(newpos, newspeed, simStep);
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

