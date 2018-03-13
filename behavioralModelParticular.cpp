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
double k = 0.3; // constant-speed error factor
double ka = 1.0; // constant factor in eq3
double kv = 0.58; //constant factor in eq3
double kd = 0.1; // constant factor in eq3
double tsys = 0.5; // system response time setting for autonomous vehicles in eq6
double rmin = 2.0; // minimum allowed distance 2 meters in eq4

double get_aref_v(A2SimVehicle * A2V) {

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

}


bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle* vehicle, double& newpos, double& newspeed)
{
	int idveh = vehicle->getId();
	
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

