//-*-Mode: C++;-*-
#ifndef _behavioralModelParticular_h_
#define _behavioralModelParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2BehavioralModel.h"
#include "simVehicleParticular.h"

enum Type {eNone, eReservedLane, eTurning, eNotAllowedInfluence, eNotAllowed, ePTStopInfluence, eOnRamp, eLaneClosureInfluence, eIncidentInfluence, eLaneClosure, eIncident, ePTStop};


class A2BEHAVIORALEXPORT behavioralModelParticular: public A2BehavioralModel
{
private:
	int seed;
	double p_distance;
public:
	behavioralModelParticular();
	~behavioralModelParticular();
	
	simVehicleParticular * arrivalNewVehicle( void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh);
	void removedVehicle( void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh );
	bool evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed);
	bool evaluateLaneChanging( A2SimVehicle *vehicle ,int threadId);
	bool isVehicleGivingWay( A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield);
	bool avoidCollision(A2SimVehicle *vehicle,A2SimVehicle *vehiclePre,double ShiftPre);
	double computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle,double VelActual,double VelDeseada, double RestoCiclo);
	double computeCarFollowingDecelerationComponentSpeed (A2SimVehicle *vehicle,double Shift,A2SimVehicle *vehicleLeader,double ShiftLeader,bool controlDecelMax=false, bool aside=false,int time=1);
	double computeMinimumGap(A2SimVehicle *vehicleUp,A2SimVehicle *vehicleDown,bool ImprudentCase=false, bool VehicleIspVehDw=false, int time=1);
	int evaluateHasTime2CrossYellowState(A2SimVehicle *vehicle,double distance2StopLine);
	int evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle,bool LeftLanePossible,bool RightLanePossible);
};

#endif
