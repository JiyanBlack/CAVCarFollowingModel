//-*-Mode: C++;-*-
#ifndef _simVehicleParticular_h_
#define _simVehicleParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2SimVehicle.h"

class A2BEHAVIORALEXPORT simVehicleParticular: public A2SimVehicle
{
private:
	float newAttribute;
	
public:
	simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh );
	~ simVehicleParticular ();
	const float getnewAttribute() const;
	void setnewAttribute ( float avalue);
};

#endif
