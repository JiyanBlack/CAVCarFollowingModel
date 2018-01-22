#include "simVehicleParticular.h"

simVehicleParticular:: simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh ) : A2SimVehicle( handlerVehicle, idhandler,isFictitiousVeh )
{
	newAttribute=0;
}

simVehicleParticular::~simVehicleParticular ()
{
}

const float simVehicleParticular::getnewAttribute() const
{
	return newAttribute;
}
void simVehicleParticular::setnewAttribute ( float avalue)
{
	newAttribute=avalue;
}