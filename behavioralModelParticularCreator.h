//-*-Mode: C++;-*-
#ifndef _behavioralModelParticularCreator_h_
#define _behavioralModelParticularCreator_h_

#include "A2BehavioralModelUtil.h"
#include "A2BehavioralModelCreator.h"

class A2BehavioralModel;

extern "C" A2BEHAVIORALEXPORT A2BehavioralModelCreator * behavioralModelParticularFactory();

class A2BEHAVIORALEXPORT behavioralModelParticularCreator : public A2BehavioralModelCreator
{
public:
	behavioralModelParticularCreator();
	~ behavioralModelParticularCreator();

	virtual A2BehavioralModel * newModel();
};

#endif
