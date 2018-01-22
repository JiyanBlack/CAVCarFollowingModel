#include "behavioralModelParticularCreator.h"
#include "behavioralModelParticular.h"
#include "A2BehavioralModelUtil.h"

A2BehavioralModelCreator * behavioralModelParticularFactory()
{
	A2BehavioralModelCreator * res = new behavioralModelParticularCreator();
	return res;
}

behavioralModelParticularCreator::behavioralModelParticularCreator () : A2BehavioralModelCreator(){}

behavioralModelParticularCreator::~ behavioralModelParticularCreator (){}

A2BehavioralModel * behavioralModelParticularCreator::newModel()
{
	A2BehavioralModel *res = new behavioralModelParticular();
	return res;
}

