from AAPI import *


def AAPILoad():
    AKIPrintString("AAPILoad")
    return 0


def AAPIInit():
    AKIPrintString("AAPIStart")
    return 0


def AAPIManage(time, timeSta, timeTrans, acycle):
    return 0


def AAPIPostManage(time, timeSta, timeTrans, acycle):
    return 0


def AAPIFinish():
    AKIPrintString("AAPIFinish")
    return 0


def AAPIUnLoad():
    return 0


def AAPIPreRouteChoiceCalculation(time, timeSta):
    return 0


def setAVState(id, state):
    GKid = ANGConnVehGetGKSimVehicleId(id)
    ANGConnSetAttributeValueInt(
        ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::isAV")),
        GKid, state)


def getAVState(id):
    GKid = ANGConnVehGetGKSimVehicleId(id)
    return ANGConnGetAttributeValueInt(
        ANGConnGetAttribute(AKIConvertFromAsciiString("GKSimVehicle::isAV")),
        GKid)


def AAPIEnterVehicle(idVeh, idsection):
    AKIVehSetAsTracked(idVeh)
    setAVState(idVeh, 1)
    parameters = AKIVehTrackedGetStaticInf(idVeh)
    parameters.reactionTime = AKIGetSimulationStepTime()
    parameters.minDistanceVeh = 2.0
    parameters.headwayMin = 0.5
    parameters.maxAcceleration = 3.0
    parameters.normalDeceleration = -3.5
    parameters.maxDeceleration = -3.5
    AKIVehSetStaticInf(idVeh, parameters)
    return 0


def AAPIExitVehicle(idVeh, idsection):
    AKIVehSetAsNoTracked(idVeh)
    return 0


def AAPIEnterPedestrian(idPedestrian, originCentroid):
    return 0


def AAPIExitPedestrian(idPedestrian, destinationCentroid):
    return 0


def AAPIEnterVehicleSection(idveh, idsection, atime):
    return 0


def AAPIExitVehicleSection(idveh, idsection, atime):
    return 0
