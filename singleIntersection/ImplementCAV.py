from AAPI import *


def AAPILoad():
    AKIPrintString("AAPILoad")
    return 0


def AAPIInit():
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


def AAPIEnterVehicle(idVeh, idsection):
    AKIVehSetAsTracked(idVeh)
    parameters = AKIVehTrackedGetStaticInf(idVeh)

    parameters.reactionTime = AKIGetSimulationStepTime()
    # parameters.sensitivityFactor = 0.0  # To avoid car-following constraining the speed
    parameters.minDistanceVeh = 0.0
    parameters.headwayMin = 0.0
    parameters.maxAcceleration = 2.0
    parameters.normalDeceleration = -3.0
    parameters.maxDeceleration = -3.0

    res = AKIVehTrackedSetStaticInf(idVeh, parameters)
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
