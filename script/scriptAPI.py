from AAPI import *
import random

seed = 1

random.seed(seed)
av = None
cav = None


def AAPILoad():
    AKIPrintString("AAPILoad")
    getRatio()
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
        ANGConnGetAttribute(
            AKIConvertFromAsciiString("GKSimVehicle::vehTypeState")), GKid,
        state)


def getRatio():
    global av, cav
    with open('percent.txt', 'r') as f:
        percent = f.read()
        ratio = percent.split('_')
        av = int(ratio[0])
        cav = int(ratio[1])
        print("Start creating vehicles with percent of av: " + str(av) +
              " , cav: " + str(cav))


def getAVState(id):
    GKid = ANGConnVehGetGKSimVehicleId(id)
    return ANGConnGetAttributeValueInt(
        ANGConnGetAttribute(
            AKIConvertFromAsciiString("GKSimVehicle::vehTypeState")), GKid)


def AAPIEnterVehicle(idVeh, idsection):
    AKIVehSetAsTracked(idVeh)
    rnd_num = random.random() * 100
    if rnd_num < av:  # set as av
        setAVState(idVeh, 1)
    elif rnd_num < (av + cav):  # set as cav
        setAVState(idVeh, 2)
    else:
        setAVState(idVeh, 0)  # set as default veh
    parameters = AKIVehTrackedGetStaticInf(idVeh)
    parameters.reactionTime = AKIGetSimulationStepTime()
    parameters.minDistanceVeh = 2.0
    parameters.headwayMin = 0.5
    parameters.maxAcceleration = 1.7
    parameters.normalDeceleration = -2.0
    parameters.maxDeceleration = -2.0
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
