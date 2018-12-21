import sys

from PyANGBasic import *
from PyANGKernel import *
from PyANGConsole import *

model = None
console = None
gui = None

seed = 1

gui_mode = False


def addColumns():
    print "Initializing columns..."
    gkveh = model.getType("GKSimVehicle")
    gkveh.addColumn("GKSimVehicle::vehTypeState", "Custom Vehicle Type",
                    GKColumn.Int, GKColumn.eExternal)
    print "Columns added!"


def findExperiment():
    experiments = model.getType("GKExperiment")
    for types in model.getCatalog().getUsedSubTypesFromType(experiments):
        for e in types.itervalues():
            if (e.getName() == "Hybrid SRC AM Experiment 479870"):             #"iMove Mixed Traffic Experiment" For Yan's original Version (LC 10/11/18)
                print "Found target experiment."                    #"Liam Test Experiment" For Liams Test experiment (LC 10/11/18)
                return e


def deleteAllReplications(exp):                                     # Would this delete the existing replications? LC
    print "Remove all existing replications..."                     #Fixed Spelling replications LC 1/11/18
    NBReps = exp.getNbReplications()
    print(NBReps)
    for replication in exp.getReplications():                       #ISSUE: Doesn't delete all experiments
        exp.removeReplication(replication)



def loadModelGui(argv):
    global model, gui
    print "check 1"
    gui = GKGUISystem.getGUISystem().getActiveGui()
    # Load a network
    print "load ang: " + argv[3]
    if gui.loadNetwork(argv[3]):
        model = gui.getActiveModel()
        print "network loaded!1"
    else:
        gui.showMessage(GGui.eCritical, "Open error",
                        "Cannot load the network")


def loadModelConsole(argv):
    global model, console
    if len(argv) < 1:
        print str("please specify script and ANG files")
        return -1
    # Start a Console
    console = ANGConsole()
    # Load a network
    print "Start loading network..."
    if console.open(argv[1]):
        model = console.getModel()
        print "network loaded!2"
    else:
        console.getLog().addError("Cannot load the network")
        print "cannot load network"
        console.close()


def quit():
    if gui_mode:
        gui.save()
        gui.closeDocument(model)
        gui.forceQuit()
    else:
        console.close()


def runReplications(exp):
    #plugin = GKSystem.getSystem().getPlugin("GGetram")  # GGetramModule
    #simulator = plugin.getCreateSimulator(model)
    av_per =  100                                                                              # Choose % of AVs
    cav_per = 0
    name = "{0:03d}".format(av_per) + "_" + "{0:03d}".format(cav_per)
    replication = GKSystem.getSystem().newObject("GKReplication", model)
    replication.setExperiment(exp)
    replication.setName(name)
    replication.setRandomSeed(seed)
    exp.addReplication(replication)
    with open('percent.txt', 'w') as f:
        f.write(name)
    print("Create new replication: " + name)
    #simulator.addSimulationTask(GKSimulationTask(replication, GKReplication.eBatch))                        #use GKReplication.eBatch to run batch sim
    #replication.enableVehiclesInBatch(True)                                                                        #use eInteractiveAutoPlay for animated sim
    #simulator.addSimulationTask(GKSimulationTask(replication, GKReplication.eInteractiveAutoPlay))
    #simulator.simulate()
    GKSystem.getSystem().executeAction("play", replication, [], "")



def main(argv):
    if gui_mode:
        loadModelGui(argv)
    else:
        loadModelConsole(argv)
    addColumns()                                #uncommented LC 1/11/18
    exp = findExperiment()
    deleteAllReplications(exp)
    runReplications(exp)
    # quit()


# if len(sys.argv) == 4:
gui_mode = True
if gui_mode:
    main(sys.argv)
else:
    if __name__ == "__main__":
        sys.exit(main(sys.argv))
