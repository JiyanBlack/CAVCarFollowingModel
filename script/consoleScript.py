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
    gkveh.addColumn("GKSimVehicle::vehTypeState", "Custom Vehicle Type", GKColumn.Int,
                    GKColumn.eExternal)
    print "Columns added!"


def findExperiment():
    experiments = model.getType("GKExperiment")
    for types in model.getCatalog().getUsedSubTypesFromType(experiments):
        for e in types.itervalues():
            if (e.getName() == "iMove Mixed Traffic Experiment"):
                print "Found target experiment."
                return e


def deleteAllReplications(exp):
    print "Remove all existing replicatoins..."
    for replication in exp.getReplications():
        exp.removeReplication(replication)


def loadModelGui(argv):
    global model, gui
    gui = GKGUISystem.getGUISystem().getActiveGui()
    # Load a network
    print "load ang: " + argv[3]
    if gui.loadNetwork(argv[3]):
        model = gui.getActiveModel()
        print "network loaded!"
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
        print "network loaded!"
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
    plugin = GKSystem.getSystem().getPlugin("GGetram")  # GGetramModule
    simulator = plugin.getCreateSimulator(model)
    for i in range(6):
        av_per = 0
        cav_per = 100 - i * 20
        name = "{0:03d}".format(av_per) + "_" + "{0:03d}".format(cav_per)
        replication = GKSystem.getSystem().newObject("GKReplication", model)
        replication.setExperiment(exp)
        replication.setName(name)
        replication.setRandomSeed(seed)
        exp.addReplication(replication)
        with open('percent.txt', 'w') as f:
            f.write(name)
        print("Create new replication: " + name)
        simulator.addSimulationTask(
            GKSimulationTask(replication, GKReplication.eBatch))
        simulator.simulate()


def main(argv):
    if gui_mode:
        loadModelGui(argv)
    else:
        loadModelConsole(argv)
    addColumns()
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
