#include "ManetSim.hh"

#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Manet");

int main(int argc, char *argv[])
{
    uint32_t numNodes = 10;
    std::string whichMobility = "random2d";
    std::string allowCollisions = "yes";
    uint32_t numPackets = 1;
    uint32_t packetSize = 1;
    double sendInterval = 1;
    double sendStartTime = 10.0;
    std::string routingProtocol = "olsr";

    CommandLine cmd;
    cmd.AddValue("numNodes", "number of nodes total", numNodes);
    cmd.AddValue("mobility", "set mobility model to use [grid|random2d]", whichMobility);
    cmd.AddValue("collisions", "allow packet collisions [yes|no]. If no, sends are staggered", allowCollisions);
    cmd.AddValue("numPackets", "set number of packets to send [default=1]", numPackets);
    cmd.AddValue("packetSize", "set packet size [default=1]", packetSize);
    cmd.AddValue("sendInterval", "set send time interval [default=1sec]", sendInterval);
    cmd.AddValue("startSend", "set time at which sends begin [default=10sec]", sendStartTime);
    cmd.AddValue("routingProtocol", "set MANET routing protocol to use [default=olsr]", routingProtocol);
    cmd.Parse(argc, argv);

    ManetSim manet(
        numNodes,
        whichMobility,
        allowCollisions == "yes" ? true : false,
        numPackets,
        packetSize,
        sendInterval,
        sendStartTime,
        routingProtocol);

    Simulator::Stop(Seconds(300.));
    Simulator::Run();
    Simulator::Destroy();

    manet.printNumPacketsReceived();
    
    return 0;
}

