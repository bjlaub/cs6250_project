#ifndef __MANETSIM_HH__
#define __MANETSIM_HH__

#include <map>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/dsdv-helper.h"
#include "ns3/internet-module.h"

class ManetSim
{
public:
    ManetSim(
        unsigned int numNodes,
        const std::string & mobilityModel,
        bool allowCollisions,
        unsigned int numPackets,
        unsigned int packetSize,
        double sendInterval,
        double sendStartTime,
        const std::string & routingProtocol);

    virtual ~ManetSim()
    { }

    void printNumPacketsReceived() const;

protected:
    typedef std::map<std::string, unsigned int> PacketCounterType;

    void createNode();
    void setupRoutingForNode(ns3::Ptr<ns3::Node> target_node);
    void installAppsForNode(ns3::Ptr<ns3::Node> node, double startTime);

    void installSimplePacketFlooder(ns3::Ptr<ns3::Node> node, double startTime);

    void packetReceived(
        std::string path,
        ns3::Ptr<const ns3::Packet> packet,
        const ns3::Address & address);

    ns3::NodeContainer m_nodes;
    ns3::WifiHelper m_wifi;
    ns3::NqosWifiMacHelper m_wifiMac;
    ns3::YansWifiPhyHelper m_wifiPhy;
    ns3::YansWifiChannelHelper m_wifiChannel;
    ns3::NetDeviceContainer m_nodeDevices;
    ns3::MobilityHelper m_mobility;
    ns3::Ipv4AddressHelper m_ipv4;
    ns3::Ipv4InterfaceContainer m_addrs;

    unsigned int m_numPackets;
    unsigned int m_packetSize;
    double m_sendInterval;
    double m_sendStartTime;

    std::string m_routingProtocol;

    PacketCounterType m_packetCounter;
};

#endif

