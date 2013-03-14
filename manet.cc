#include <map>
#include <fstream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/internet-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Manet");

//static void
//CourseChangeCallback (std::string path, Ptr<const MobilityModel> model)
//{
//    Vector position = model->GetPosition ();
//    std::cout << "CourseChange " 
//              << path 
//              << " x=" << position.x 
//              << ", y=" << position.y 
//              << ", z=" << position.z 
//              << std::endl;
//}

typedef std::map<std::string, unsigned int> PacketCounterType;
PacketCounterType g_packetCounter;

static void packetReceived(
    std::string path,
    Ptr<const Packet> packet,
    const Address & address)
{
    if(g_packetCounter.find(path) == g_packetCounter.end())
        g_packetCounter[path] = 1;
    else
        g_packetCounter[path] += 1;
}

int 
main (int argc, char *argv[])
{
    //LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
    //LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    //LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);

    uint32_t numNodes = 10;
    std::string whichMobility = "random2d";
    std::string allowCollisions = "yes";
    uint32_t numPackets = 1;
    uint32_t packetSize = 1;
    double sendInterval = 1;
    double sendStartTime = 10.0;
    CommandLine cmd;
    cmd.AddValue("numNodes", "number of nodes total", numNodes);
    cmd.AddValue("mobility", "set mobility model to use [grid|random2d]", whichMobility);
    cmd.AddValue("collisions", "allow packet collisions [yes|no]. If no, sends are staggered", allowCollisions);
    cmd.AddValue("numPackets", "set number of packets to send [default=1]", numPackets);
    cmd.AddValue("packetSize", "set packet size [default=1]", packetSize);
    cmd.AddValue("sendInterval", "set send time interval [default=1sec]", sendInterval);
    cmd.AddValue("startSend", "set time at which sends begin [default=10sec]", sendStartTime);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(numNodes);

    WifiHelper wifi = WifiHelper::Default();
    wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel");

    wifiMac.SetType("ns3::AdhocWifiMac");

    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager");

    wifiPhy.SetChannel(wifiChannel.Create());

    NetDeviceContainer nodeDevices = wifi.Install(wifiPhy, wifiMac, nodes);

    // enable OLSR
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;

    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);
    list.Add(olsr, 100);

    // add IPv4 protocol stack
    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(nodes);

    // assign addresses 
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer addrs = ipv4.Assign(nodeDevices);

    // create a mobility model
    MobilityHelper mobility;
    if(whichMobility == "grid")
    {
        mobility.SetPositionAllocator ("ns3::GridPositionAllocator", 
            "MinX", DoubleValue (0.0),
            "MinY", DoubleValue (0.0),
            "DeltaX", DoubleValue (5.0),
            "DeltaY", DoubleValue (10.0),
            "GridWidth", UintegerValue (3),
            "LayoutType", StringValue ("RowFirst"));
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install(nodes);
    }
    else
    {
        Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();

        Ptr<UniformRandomVariable> uni = CreateObject<UniformRandomVariable>();
        uni->SetAttribute("Min", DoubleValue(100));
        uni->SetAttribute("Max", DoubleValue(400));
        for(uint32_t i = 0; i < numNodes; ++i)
        {
            double x = uni->GetValue();
            double y = uni->GetValue();
            posAlloc->Add(Vector(x, y, 0.));
    
            //NS_LOG_UNCOND("starting point for node " << i << ": x=" << x << ", y=" << y);
        }

        mobility.SetPositionAllocator(posAlloc);
        mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
            "Bounds", RectangleValue(Rectangle(0, 500, 0, 500)),
            "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
            "Pause", StringValue("ns3::UniformRandomVariable[Min=0.2|Max=1.5]"));
    
        mobility.Install(nodes);

        //Config::Connect(
        //    "/NodeList/*/$ns3::MobilityModel/CourseChange",
        //    MakeCallback(&CourseChangeCallback));
    }

    // Setup applications

    uint16_t port = 12345;

    // install a PacketSink on every node
    PacketSinkHelper sink(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer serverApps = sink.Install(nodes);
    serverApps.Start(Seconds(9.0));

    Config::Connect(
        "/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
        MakeCallback(&packetReceived));

    // for each node, create and install a UdpEchoClient application that sends
    // 100 bytes every 1 second. Create one of these for EACH other node, i.e.
    // every node sends to ALL other nodes
    double nextStartTime = sendStartTime;
    for(uint32_t srcNode = 0; srcNode < numNodes; ++srcNode)
    {
        // pointer to the source node for this sender
        Ptr<Node> sourceNode = NodeList::GetNode(srcNode);
        
        double delta = 0.0;
        for(uint32_t trgNode = 0; trgNode < numNodes; ++trgNode)
        {
            if(srcNode == trgNode)
                continue;

            // pointer to target node for this sender
            Ptr<Node> sinkNode = NodeList::GetNode(trgNode);

            UdpEchoClientHelper sender(
                sinkNode->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(),
                port);
            sender.SetAttribute("MaxPackets", UintegerValue(numPackets));
            sender.SetAttribute("Interval", TimeValue(Seconds(sendInterval)));
            sender.SetAttribute("PacketSize", UintegerValue(packetSize));
            sender.SetAttribute("StartTime", TimeValue(Seconds(nextStartTime + delta)));
            sender.SetAttribute("StopTime", TimeValue(Seconds(nextStartTime + delta + 1.0)));

            sender.Install(sourceNode);

            if(allowCollisions == "no")
                delta += (1.0 / double(numNodes));
        }
        
        if(allowCollisions == "no")
            nextStartTime += 2.0;
    }

    // Setup tracing

//    AsciiTraceHelper ascii;
//    wifiPhy.EnableAsciiAll(ascii.CreateFileStream("manet.tr"));

    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(
        "manet.routes", std::ios::out);
    olsr.PrintRoutingTableAllEvery(Seconds(2), routingStream);

    Simulator::Stop(Seconds(300.));
    Simulator::Run();
    Simulator::Destroy();

    for(PacketCounterType::const_iterator it = g_packetCounter.begin();
        it != g_packetCounter.end();
        ++it)
    {
        std::cout << it->first << " -- " << it-> second << "\n";
    }
}

