#include "ManetSim.hh"

#include <stdexcept>
#include <sstream>

using namespace ns3;

ManetSim::ManetSim(
    unsigned int numNodes,
    const std::string & mobilityModel,
    bool allowCollisions,
    unsigned int numPackets,
    unsigned int packetSize,
    double sendInterval,
    double sendStartTime,
    const std::string & routingProtocol)
    : m_wifi(WifiHelper::Default()),
      m_wifiMac(NqosWifiMacHelper::Default()),
      m_wifiPhy(YansWifiPhyHelper::Default()),
      m_wifiChannel(YansWifiChannelHelper::Default()),
      m_numPackets(numPackets),
      m_packetSize(packetSize),
      m_sendInterval(sendInterval),
      m_sendStartTime(sendStartTime),
      m_routingProtocol(routingProtocol)
{
    m_wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
    m_wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    m_wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel");

    m_wifiMac.SetType("ns3::AdhocWifiMac");
    m_wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager");
    m_wifiPhy.SetChannel(m_wifiChannel.Create());

    m_ipv4.SetBase("10.1.1.0", "255.255.255.0");

    if(mobilityModel == "grid")
    {
        m_mobility.SetPositionAllocator ("ns3::GridPositionAllocator", 
            "MinX", DoubleValue (0.0),
            "MinY", DoubleValue (0.0),
            "DeltaX", DoubleValue (5.0),
            "DeltaY", DoubleValue (10.0),
            "GridWidth", UintegerValue (3),
            "LayoutType", StringValue ("RowFirst"));
        m_mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    }
    else if(mobilityModel == "random2d")
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
        }

        m_mobility.SetPositionAllocator(posAlloc);
        m_mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
            "Bounds", RectangleValue(Rectangle(0, 500, 0, 500)),
            "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
            "Pause", StringValue("ns3::UniformRandomVariable[Min=0.2|Max=1.5]"));
    }
    else
    {
        throw std::runtime_error("Unsupported mobility model: " + mobilityModel);
    }

    // create initial nodes
    for(unsigned int n = 0; n < numNodes; ++n)
        createNode();

    // install applications for initial nodes
    for(unsigned int n = 0; n < numNodes; ++n)
    {
        Ptr<Node> node = NodeList::GetNode(n);
        installAppsForNode(node, m_sendStartTime);
    }

///    // Setup applications
///
///    uint16_t port = 12345;
///
///    // install a PacketSink on every node
///    PacketSinkHelper sink(
///        "ns3::UdpSocketFactory",
///        InetSocketAddress(Ipv4Address::GetAny(), port));
///    ApplicationContainer serverApps = sink.Install(m_nodes);
///    serverApps.Start(Seconds(9.0));
///
///    Config::Connect(
///        "/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
///        MakeCallback(&ManetSim::packetReceived, this));
///
///    // for each node, create and install a UdpEchoClient application that sends
///    // 100 bytes every 1 second. Create one of these for EACH other node, i.e.
///    // every node sends to ALL other nodes
///    double nextStartTime = sendStartTime;
///    for(uint32_t srcNode = 0; srcNode < numNodes; ++srcNode)
///    {
///        // pointer to the source node for this sender
///        Ptr<Node> sourceNode = NodeList::GetNode(srcNode);
///        
///        double delta = 0.0;
///        for(uint32_t trgNode = 0; trgNode < numNodes; ++trgNode)
///        {
///            if(srcNode == trgNode)
///                continue;
///
///            // pointer to target node for this sender
///            Ptr<Node> sinkNode = NodeList::GetNode(trgNode);
///
///            UdpEchoClientHelper sender(
///                sinkNode->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(),
///                port);
///            sender.SetAttribute("MaxPackets", UintegerValue(numPackets));
///            sender.SetAttribute("Interval", TimeValue(Seconds(sendInterval)));
///            sender.SetAttribute("PacketSize", UintegerValue(packetSize));
///            sender.SetAttribute("StartTime", TimeValue(Seconds(nextStartTime + delta)));
///            sender.SetAttribute("StopTime", TimeValue(Seconds(nextStartTime + delta + 1.0)));
///
///            sender.Install(sourceNode);
///
///            if(!allowCollisions)
///                delta += (1.0 / double(numNodes));
///        }
///        
///        if(!allowCollisions)
///            nextStartTime += 2.0;
///    }

}

void ManetSim::createNode()
{
    m_nodes.Create(1);
    Ptr<Node> newNode = m_nodes.Get(m_nodes.GetN() - 1);
    
    setupRoutingForNode(newNode);

    NetDeviceContainer devs = m_wifi.Install(m_wifiPhy, m_wifiMac, newNode);
    m_addrs.Add(m_ipv4.Assign(devs));
    m_mobility.Install(newNode);

    NS_LOG_UNCOND("Created new node with ID " << newNode->GetId());
}

void ManetSim::setupRoutingForNode(Ptr<Node> target_node)
{
    OlsrHelper olsr;
    AodvHelper aodv;
    DsdvHelper dsdv;
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);

    if(m_routingProtocol == "olsr")
        list.Add(olsr, 100);
    else if(m_routingProtocol == "aodv")
        list.Add(aodv, 100);
    else if(m_routingProtocol == "dsdv")
        list.Add(dsdv, 100);
    else
        throw std::runtime_error("invalid routing protocol: " + m_routingProtocol);

    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(target_node);
}

void ManetSim::installAppsForNode(Ptr<Node> node, double startTime)
{
    // TODO: replace w/ something more interesting
    installSimplePacketFlooder(node, startTime);
}

void ManetSim::installSimplePacketFlooder(Ptr<Node> node, double startTime)
{
    uint16_t port = 12345;
    uint32_t srcNode = node->GetId();

    PacketSinkHelper sink(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer serverApps = sink.Install(node);
    serverApps.Start(Seconds(startTime));

    std::ostringstream oss;
    oss << "/NodeList/" << srcNode << "/ApplicationList/*/$ns3::PacketSink/Rx";

    Config::Connect(
        oss.str(),
        MakeCallback(&ManetSim::packetReceived, this));

    for(uint32_t trgNode = 0; trgNode < NodeList::GetNNodes(); ++trgNode)
    {
        if(srcNode == trgNode)
            continue;

        Ptr<Node> sinkNode = NodeList::GetNode(trgNode);

        UdpEchoClientHelper sender(
            sinkNode->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(),
            port);
        sender.SetAttribute("MaxPackets", UintegerValue(m_numPackets));
        sender.SetAttribute("Interval", TimeValue(Seconds(m_sendInterval)));
        sender.SetAttribute("PacketSize", UintegerValue(m_packetSize));
        //sender.SetAttribute("StartTime", TimeValue(Seconds(nextStartTime + delta)));
        //sender.SetAttribute("StopTime", TimeValue(Seconds(nextStartTime + delta + 1.0)));
        sender.SetAttribute("StartTime", TimeValue(Seconds(startTime)));

        sender.Install(node);
    }
}

void ManetSim::printNumPacketsReceived() const
{
    for(PacketCounterType::const_iterator it = m_packetCounter.begin();
        it != m_packetCounter.end();
        ++it)
    {
        std::cout << it->first << " -- " << it->second << "\n";
    }
}

void ManetSim::packetReceived(
    std::string path,
    Ptr<const Packet> packet,
    const Address & address)
{
    if(m_packetCounter.find(path) == m_packetCounter.end())
        m_packetCounter[path] = 1;
    else
        m_packetCounter[path] += 1;
}

