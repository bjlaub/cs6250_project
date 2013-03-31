#include "ManetSim.hh"
#include "GnutellaApp.h"

#include "ns3/flow-monitor-helper.h"

#include <stdexcept>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ManetSim");

ManetSim::ManetSim(
    unsigned int numNodes,
    const std::string & mobilityModel,
    bool allowCollisions,
    unsigned int numPackets,
    unsigned int packetSize,
    double sendInterval,
    double sendStartTime,
    const std::string & routingProtocol,
    const std::string & whichApplication)
    : m_wifi(WifiHelper::Default()),
      m_wifiMac(NqosWifiMacHelper::Default()),
      m_wifiPhy(YansWifiPhyHelper::Default()),
      m_wifiChannel(YansWifiChannelHelper::Default()),
      m_numPackets(numPackets),
      m_packetSize(packetSize),
      m_sendInterval(sendInterval),
      m_sendStartTime(sendStartTime),
      m_routingProtocol(routingProtocol),
      m_whichApplication(whichApplication)
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
        installAppsForNode(
            m_whichApplication,
            node,
            m_sendStartTime);
    }

    
    FlowMonitorHelper fmh;
    m_fm = fmh.InstallAll();

    AsciiTraceHelper ascii;
    m_wifiPhy.EnableAsciiAll(ascii.CreateFileStream("manet.tr"));

    if(m_routingProtocol == "olsr")
    {
        OlsrHelper olsr;
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(
            "manet.routes", std::ios::out);
        olsr.PrintRoutingTableAllEvery(Seconds(2), routingStream);
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
//    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
//    list.Add(staticRouting, 0);

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

void ManetSim::installAppsForNode(
    const std::string & whichApp,
    Ptr<Node> node,
    double startTime)
{
    if(whichApp == "packet_flooding")
        installPacketFlooder(node, startTime);
    else if(whichApp == "gnutella")
    {
        LogComponentEnable("GnutellaApp", LOG_LEVEL_INFO);
        installBaselineGnutella(node, startTime);
    }
    else if(whichApp == "gnutella_query_caching")
        throw std::runtime_error("gnutella_query_caching is not implemented");
    else
        throw std::runtime_error("invalid application type: " + whichApp);
}

void ManetSim::installPacketFlooder(Ptr<Node> node, double startTime)
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

void ManetSim::logResults() const
{
    if(m_whichApplication == "packet_flooding")
        printNumPacketsReceived();
    //else if(m_whichApplication == "gnutella")
    //    printGnutellaStats();

    std::map<FlowId, FlowMonitor::FlowStats> fs = m_fm->GetFlowStats();
    std::map<FlowId, FlowMonitor::FlowStats>::iterator it;

    uint32_t delay = 0;
    double lost_packets = 0;
    double qdv = 0;
    double j = 0;

    for(it = fs.begin(); it != fs.end(); it++)
    {
        lost_packets += (*it).second.lostPackets;
        qdv += (*it).second.jitterSum.GetSeconds();
        delay += (*it).second.delaySum.GetSeconds();
        j++;
    }
    
    NS_LOG_INFO("Average Number of Lost Packets: " << lost_packets/j);
    NS_LOG_INFO("Average PDV: " << qdv/j);
    NS_LOG_INFO("Average Delay: " << delay/j);
}

void ManetSim::printNumPacketsReceived() const
{
    for(PacketCounterType::const_iterator it = m_packetCounter.begin();
        it != m_packetCounter.end();
        ++it)
    {
        NS_LOG_INFO(it->first << " -- " << it->second);
    }
}

//void ManetSim::printGnutellaStats() const
//{
//    for(uint32_t n = 0; n < m_nodes.GetN(); ++n)
//    {
//        Ptr<Node> node = m_nodes.Get(n);
//        Ptr<GnutellaApp> gapp = Ptr<GnutellaApp> (dynamic_cast<GnutellaApp *> (PeekPointer (node->GetApplication(0))));
//        if(gapp)
//        {
//            gapp->LogMessageStats();
//        }
//    }
//}

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

void ManetSim::installBaselineGnutella(Ptr<Node> node, double startTime)
{
    unsigned int port = DEFAULT_LISTENING_PORT;
    Ptr<GnutellaApp> app;
    ApplicationContainer apps;

    // if this node is the very first one in the container, assume it's
    // the first node in the simulation and doesn't need a bootstrapping
    // node
    if(node == (*m_nodes.Begin()))
    {
        InetSocketAddress addr(m_addrs.GetAddress(0), port);
        Address nullAddress;
        app = CreateObject<GnutellaApp>(addr, nullAddress);
    }
    else
    {
        // otherwise, need to bootstrap this node somehow
        // TODO: what's the best way to do this???
        
        // find this node's position in the list of nodes
        unsigned int idx = 0;
        NodeContainer::Iterator it;
        for(it = m_nodes.Begin(); it != m_nodes.End(); ++it, ++idx)
        {
            if(*it == node)
                break;
        }

        InetSocketAddress addr(m_addrs.GetAddress(idx), port);
        InetSocketAddress boot(m_addrs.GetAddress(idx - 1), port);
        //InetSocketAddress boot(m_addrs.GetAddress(0), port);

        app = CreateObject<GnutellaApp>(addr, boot);
    }

    node->AddApplication(app);
    apps.Add(app);

    apps.Start(Seconds(startTime));
    // TODO: make this configurable
    apps.Stop(Seconds(299.));
}

