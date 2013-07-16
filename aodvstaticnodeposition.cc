
#include "ns3/ipv4.h"
#include "ns3/ipv4-route.h"
#include "ns3/core-module.h"
#include "ns3/simulator-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/global-routing-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/wifi-phy.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-phy.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/random-variable.h"
#include "ns3/energy-module.h"
#include "ns3/aodv-routing-protocol.h"
#include "ns3/aodv-rtable.h"
#include "ns3/ipv4-routing-table-entry.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

using namespace ns3;
using namespace std;
void RemainingEnergy (double oldValue, double remainingEnergy);
void TotalEnergy (double oldValue, double totalEnergy); 
int t=0;
EnergySourceContainer sources;
class MeshTest
{
public:
    // Init test
    MeshTest ();

    // Configure test from command line arguments
    void Configure (int argc, char ** argv);
    // Run test
    int Run ();
private:
    int
    m_xSize; //x size of the grid
    int
    m_ySize; //y size of the grid
    double
    m_step; //separation between nodes
    bool printRoutes;
    double
    m_totalTime;
    uint32_t m_packetSize;
    bool
    pcap;
    std::string m_txrate;

    double
    m_txrate_dob;
    //to calculate the lenght of the simulation
    float m_timeTotal, m_timeStart, m_timeEnd;
    // List of network nodes
    NodeContainer nodes;
    // List of all wifi devices
    NetDeviceContainer wifiDevices;
    //Addresses of interfaces:
    Ipv4InterfaceContainer interfaces;
    // MeshHelper. Report is not static methods
    WifiHelper wifi_aodv;
private:
    // Create nodes and setup their mobility
    void CreateNodes ();
    // Install internet m_stack on nodes
    void InstallInternetStack ();
    // Install applications randomly
    void InstallApplicationRandom ();
};
MeshTest::MeshTest () :
    m_xSize (5),
    m_ySize (5),
    m_step (170),
    printRoutes (true),
    m_totalTime (240),

    m_packetSize (256),

    pcap (true),

    m_txrate ("10kbps"),
    m_txrate_dob (150) //needed in kbps for the trace file
{
}
void
MeshTest::Configure (int argc, char *argv[])
{
    CommandLine cmd;
    cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
    cmd.AddValue ("m_xSize", "m_xSize", m_xSize);
    cmd.AddValue ("m_ySize", "m_ySize", m_ySize);
    cmd.AddValue ("m_txrate", "m_txrate", m_txrate);
    cmd.AddValue ("m_txrate_dob", "m_txrate_dob", m_txrate_dob);
    cmd.Parse (argc, argv);
}
// remaining energy calculation
void RemainingEnergy (double oldValue, double remainingEnergy)
{
    double speed;
    ofstream outfile1;
    ofstream outfile3;

    outfile1.open("flows_drain_speed7.txt", ios::app);
    outfile3.open("nod_remain_energy7.txt", ios::app);


    //NS_LOG_UNCOND (Simulator::Now().GetSeconds ()<< "s Current remaining energy = " << remainingEnergy << "J");
    if(remainingEnergy == 0.0546874)
    {
        //NS_LOG_UNCOND("BATTERY IS LOW....,Change path");
        outfile3<<"battery is low..... change node";

    }

    // Energy draining speed calculation

    speed=(oldValue-remainingEnergy)/(Simulator::Now ().GetSeconds ()-(Simulator::Now ().GetSeconds ()-1));

    outfile1 <<"nodes draining speed   "<<speed<<"\n";
    if(speed<8.0000e-06)
    {
        //NS_LOG_UNCOND("draining speed is high....,Change node");
        outfile1<<"draining speed is high....,Change node";
    }

    if(remainingEnergy>0.0543445)
        outfile3 <<Simulator::Now().GetSeconds ()<< "s Current remaining energy = " << remainingEnergy << "J"<< "\n";

    outfile1.close();
    outfile3.close();

}
void TotalEnergy (double oldValue, double totalEnergy)
{

    //NS_LOG_UNCOND ( Simulator::Now().GetSeconds() <<"s Total energyconsumed by radio = " << totalEnergy << "J");

}
void MeshTest::CreateNodes ()
{
    int i, j;
    double m_txpower = 18.0; // dbm
    // Calculate m_ySize*m_xSize stations grid topology
    double position_x = 0;
    double position_y = 0;
    ListPositionAllocator myListPositionAllocator;
    for (i = 1; i <= m_xSize; i++){
        for (j = 1; j <= m_ySize; j++){
            std::cout << "Node at x = " << position_x << ", y = " << position_y << "\n";
            Vector3D n_pos (position_x, position_y, 0.0);
            myListPositionAllocator.Add(n_pos);
            position_y += m_step;
        }
        position_y = 0;
        position_x += m_step;
    }
    // Create the nodes
    nodes.Create (m_xSize*m_ySize);
    // Configure YansWifiChannel
    YansWifiPhyHelper WifiPhy = YansWifiPhyHelper::Default ();
    WifiPhy.Set("EnergyDetectionThreshold", DoubleValue (-89.0) );
    WifiPhy.Set("CcaMode1Threshold", DoubleValue (-62.0) );
    WifiPhy.Set("TxGain", DoubleValue (1.0) );
    WifiPhy.Set("RxGain", DoubleValue (1.0) );
    WifiPhy.Set("TxPowerLevels", UintegerValue (1) );
    WifiPhy.Set("TxPowerEnd", DoubleValue (m_txpower) );
    WifiPhy.Set("TxPowerStart", DoubleValue (m_txpower) );
    WifiPhy.Set("RxNoiseFigure", DoubleValue (7.0) );
    YansWifiChannelHelper WifiChannel;
    WifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    WifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",
                                    StringValue ("2.7"));
    WifiPhy.SetChannel (WifiChannel.Create ());
    wifi_aodv.SetStandard (WIFI_PHY_STANDARD_80211a);
    wifi_aodv.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                       StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (2500));
    // Install protocols and return container
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
    wifiMac.SetType ("ns3::AdhocWifiMac");
    wifiDevices = wifi_aodv.Install (WifiPhy, wifiMac, nodes);
    if (pcap)
    {
        WifiPhy.EnablePcapAll (std::string ("energy-aodv"));
    }

    // Place the protocols in the positions calculated before
    MobilityHelper mobility;
    mobility.SetPositionAllocator(&myListPositionAllocator);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);

}
void MeshTest::InstallInternetStack ()
{
    //configure AODV
    AodvHelper aodv;
    aodv.Set ("AllowedHelloLoss", UintegerValue (20));
    aodv.Set ("HelloInterval", TimeValue (Seconds (3)));
    aodv.Set ("RreqRetries", UintegerValue (5));
    aodv.Set ("ActiveRouteTimeout", TimeValue (Seconds (100)));
    aodv.Set ("DestinationOnly", BooleanValue (true));
    //Install the internet protocol stack on all nodes
    InternetStackHelper internetStack;
    internetStack.SetRoutingHelper (aodv);
    internetStack.Install (nodes);
    //Assign IP addresses to the devices interfaces (m_nIfaces)
    Ipv4AddressHelper address;
    address.SetBase ("192.168.1.0", "255.255.255.0");
    interfaces = address.Assign (wifiDevices);
    if (printRoutes)
    {
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("energy-aodv.routes", std::ios::out);
        aodv.PrintRoutingTableAllAt (Seconds (20), routingStream);
    }

}
void MeshTest::InstallApplicationRandom ()
{
    // Create as many connections as nodes has the grid
    int m_nconn = m_xSize * m_ySize;
    int i=0;
    int m_source, m_dest, m_dest_port;
    char num [2];
    char onoff [7];
    char sink [6];
    double start_time, stop_time, duration;
    // Set the parameters of the onoff application
    Config::SetDefault ("ns3::OnOffApplication::PacketSize",
                        UintegerValue (m_packetSize));
    Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue (m_txrate));
    ApplicationContainer apps [m_nconn];
    UniformVariable rand_nodes (0,m_ySize*m_xSize-1);
    UniformVariable rand_port (49000,49100);
    // 50 seconds for transitori are left at the beginning.
    UniformVariable a(50,m_totalTime-15);
    for (i = 0; i < m_nconn; i++){
        start_time = a.GetValue();
        ExponentialVariable b(30);
        duration = b.GetValue()+1;
        //
        //
        //
        //if
        //If the exponential variable gives us a value that added to the start time
        //is greater than the maximum permitted, this is changed for the maximum
        //10 seconds are left at the end to calculate well the statistics of each flow
        if( (start_time + duration) > (m_totalTime - 10)){
            stop_time = m_totalTime-10;
        }else{
            stop_time = start_time + duration;
        }
        // Create different names for the connections
        // (we can not use vectors for OnOffHelper)
        strcpy(onoff,"onoff");
        strcpy(sink,"sink");
        sprintf(num,"%d",i);
        strcat(onoff,num);
        strcat(sink,num);
        // Set random variables of the destination (server) and destination port.
        //m_dest = rand_nodes.GetInteger (0,m_ySize*m_xSize-1);
        m_dest=3;
        m_dest_port = rand_port.GetInteger (49000,49100);
        // Set random variables of the source (client)
        //m_source = rand_nodes.GetInteger (0,m_ySize*m_xSize-1);
        m_source=3;
        // Client and server can not be the same node.
        //while (m_source == m_dest){
        //m_source = rand_nodes.GetInteger (0,m_ySize*m_xSize-1);
        //}
        // Plot the connection values
        std::cout << "\n\t Node "<< m_source << " to " << m_dest;
        std::cout << "\n Start_time: " << start_time << "s";
        std::cout << "\n Stop_time: " << stop_time << "s\n";
        // Define UDP traffic for the onoff application
        OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress
                                                             (interfaces.GetAddress (m_dest), m_dest_port)));
        onoff.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable (1)));
        onoff.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable (0)));
        apps[i] = onoff.Install (nodes.Get(m_source));
        apps[i].Start (Seconds (start_time));
        apps[i].Stop (Seconds (stop_time));
        // Create a packet sink to receive the packets
        PacketSinkHelper sink ("ns3::UdpSocketFactory",InetSocketAddress
                               (interfaces.GetAddress (m_dest), 49001));
        apps[i] = sink.Install (nodes.Get (m_dest));
        apps[i].Start (Seconds (1.0));
    }

    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",
                          DoubleValue(0.1));
    sources = basicSourceHelper.Install(nodes);
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
    DeviceEnergyModelContainer deviceModels =
            radioEnergyHelper.Install(wifiDevices, sources);

    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>
            (sources.Get(0));
    Ptr<BasicEnergySource> basicSourcePtr1 = DynamicCast<BasicEnergySource>
            (sources.Get(1));
    Ptr<BasicEnergySource> basicSourcePtr2 = DynamicCast<BasicEnergySource>
            (sources.Get(2));
    Ptr<BasicEnergySource> basicSourcePtr3 = DynamicCast<BasicEnergySource>
            (sources.Get(3));
    Ptr<BasicEnergySource> basicSourcePtr4 = DynamicCast<BasicEnergySource>
            (sources.Get(4));
    Ptr<BasicEnergySource> basicSourcePtr5 = DynamicCast<BasicEnergySource>
            (sources.Get(5));
    Ptr<BasicEnergySource> basicSourcePtr6 = DynamicCast<BasicEnergySource>
            (sources.Get(6));
    Ptr<BasicEnergySource> basicSourcePtr7 = DynamicCast<BasicEnergySource>
            (sources.Get(7));
    Ptr<BasicEnergySource> basicSourcePtr8 = DynamicCast<BasicEnergySource>
            (sources.Get(8));
    Ptr<BasicEnergySource> basicSourcePtr9 = DynamicCast<BasicEnergySource>
            (sources.Get(9));
    Ptr<BasicEnergySource> basicSourcePtr10 = DynamicCast<BasicEnergySource>
            (sources.Get(10));
    Ptr<BasicEnergySource> basicSourcePtr11 = DynamicCast<BasicEnergySource>
            (sources.Get(11));
    Ptr<BasicEnergySource> basicSourcePtr12 = DynamicCast<BasicEnergySource>
            (sources.Get(12));
    Ptr<BasicEnergySource> basicSourcePtr13 = DynamicCast<BasicEnergySource>
            (sources.Get(13));
    Ptr<BasicEnergySource> basicSourcePtr14 = DynamicCast<BasicEnergySource>
            (sources.Get(14));
    Ptr<BasicEnergySource> basicSourcePtr15 = DynamicCast<BasicEnergySource>
            (sources.Get(15));
    Ptr<BasicEnergySource> basicSourcePtr16 = DynamicCast<BasicEnergySource>
            (sources.Get(16));
    Ptr<BasicEnergySource> basicSourcePtr17 = DynamicCast<BasicEnergySource>
            (sources.Get(17));
    Ptr<BasicEnergySource> basicSourcePtr18 = DynamicCast<BasicEnergySource>
            (sources.Get(18));
    Ptr<BasicEnergySource> basicSourcePtr19 = DynamicCast<BasicEnergySource>
            (sources.Get(19));
    Ptr<BasicEnergySource> basicSourcePtr20 = DynamicCast<BasicEnergySource>
            (sources.Get(20));
    Ptr<BasicEnergySource> basicSourcePtr21 = DynamicCast<BasicEnergySource>
            (sources.Get(21));
    Ptr<BasicEnergySource> basicSourcePtr22 = DynamicCast<BasicEnergySource>
            (sources.Get(22));
    Ptr<BasicEnergySource> basicSourcePtr23 = DynamicCast<BasicEnergySource>
            (sources.Get(23));
    Ptr<BasicEnergySource> basicSourcePtr24 = DynamicCast<BasicEnergySource>
            (sources.Get(24));

    ofstream outfile3;
    outfile3.open("nod_remain_energy.txt", ios::app);
    outfile3<<"node 1";
    basicSourcePtr -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 2";
    basicSourcePtr1 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 3";
    basicSourcePtr2 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 4";
    basicSourcePtr3 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 5";
    basicSourcePtr4 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 6";
    basicSourcePtr5 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 7";
    basicSourcePtr6 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 8";
    basicSourcePtr7 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 9";
    basicSourcePtr8 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 10";
    basicSourcePtr9 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 11";
    basicSourcePtr10 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 12";
    basicSourcePtr11 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 13";
    basicSourcePtr12 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 14";
    basicSourcePtr13 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 15";
    basicSourcePtr14 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 16";
    basicSourcePtr15 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 17";
    basicSourcePtr16 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 18";
    basicSourcePtr17 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 19";
    basicSourcePtr18 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 20";
    basicSourcePtr19 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 21";
    basicSourcePtr20 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 22";
    basicSourcePtr21 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 23";
    basicSourcePtr22 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 24";
    basicSourcePtr23 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    outfile3<<"node 25";
    basicSourcePtr24 -> TraceConnectWithoutContext("RemainingEnergy",MakeCallback(&RemainingEnergy));
    Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr ->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get(0);
    NS_ASSERT (basicRadioModelPtr != NULL);
    basicRadioModelPtr ->TraceConnectWithoutContext("TotalEnergyConsumption",MakeCallback(&TotalEnergy));

    Ptr<DeviceEnergyModel> basicRadioModelPtr1 = basicSourcePtr1 ->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get(0);
    NS_ASSERT (basicRadioModelPtr != NULL);
    basicRadioModelPtr1 ->TraceConnectWithoutContext("TotalEnergyConsumption",MakeCallback(&TotalEnergy));

}
int MeshTest::Run ()
{
    CreateNodes ();
    InstallInternetStack ();
    InstallApplicationRandom ();
    // Install FlowMonitor on all nodes
    ofstream os;
    os.open("nh.txt",ios::app);
    std::map<Ipv4Address, aodv::RoutingTableEntry>m_ipv4AddressEntry;
    for (std::map<Ipv4Address, aodv::RoutingTableEntry>::const_iterator iter = m_ipv4AddressEntry.begin (); iter != m_ipv4AddressEntry.end (); ++iter)

    {
        std::stringstream ipStream;

        iter->second.GetNextHop().Print(ipStream);
        std::string ip=ipStream.str();
        std::stringstream s(ip);
        os<<"Next hop"<<ip<<"\n";
    }
    os.close();


    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    m_timeStart=clock();
    Simulator::Stop (Seconds (m_totalTime));
    Simulator::Run ();
    // Define variables to calculate the metrics
    //Print per flow statistics
    monitor->CheckForLostPackets ();

    // Print per flow statistics
    monitor->CheckForLostPackets ();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>
            (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    uint32_t txPacketsum = 0;
    uint32_t rxPacketsum = 0;
    uint32_t rxBytesum = 0;
    uint32_t DropPacketsum = 0;
    uint32_t LostPacketsum = 0;
    double Delaysum = 0;
    ofstream of;
    of.open("staticpath.txt",ios::app);

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i =
         stats.begin (); i != stats.end (); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        of << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        txPacketsum += i->second.txPackets;
        rxPacketsum += i->second.rxPackets;
        rxBytesum += i->second.rxBytes;
        LostPacketsum += i->second.lostPackets;
        DropPacketsum += i->second.packetsDropped.size();
        Delaysum += i->second.delaySum.GetSeconds();

        std::cout << "  Tx :   " << i->second.txPackets << "(" << i->second.txBytes<< ")" << "\t";
        std::cout << "  Rx :   " << i->second.rxPackets << "(" << i->second.rxBytes<< ")" << "\n";

        std::cout << "************************************************************************** ******" << "\n";
        std::cout << "  Lost Packets:   " << i->second.lostPackets << "\n";

        std::cout << "  Drop Packets:   " << i->second.packetsDropped.size() << "\n";

        std::cout << "************************************************************************** ******" << "\n";
        std::cout << "  Packets Delivery Ratio: " << ((rxPacketsum * 100) /txPacketsum + 1) << "%" << "\n";
        std::cout << "  Packets Lost Ratio: " << ((LostPacketsum * 100) /txPacketsum) << "%" << "\n";
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 10.0 /1024 / 1024  << " Mbps\n";
        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ @@@@@@" << "\n";
    }
    of.close();
    std::cout << "************************************************************************** ******" << "\n";
    std::cout << "  All Tx Packets: " << txPacketsum <<"                  " << "  All Rx Packets: " << rxPacketsum << "\n";
    std::cout << "  All Lost Packets: " << LostPacketsum <<"                " << "  All Drop Packets: " << DropPacketsum << "\n";
    std::cout << "************************************************************************** ******" << "\n";
    std::cout << "  Packets Delivery Ratio: " << ((rxPacketsum * 100) /txPacketsum + 1) << "%" << "          " << "  Packets Lost Ratio: " << ((LostPacketsum * 100) /txPacketsum) << "%" << "\n";
    std::cout << "  All Delay: " << Delaysum / txPacketsum << "\n";
    std::cout << "  Throughput: " << rxBytesum * 8.0 / 10.0 / 1024 / 1024  << " Mbps\n";

    // We are only interested in the metrics of the data flows. This AODV
    // implementation create other flows with routing information at low bitrates,
    // so a margin is defined to ensure that only our data flows are filtered.
    return 0;
}
int main (int argc, char *argv[])
{
    MeshTest t;
    t.Configure (argc, argv);
    return t.Run();
}

