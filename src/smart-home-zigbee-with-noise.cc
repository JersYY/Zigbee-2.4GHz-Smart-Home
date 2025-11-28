/*
 * Comprehensive ZigBee Smart Home Network Simulation with Performance Metrics
 * Enhanced with Gaussian Noise and Rayleigh Fading Channel Model
 *
 * Enhancements:
 * - Throughput calculation
 * - Packet loss rate
 * - Power consumption estimation
 * - End-to-end delay measurement
 * - Scalability testing with variable number of nodes
 * - Gaussian noise simulation (AWGN)
 * - Rayleigh fading with distance-based path loss
 */

#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/zigbee-module.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <string>
#include <cmath>
#include <random>

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

NS_LOG_COMPONENT_DEFINE("SmartHomeZigbeePerformance");

// Global container for all Zigbee stacks
ZigbeeStackContainer g_zigbeeStacks;

// Random number generators for noise and fading
std::random_device rd;
std::mt19937 gen(rd());

// Enhanced statistics tracking
struct NetworkStats {
    uint32_t packetsTransmitted = 0;
    uint32_t packetsReceived = 0;
    uint32_t packetsFailed = 0;
    uint32_t bytesTransmitted = 0;
    uint32_t bytesReceived = 0;
    uint32_t routeDiscoveries = 0;
    uint32_t joinAttempts = 0;
    uint32_t joinSuccesses = 0;
    uint32_t groupCommands = 0;

    // Channel quality metrics
    uint32_t packetsDroppedByNoise = 0;
    uint32_t packetsDroppedByFading = 0;
    std::vector<double> snrValues; // Signal-to-Noise Ratio samples
    std::vector<double> fadingCoefficients; // Rayleigh fading samples

    // Timing metrics
    Time firstPacketTime = Seconds(0);
    Time lastPacketTime = Seconds(0);

    // Delay tracking
    std::map<uint32_t, Time> packetSendTimes; // packet UID -> send time
    std::vector<double> delays; // in milliseconds

    // Power consumption (estimated)
    double totalTxPower = 0.0; // mW
    double totalRxPower = 0.0; // mW
    double totalIdlePower = 0.0; // mW

    // Per-node statistics
    std::map<uint32_t, uint32_t> nodePacketsSent;
    std::map<uint32_t, uint32_t> nodePacketsReceived;
};

NetworkStats g_stats;

// Power consumption constants (typical ZigBee values in mW)
const double TX_POWER = 35.0;      // Transmission power
const double RX_POWER = 25.0;      // Reception power
const double IDLE_POWER = 0.3;     // Idle/sleep power
const double TX_TIME_PER_BYTE = 0.032; // ms per byte at 250kbps

// Channel model parameters
const double NOISE_FLOOR_DBM = -95.0;      // Noise floor in dBm
const double TX_POWER_DBM = 0.0;           // Transmit power in dBm (1 mW)
const double FREQUENCY_GHZ = 2.4;          // ZigBee frequency
const double REFERENCE_DISTANCE = 1.0;     // Reference distance in meters
const double PATH_LOSS_EXPONENT = 3.0;     // Path loss exponent (indoor)
const double SNR_THRESHOLD_DB = 6.0;       // Minimum SNR for successful reception

// Group addresses
const Mac16Address GROUP_LIVING_ROOM = Mac16Address("00:01");
const Mac16Address GROUP_BEDROOM = Mac16Address("00:02");
const Mac16Address GROUP_ALL_LIGHTS = Mac16Address("00:03");

/**
 * Generate Rayleigh fading coefficient
 * Rayleigh distribution models amplitude of received signal in multipath fading
 */
double GenerateRayleighFading()
{
    // Rayleigh fading is generated from two independent Gaussian random variables
    std::normal_distribution<double> gaussian(0.0, 1.0);
    
    double real = gaussian(gen);
    double imag = gaussian(gen);
    
    // Rayleigh amplitude: sqrt(real^2 + imag^2)
    double amplitude = std::sqrt(real * real + imag * imag);
    
    // Normalize to have average power of 1
    amplitude /= std::sqrt(2.0);
    
    return amplitude;
}

/**
 * Calculate path loss based on distance (in dB)
 * Using log-distance path loss model
 */
double CalculatePathLoss(double distance)
{
    if (distance < REFERENCE_DISTANCE)
    {
        distance = REFERENCE_DISTANCE;
    }
    
    // Path loss = PL(d0) + 10*n*log10(d/d0)
    // For 2.4 GHz at 1m reference: PL(d0) ≈ 40 dB
    double pathLoss = 40.0 + 10.0 * PATH_LOSS_EXPONENT * std::log10(distance / REFERENCE_DISTANCE);
    
    return pathLoss;
}

/**
 * Generate Gaussian noise (AWGN - Additive White Gaussian Noise)
 * Returns noise power in dBm
 */
double GenerateGaussianNoise()
{
    std::normal_distribution<double> gaussian(0.0, 1.0);
    double noiseSample = gaussian(gen);
    
    // Convert to dBm scale (centered at noise floor)
    double noiseDbm = NOISE_FLOOR_DBM + noiseSample * 3.0; // 3 dB standard deviation
    
    return noiseDbm;
}

/**
 * Calculate received signal power with fading and path loss
 * Returns received power in dBm
 */
double CalculateReceivedPower(double distance, double& fadingCoeff)
{
    // Generate Rayleigh fading coefficient
    fadingCoeff = GenerateRayleighFading();
    double fadingDb = 20.0 * std::log10(fadingCoeff); // Convert to dB
    
    // Calculate path loss
    double pathLossDb = CalculatePathLoss(distance);
    
    // Received power = TX power - path loss + fading
    double rxPowerDbm = TX_POWER_DBM - pathLossDb + fadingDb;
    
    return rxPowerDbm;
}

/**
 * Calculate Signal-to-Noise Ratio (SNR)
 * Returns SNR in dB
 */
double CalculateSNR(double rxPowerDbm, double noisePowerDbm)
{
    // SNR = Signal Power - Noise Power (in dB)
    return rxPowerDbm - noisePowerDbm;
}

/**
 * Simulate packet reception with noise and fading
 * Returns true if packet is successfully received
 */
bool SimulateChannelEffects(uint32_t srcNodeId, uint32_t dstNodeId)
{
    // Get node positions
    Ptr<Node> srcNode = g_zigbeeStacks.Get(srcNodeId)->GetObject<ZigbeeStack>()->GetNode();
    Ptr<Node> dstNode = g_zigbeeStacks.Get(dstNodeId)->GetObject<ZigbeeStack>()->GetNode();
    
    Ptr<MobilityModel> srcMobility = srcNode->GetObject<MobilityModel>();
    Ptr<MobilityModel> dstMobility = dstNode->GetObject<MobilityModel>();
    
    double distance = srcMobility->GetDistanceFrom(dstMobility);
    
    // Calculate received power with fading
    double fadingCoeff;
    double rxPowerDbm = CalculateReceivedPower(distance, fadingCoeff);
    
    // Generate noise
    double noisePowerDbm = GenerateGaussianNoise();
    
    // Calculate SNR
    double snrDb = CalculateSNR(rxPowerDbm, noisePowerDbm);
    
    // Store statistics
    g_stats.snrValues.push_back(snrDb);
    g_stats.fadingCoefficients.push_back(fadingCoeff);
    
    // Check if packet is received successfully
    if (snrDb < SNR_THRESHOLD_DB)
    {
        // Packet dropped due to poor channel conditions
        if (fadingCoeff < 0.5) // Deep fade
        {
            g_stats.packetsDroppedByFading++;
        }
        else // Noise dominated
        {
            g_stats.packetsDroppedByNoise++;
        }
        return false;
    }
    
    return true;
}

/**
 * Print a formatted message with timestamp and node info
 */
void PrintMessage(Ptr<ZigbeeStack> stack, const std::string& message)
{
    std::cout << std::fixed << std::setprecision(3)
              << "[" << Simulator::Now().As(Time::S) << "] "
              << "Node " << stack->GetNode()->GetId()
              << " [" << stack->GetNwk()->GetNetworkAddress() << "]: "
              << message << std::endl;
}

/**
 * Calculate transmission time for a packet
 */
double CalculateTransmissionTime(uint32_t packetSize)
{
    // ZigBee data rate: 250 kbps
    // Add overhead: PHY header (6 bytes) + MAC header (~25 bytes) + NWK header (~8 bytes)
    uint32_t totalBytes = packetSize + 6 + 25 + 8;
    return (totalBytes * 8.0) / 250000.0 * 1000.0; // milliseconds
}

/**
 * Estimate power consumption for transmission
 */
void AddTransmissionPower(uint32_t packetSize)
{
    double txTime = CalculateTransmissionTime(packetSize);
    g_stats.totalTxPower += TX_POWER * txTime / 1000.0; // mW·s
}

/**
 * Estimate power consumption for reception
 */
void AddReceptionPower(uint32_t packetSize)
{
    double rxTime = CalculateTransmissionTime(packetSize);
    g_stats.totalRxPower += RX_POWER * rxTime / 1000.0; // mW·s
}

/**
 * APS Data Indication Callback (Enhanced with channel simulation)
 */
void ApsDataIndication(Ptr<ZigbeeStack> stack, ApsdeDataIndicationParams params, Ptr<Packet> p)
{
    // Note: In real implementation, channel effects would be applied at PHY layer
    // Here we simulate it at application layer for demonstration
    
    g_stats.packetsReceived++;
    g_stats.bytesReceived += p->GetSize();

    // Update per-node statistics
    uint32_t nodeId = stack->GetNode()->GetId();
    g_stats.nodePacketsReceived[nodeId]++;

    // Track last packet time
    g_stats.lastPacketTime = Simulator::Now();

    // Calculate delay if we have send time
    uint32_t packetUid = p->GetUid();
    if (g_stats.packetSendTimes.find(packetUid) != g_stats.packetSendTimes.end())
    {
        Time delay = Simulator::Now() - g_stats.packetSendTimes[packetUid];
        g_stats.delays.push_back(delay.GetMilliSeconds());
        g_stats.packetSendTimes.erase(packetUid);
    }

    // Estimate reception power
    AddReceptionPower(p->GetSize());

    std::string addrMode;
    if (params.m_dstAddrMode == ApsDstAddressMode::DST_ADDR16_DST_ENDPOINT_PRESENT)
    {
        addrMode = "UNICAST";
    }
    else if (params.m_dstAddrMode == ApsDstAddressMode::GROUP_ADDR_DST_ENDPOINT_NOT_PRESENT)
    {
        addrMode = "GROUPCAST";
        g_stats.groupCommands++;
    }
    else
    {
        addrMode = "UNKNOWN";
    }

    PrintMessage(stack, "RECEIVED " + addrMode + " DATA (Size: " +
                 std::to_string(p->GetSize()) + " bytes, Endpoint: " +
                 std::to_string(params.m_dstEndPoint) + ", Cluster: " +
                 std::to_string(params.m_clusterId) + ")");
}

/**
 * APS Data Confirm Callback (Track failures)
 */
void ApsDataConfirm(Ptr<ZigbeeStack> stack, ApsdeDataConfirmParams params)
{
    if (params.m_status != ApsStatus::SUCCESS)
    {
        g_stats.packetsFailed++;
        PrintMessage(stack, "DATA TRANSMISSION FAILED - Status: " +
                     std::to_string(static_cast<int>(params.m_status)));
    }
}

/**
 * Network Formation Confirm
 */
void NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params)
{
    if (params.m_status == NwkStatus::SUCCESS)
    {
        PrintMessage(stack, "Network formation SUCCESSFUL");
    }
    else
    {
        PrintMessage(stack, "Network formation FAILED - Status: " + std::to_string(params.m_status));
    }
}

/**
 * Network Discovery Confirm
 */
void NwkNetworkDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params)
{
    if (params.m_status == NwkStatus::SUCCESS)
    {
        PrintMessage(stack, "Network discovery completed - Found " +
                     std::to_string(params.m_netDescList.size()) + " network(s)");

        NlmeJoinRequestParams joinParams;
        CapabilityInformation capaInfo;
        capaInfo.SetDeviceType(ROUTER);
        capaInfo.SetAllocateAddrOn(true);

        joinParams.m_rejoinNetwork = JoiningMethod::ASSOCIATION;
        joinParams.m_capabilityInfo = capaInfo.GetCapability();
        joinParams.m_extendedPanId = params.m_netDescList[0].m_extPanId;

        g_stats.joinAttempts++;
        Simulator::ScheduleNow(&ZigbeeNwk::NlmeJoinRequest, stack->GetNwk(), joinParams);
    }
    else
    {
        PrintMessage(stack, "Network discovery FAILED - Status: " + std::to_string(params.m_status));
    }
}

/**
 * Join Confirm
 */
void NwkJoinConfirm(Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params)
{
    if (params.m_status == NwkStatus::SUCCESS)
    {
        g_stats.joinSuccesses++;
        PrintMessage(stack, "Joined network successfully");
        std::cout << "  Short Address: " << params.m_networkAddress << std::endl;

        NlmeStartRouterRequestParams startRouterParams;
        Simulator::ScheduleNow(&ZigbeeNwk::NlmeStartRouterRequest,
                              stack->GetNwk(), startRouterParams);
    }
    else
    {
        PrintMessage(stack, "Join FAILED - Status: " + std::to_string(params.m_status));
    }
}

/**
 * Route Discovery Confirm
 */
void NwkRouteDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeRouteDiscoveryConfirmParams params)
{
    g_stats.routeDiscoveries++;

    if (params.m_status == NwkStatus::SUCCESS)
    {
        PrintMessage(stack, "Route discovery SUCCESSFUL");
    }
    else
    {
        PrintMessage(stack, "Route discovery FAILED - Status: " + std::to_string(params.m_status));
    }
}

/**
 * Send temperature reading from sensor (Unicast) with channel simulation
 */
void SendTemperatureReading(Ptr<ZigbeeStack> sensorStack, Ptr<ZigbeeStack> coordinatorStack)
{
    uint32_t srcNodeId = sensorStack->GetNode()->GetId();
    uint32_t dstNodeId = coordinatorStack->GetNode()->GetId();
    
    // Simulate channel effects
    bool channelSuccess = SimulateChannelEffects(srcNodeId, dstNodeId);
    
    if (!channelSuccess)
    {
        PrintMessage(sensorStack, "Packet DROPPED by channel (noise/fading)");
        g_stats.packetsTransmitted++;
        g_stats.packetsFailed++;
        return;
    }
    
    uint8_t tempData[2] = {0xEB, 0x00};
    Ptr<Packet> p = Create<Packet>(tempData, 2);

    // Track send time
    g_stats.packetSendTimes[p->GetUid()] = Simulator::Now();

    if (g_stats.firstPacketTime == Seconds(0))
    {
        g_stats.firstPacketTime = Simulator::Now();
    }

    ApsdeDataRequestParams dataReqParams;
    ZigbeeApsTxOptions txOptions;

    dataReqParams.m_useAlias = false;
    dataReqParams.m_txOptions = txOptions.GetTxOptions();
    dataReqParams.m_srcEndPoint = 1;
    dataReqParams.m_dstEndPoint = 1;
    dataReqParams.m_clusterId = 0x0402;
    dataReqParams.m_profileId = 0x0104;
    dataReqParams.m_dstAddrMode = ApsDstAddressMode::DST_ADDR16_DST_ENDPOINT_PRESENT;
    dataReqParams.m_dstAddr16 = coordinatorStack->GetNwk()->GetNetworkAddress();

    g_stats.packetsTransmitted++;
    g_stats.bytesTransmitted += p->GetSize();

    // Update per-node statistics
    g_stats.nodePacketsSent[srcNodeId]++;

    // Estimate transmission power
    AddTransmissionPower(p->GetSize());

    PrintMessage(sensorStack, "Sending temperature reading to Coordinator");

    Simulator::ScheduleNow(&ZigbeeAps::ApsdeDataRequest,
                          sensorStack->GetAps(), dataReqParams, p);
}

/**
 * Send group command (Light control)
 */
void SendGroupCommand(Ptr<ZigbeeStack> sourceStack, Mac16Address groupAddr,
                      const std::string& commandName, uint8_t commandId)
{
    uint8_t cmdData[1] = {commandId};
    Ptr<Packet> p = Create<Packet>(cmdData, 1);

    // Track send time
    g_stats.packetSendTimes[p->GetUid()] = Simulator::Now();

    ApsdeDataRequestParams dataReqParams;
    ZigbeeApsTxOptions txOptions;

    dataReqParams.m_useAlias = false;
    dataReqParams.m_txOptions = txOptions.GetTxOptions();
    dataReqParams.m_srcEndPoint = 1;
    dataReqParams.m_clusterId = 0x0006;
    dataReqParams.m_profileId = 0x0104;
    dataReqParams.m_dstAddrMode = ApsDstAddressMode::GROUP_ADDR_DST_ENDPOINT_NOT_PRESENT;
    dataReqParams.m_dstAddr16 = groupAddr;

    g_stats.packetsTransmitted++;
    g_stats.bytesTransmitted += p->GetSize();

    // Update per-node statistics
    uint32_t nodeId = sourceStack->GetNode()->GetId();
    g_stats.nodePacketsSent[nodeId]++;

    // Estimate transmission power
    AddTransmissionPower(p->GetSize());

    std::ostringstream oss;
    oss << groupAddr;
    PrintMessage(sourceStack, "Sending GROUP command '" + commandName +
                 "' to group [" + oss.str() + "]");

    Simulator::ScheduleNow(&ZigbeeAps::ApsdeDataRequest,
                          sourceStack->GetAps(), dataReqParams, p);
}

/**
 * Add device endpoint to a group
 */
void AddToGroup(Ptr<ZigbeeStack> stack, Mac16Address groupAddr, uint8_t endpoint,
                const std::string& groupName)
{
    ApsmeGroupRequestParams groupParams;
    groupParams.m_groupAddress = groupAddr;
    groupParams.m_endPoint = endpoint;

    std::ostringstream oss;
    oss << groupAddr;
    PrintMessage(stack, "Adding endpoint " + std::to_string(endpoint) +
                 " to group '" + groupName + "' [" + oss.str() + "]");

    Simulator::ScheduleNow(&ZigbeeAps::ApsmeAddGroupRequest,
                          stack->GetAps(), groupParams);
}

/**
 * Calculate and print performance metrics
 */
void PrintPerformanceMetrics(uint32_t numNodes, double simTime)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "PERFORMANCE METRICS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Network Size: " << numNodes << " nodes" << std::endl;
    std::cout << "Simulation Time: " << simTime << "s" << std::endl;
    std::cout << "\n--- Packet Statistics ---" << std::endl;
    std::cout << "  Packets Transmitted:  " << g_stats.packetsTransmitted << std::endl;
    std::cout << "  Packets Received:     " << g_stats.packetsReceived << std::endl;
    std::cout << "  Packets Failed:       " << g_stats.packetsFailed << std::endl;
    std::cout << "  Bytes Transmitted:    " << g_stats.bytesTransmitted << std::endl;
    std::cout << "  Bytes Received:       " << g_stats.bytesReceived << std::endl;

    // Packet Delivery Ratio (PDR)
    double pdr = 0.0;
    if (g_stats.packetsTransmitted > 0)
    {
        pdr = (double)g_stats.packetsReceived / g_stats.packetsTransmitted * 100.0;
    }
    std::cout << "\n--- Delivery Performance ---" << std::endl;
    std::cout << "  Packet Delivery Ratio: " << std::fixed << std::setprecision(2)
              << pdr << "%" << std::endl;

    // Packet Loss Rate
    double lossRate = 100.0 - pdr;
    std::cout << "  Packet Loss Rate:      " << std::fixed << std::setprecision(2)
              << lossRate << "%" << std::endl;

    // Channel-specific losses
    std::cout << "\n--- Channel Quality Statistics ---" << std::endl;
    std::cout << "  Packets Dropped by Noise:  " << g_stats.packetsDroppedByNoise << std::endl;
    std::cout << "  Packets Dropped by Fading: " << g_stats.packetsDroppedByFading << std::endl;
    
    // SNR statistics
    if (!g_stats.snrValues.empty())
    {
        double sumSNR = 0.0, minSNR = g_stats.snrValues[0], maxSNR = g_stats.snrValues[0];
        for (double snr : g_stats.snrValues)
        {
            sumSNR += snr;
            if (snr < minSNR) minSNR = snr;
            if (snr > maxSNR) maxSNR = snr;
        }
        double avgSNR = sumSNR / g_stats.snrValues.size();
        
        std::cout << "  Average SNR:           " << std::fixed << std::setprecision(2)
                  << avgSNR << " dB" << std::endl;
        std::cout << "  Min SNR:               " << std::fixed << std::setprecision(2)
                  << minSNR << " dB" << std::endl;
        std::cout << "  Max SNR:               " << std::fixed << std::setprecision(2)
                  << maxSNR << " dB" << std::endl;
        std::cout << "  SNR Threshold:         " << std::fixed << std::setprecision(2)
                  << SNR_THRESHOLD_DB << " dB" << std::endl;
    }
    
    // Fading statistics
    if (!g_stats.fadingCoefficients.empty())
    {
        double sumFading = 0.0, minFading = g_stats.fadingCoefficients[0];
        double maxFading = g_stats.fadingCoefficients[0];
        for (double fading : g_stats.fadingCoefficients)
        {
            sumFading += fading;
            if (fading < minFading) minFading = fading;
            if (fading > maxFading) maxFading = fading;
        }
        double avgFading = sumFading / g_stats.fadingCoefficients.size();
        
        std::cout << "  Average Fading Coeff:  " << std::fixed << std::setprecision(3)
                  << avgFading << std::endl;
        std::cout << "  Min Fading Coeff:      " << std::fixed << std::setprecision(3)
                  << minFading << std::endl;
        std::cout << "  Max Fading Coeff:      " << std::fixed << std::setprecision(3)
                  << maxFading << std::endl;
    }

    // Throughput calculation
    if (g_stats.lastPacketTime > g_stats.firstPacketTime)
    {
        double duration = (g_stats.lastPacketTime - g_stats.firstPacketTime).GetSeconds();
        double throughputKbps = (g_stats.bytesReceived * 8.0) / duration / 1000.0;
        double throughputPps = g_stats.packetsReceived / duration;

        std::cout << "\n--- Throughput ---" << std::endl;
        std::cout << "  Data Duration:         " << std::fixed << std::setprecision(3)
                  << duration << "s" << std::endl;
        std::cout << "  Throughput:            " << std::fixed << std::setprecision(2)
                  << throughputKbps << " kbps" << std::endl;
        std::cout << "  Throughput:            " << std::fixed << std::setprecision(2)
                  << throughputPps << " packets/s" << std::endl;
        std::cout << "  Average Packet Size:   " << std::fixed << std::setprecision(1)
                  << (g_stats.packetsReceived > 0 ?
                      (double)g_stats.bytesReceived / g_stats.packetsReceived : 0.0)
                  << " bytes" << std::endl;
    }

    // Delay statistics
    if (!g_stats.delays.empty())
    {
        double sumDelay = 0.0;
        double minDelay = g_stats.delays[0];
        double maxDelay = g_stats.delays[0];

        for (double delay : g_stats.delays)
        {
            sumDelay += delay;
            if (delay < minDelay) minDelay = delay;
            if (delay > maxDelay) maxDelay = delay;
        }

        double avgDelay = sumDelay / g_stats.delays.size();

        std::cout << "\n--- End-to-End Delay ---" << std::endl;
        std::cout << "  Samples:               " << g_stats.delays.size() << std::endl;
        std::cout << "  Average Delay:         " << std::fixed << std::setprecision(3)
                  << avgDelay << " ms" << std::endl;
        std::cout << "  Minimum Delay:         " << std::fixed << std::setprecision(3)
                  << minDelay << " ms" << std::endl;
        std::cout << "  Maximum Delay:         " << std::fixed << std::setprecision(3)
                  << maxDelay << " ms" << std::endl;
    }

    // Power consumption estimation
    double totalSimTime = simTime * numNodes; // node-seconds
    g_stats.totalIdlePower = IDLE_POWER * totalSimTime;
    double totalPower = g_stats.totalTxPower + g_stats.totalRxPower + g_stats.totalIdlePower;

    std::cout << "\n--- Power Consumption (Estimated) ---" << std::endl;
    std::cout << "  TX Power:              " << std::fixed << std::setprecision(3)
              << g_stats.totalTxPower << " mW·s ("
              << (g_stats.totalTxPower / totalPower * 100.0) << "%)" << std::endl;
    std::cout << "  RX Power:              " << std::fixed << std::setprecision(3)
              << g_stats.totalRxPower << " mW·s ("
              << (g_stats.totalRxPower / totalPower * 100.0) << "%)" << std::endl;
    std::cout << "  Idle Power:            " << std::fixed << std::setprecision(3)
              << g_stats.totalIdlePower << " mW·s ("
              << (g_stats.totalIdlePower / totalPower * 100.0) << "%)" << std::endl;
    std::cout << "  Total Power:           " << std::fixed << std::setprecision(3)
              << totalPower << " mW·s" << std::endl;
    std::cout << "  Average Power/Node:    " << std::fixed << std::setprecision(3)
              << (totalPower / numNodes) << " mW·s" << std::endl;

    // Network efficiency
    std::cout << "\n--- Network Efficiency ---" << std::endl;
    std::cout << "  Join Success Rate:     " << std::fixed << std::setprecision(2)
              << (g_stats.joinAttempts > 0 ?
                  (double)g_stats.joinSuccesses / g_stats.joinAttempts * 100.0 : 0.0)
              << "%" << std::endl;
    std::cout << "  Route Discoveries:     " << g_stats.routeDiscoveries << std::endl;
    std::cout << "  Group Commands:        " << g_stats.groupCommands << std::endl;

    std::cout << "========================================\n" << std::endl;
}

/**
 * Export results to CSV for analysis
 */
void ExportResultsToCSV(uint32_t numNodes, double simTime, const std::string& filename)
{
    std::ofstream csvFile;
    bool fileExists = std::ifstream(filename).good();

    csvFile.open(filename, std::ios::app);

    // Write header if file is new
    if (!fileExists)
    {
        csvFile << "NumNodes,SimTime,PacketsSent,PacketsReceived,PacketsFailed,"
                << "BytesSent,BytesReceived,PDR,LossRate,ThroughputKbps,"
                << "AvgDelay,MinDelay,MaxDelay,TxPower,RxPower,IdlePower,"
                << "TotalPower,JoinSuccessRate,RouteDiscoveries,"
                << "PacketsDroppedNoise,PacketsDroppedFading,AvgSNR,MinSNR,MaxSNR,"
                << "AvgFading,MinFading,MaxFading\n";
    }

    // Calculate metrics
    double pdr = (g_stats.packetsTransmitted > 0) ?
                 (double)g_stats.packetsReceived / g_stats.packetsTransmitted * 100.0 : 0.0;
    double lossRate = 100.0 - pdr;

    double throughputKbps = 0.0;
    if (g_stats.lastPacketTime > g_stats.firstPacketTime)
    {
        double duration = (g_stats.lastPacketTime - g_stats.firstPacketTime).GetSeconds();
        throughputKbps = (g_stats.bytesReceived * 8.0) / duration / 1000.0;
    }

    double avgDelay = 0.0, minDelay = 0.0, maxDelay = 0.0;
    if (!g_stats.delays.empty())
    {
        double sumDelay = 0.0;
        minDelay = maxDelay = g_stats.delays[0];
        for (double delay : g_stats.delays)
        {
            sumDelay += delay;
            if (delay < minDelay) minDelay = delay;
            if (delay > maxDelay) maxDelay = delay;
        }
        avgDelay = sumDelay / g_stats.delays.size();
    }

    double totalSimTime = simTime * numNodes;
    g_stats.totalIdlePower = IDLE_POWER * totalSimTime;
    double totalPower = g_stats.totalTxPower + g_stats.totalRxPower + g_stats.totalIdlePower;

    double joinSuccessRate = (g_stats.joinAttempts > 0) ?
                             (double)g_stats.joinSuccesses / g_stats.joinAttempts * 100.0 : 0.0;

    // Channel quality metrics
    double avgSNR = 0.0, minSNR = 0.0, maxSNR = 0.0;
    if (!g_stats.snrValues.empty())
    {
        double sumSNR = 0.0;
        minSNR = maxSNR = g_stats.snrValues[0];
        for (double snr : g_stats.snrValues)
        {
            sumSNR += snr;
            if (snr < minSNR) minSNR = snr;
            if (snr > maxSNR) maxSNR = snr;
        }
        avgSNR = sumSNR / g_stats.snrValues.size();
    }

    double avgFading = 0.0, minFading = 0.0, maxFading = 0.0;
    if (!g_stats.fadingCoefficients.empty())
    {
        double sumFading = 0.0;
        minFading = maxFading = g_stats.fadingCoefficients[0];
        for (double fading : g_stats.fadingCoefficients)
        {
            sumFading += fading;
            if (fading < minFading) minFading = fading;
            if (fading > maxFading) maxFading = fading;
        }
        avgFading = sumFading / g_stats.fadingCoefficients.size();
    }

    // Write data
    csvFile << numNodes << ","
            << simTime << ","
            << g_stats.packetsTransmitted << ","
            << g_stats.packetsReceived << ","
            << g_stats.packetsFailed << ","
            << g_stats.bytesTransmitted << ","
            << g_stats.bytesReceived << ","
            << pdr << ","
            << lossRate << ","
            << throughputKbps << ","
            << avgDelay << ","
            << minDelay << ","
            << maxDelay << ","
            << g_stats.totalTxPower << ","
            << g_stats.totalRxPower << ","
            << g_stats.totalIdlePower << ","
            << totalPower << ","
            << joinSuccessRate << ","
            << g_stats.routeDiscoveries << ","
            << g_stats.packetsDroppedByNoise << ","
            << g_stats.packetsDroppedByFading << ","
            << avgSNR << ","
            << minSNR << ","
            << maxSNR << ","
            << avgFading << ","
            << minFading << ","
            << maxFading << "\n";

    csvFile.close();

    std::cout << "Results exported to: " << filename << std::endl;
}

int main(int argc, char* argv[])
{
    // Command line parameters
    uint32_t numNodes = 6;
    uint32_t simTime = 300;
    bool verbose = false;
    bool manyToOne = true;
    bool exportCSV = false;
    std::string csvFilename = "zigbee_performance.csv";
    bool enableNoise = true;
    bool enableFading = true;
    double snrThreshold = SNR_THRESHOLD_DB;

    CommandLine cmd;
    cmd.AddValue("numNodes", "Number of nodes in network", numNodes);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("verbose", "Enable verbose logging", verbose);
    cmd.AddValue("manyToOne", "Enable Many-to-One routing", manyToOne);
    cmd.AddValue("exportCSV", "Export results to CSV file", exportCSV);
    cmd.AddValue("csvFile", "CSV filename for results", csvFilename);
    cmd.AddValue("enableNoise", "Enable Gaussian noise simulation", enableNoise);
    cmd.AddValue("enableFading", "Enable Rayleigh fading simulation", enableFading);
    cmd.AddValue("snrThreshold", "SNR threshold in dB for successful reception", snrThreshold);
    cmd.Parse(argc, argv);

    // Validate parameters
    if (numNodes < 3)
    {
        std::cerr << "Error: Minimum 3 nodes required (1 coordinator + 2 routers)" << std::endl;
        return 1;
    }

    // Configure logging
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));

    if (verbose)
    {
        LogComponentEnable("ZigbeeNwk", LOG_LEVEL_DEBUG);
        LogComponentEnable("ZigbeeAps", LOG_LEVEL_DEBUG);
    }

    // Set random seed
    RngSeedManager::SetSeed(12345);
    RngSeedManager::SetRun(1);

    std::cout << "\n========================================" << std::endl;
    std::cout << "SMART HOME ZIGBEE NETWORK SIMULATION" << std::endl;
    std::cout << "WITH PERFORMANCE METRICS" << std::endl;
    std::cout << "AND CHANNEL EFFECTS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Nodes: " << numNodes << std::endl;
    std::cout << "Simulation Time: " << simTime << "s" << std::endl;
    std::cout << "Many-to-One Routing: " << (manyToOne ? "Enabled" : "Disabled") << std::endl;
    std::cout << "Gaussian Noise: " << (enableNoise ? "Enabled" : "Disabled") << std::endl;
    std::cout << "Rayleigh Fading: " << (enableFading ? "Enabled" : "Disabled") << std::endl;
    std::cout << "SNR Threshold: " << snrThreshold << " dB" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Create nodes
    NodeContainer nodes;
    nodes.Create(numNodes);

    // Configure LR-WPAN devices
    LrWpanHelper lrWpanHelper;
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);
    lrWpanHelper.SetExtendedAddresses(lrwpanDevices);

    // Configure channel
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    for (uint32_t i = 0; i < lrwpanDevices.GetN(); i++)
    {
        Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
        dev->SetChannel(channel);
    }

    // Configure mobility (Grid topology)
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(80.0),
                                  "DeltaY", DoubleValue(60.0),
                                  "GridWidth", UintegerValue(3),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.Install(nodes);

    // Install Zigbee stack
    ZigbeeHelper zigbeeHelper;
    g_zigbeeStacks = zigbeeHelper.Install(lrwpanDevices);

    // Configure callbacks for all stacks
    for (uint32_t i = 0; i < g_zigbeeStacks.GetN(); i++)
    {
        Ptr<ZigbeeStack> zstack = g_zigbeeStacks.Get(i)->GetObject<ZigbeeStack>();

        // Assign streams for reproducibility
        zstack->GetNwk()->AssignStreams(i * 10);

        // APS callbacks
        zstack->GetAps()->SetApsdeDataIndicationCallback(
            MakeBoundCallback(&ApsDataIndication, zstack));
        zstack->GetAps()->SetApsdeDataConfirmCallback(
            MakeBoundCallback(&ApsDataConfirm, zstack));

        // NWK callbacks
        zstack->GetNwk()->SetNlmeNetworkFormationConfirmCallback(
            MakeBoundCallback(&NwkNetworkFormationConfirm, zstack));
        zstack->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
            MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack));
        zstack->GetNwk()->SetNlmeJoinConfirmCallback(
            MakeBoundCallback(&NwkJoinConfirm, zstack));
        zstack->GetNwk()->SetNlmeRouteDiscoveryConfirmCallback(
            MakeBoundCallback(&NwkRouteDiscoveryConfirm, zstack));
    }

    // Get coordinator stack
    Ptr<ZigbeeStack> coordinator = g_zigbeeStacks.Get(0)->GetObject<ZigbeeStack>();

    // ===== NETWORK FORMATION =====
    NlmeNetworkFormationRequestParams netFormParams;
    netFormParams.m_scanChannelList.channelPageCount = 1;
    netFormParams.m_scanChannelList.channelsField[0] = ALL_CHANNELS;
    netFormParams.m_scanDuration = 0;
    netFormParams.m_superFrameOrder = 15;
    netFormParams.m_beaconOrder = 15;

    Simulator::ScheduleWithContext(coordinator->GetNode()->GetId(),
                                   Seconds(1.0),
                                   &ZigbeeNwk::NlmeNetworkFormationRequest,
                                   coordinator->GetNwk(),
                                   netFormParams);

    // ===== DEVICE JOINING (Staggered) =====
    NlmeNetworkDiscoveryRequestParams netDiscParams;
    netDiscParams.m_scanChannelList.channelPageCount = 1;
    netDiscParams.m_scanChannelList.channelsField[0] = 0x00007800; // Channels 11-14
    netDiscParams.m_scanDuration = 2;

    double joinTime = 3.0;
    for (uint32_t i = 1; i < numNodes; i++)
    {
        Ptr<ZigbeeStack> stack = g_zigbeeStacks.Get(i)->GetObject<ZigbeeStack>();
        Simulator::ScheduleWithContext(stack->GetNode()->GetId(),
                                       Seconds(joinTime),
                                       &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                       stack->GetNwk(),
                                       netDiscParams);
        joinTime += 2.0;
    }

    // ===== GROUP CONFIGURATION =====
    double groupTime = joinTime + 2.0;

    // Add routers to groups (if enough nodes exist)
    if (numNodes >= 5)
    {
        Ptr<ZigbeeStack> router4 = g_zigbeeStacks.Get(4)->GetObject<ZigbeeStack>();
        Simulator::Schedule(Seconds(groupTime), &AddToGroup,
                           router4, GROUP_LIVING_ROOM, 1, "Living Room");
        Simulator::Schedule(Seconds(groupTime + 0.1), &AddToGroup,
                           router4, GROUP_ALL_LIGHTS, 1, "All Lights");
    }

    if (numNodes >= 6)
    {
        Ptr<ZigbeeStack> router5 = g_zigbeeStacks.Get(5)->GetObject<ZigbeeStack>();
        Simulator::Schedule(Seconds(groupTime + 0.2), &AddToGroup,
                           router5, GROUP_BEDROOM, 1, "Bedroom");
        Simulator::Schedule(Seconds(groupTime + 0.3), &AddToGroup,
                           router5, GROUP_ALL_LIGHTS, 1, "All Lights");
    }

    // ===== ROUTING =====
    double routingTime = groupTime + 2.0;

    if (manyToOne)
    {
        // Many-to-One routing
        NlmeRouteDiscoveryRequestParams routeDiscParams;
        routeDiscParams.m_dstAddrMode = NO_ADDRESS;

        Simulator::Schedule(Seconds(routingTime),
                           &ZigbeeNwk::NlmeRouteDiscoveryRequest,
                           coordinator->GetNwk(),
                           routeDiscParams);
    }
    else
    {
        // Mesh routing from coordinator to farthest node
        if (numNodes >= 4)
        {
            Ptr<ZigbeeStack> farthestNode = g_zigbeeStacks.Get(numNodes - 1)->GetObject<ZigbeeStack>();
            NlmeRouteDiscoveryRequestParams routeDiscParams;
            routeDiscParams.m_dstAddr = farthestNode->GetNwk()->GetNetworkAddress();
            routeDiscParams.m_dstAddrMode = UCST_BCST;
            routeDiscParams.m_radius = 0;

            Simulator::Schedule(Seconds(routingTime),
                               &ZigbeeNwk::NlmeRouteDiscoveryRequest,
                               coordinator->GetNwk(),
                               routeDiscParams);
        }
    }

    // ===== DATA TRANSMISSION =====
    double dataTime = routingTime + 5.0;

    // Temperature sensor reports (periodic) - from last router to coordinator
    if (numNodes >= 4)
    {
        Ptr<ZigbeeStack> sensorNode = g_zigbeeStacks.Get(numNodes - 1)->GetObject<ZigbeeStack>();
        uint32_t numReports = std::min(10u, (uint32_t)((simTime - dataTime) / 15.0));

        for (uint32_t i = 0; i < numReports; i++)
        {
            Simulator::Schedule(Seconds(dataTime + i * 15.0),
                               &SendTemperatureReading, sensorNode, coordinator);
        }
    }

    // Group commands (light control)
    if (numNodes >= 5)
    {
        Simulator::Schedule(Seconds(dataTime + 5.0),
                           &SendGroupCommand, coordinator, GROUP_LIVING_ROOM,
                           "Turn ON Living Room", 0x01);

        Simulator::Schedule(Seconds(dataTime + 20.0),
                           &SendGroupCommand, coordinator, GROUP_ALL_LIGHTS,
                           "Turn OFF All Lights", 0x00);

        Simulator::Schedule(Seconds(dataTime + 35.0),
                           &SendGroupCommand, coordinator, GROUP_ALL_LIGHTS,
                           "Turn ON All Lights", 0x01);
    }

    // Additional unicast traffic for larger networks
    if (numNodes >= 8)
    {
        // Multiple sensors reporting
        for (uint32_t i = 3; i < std::min(numNodes, 8u); i++)
        {
            Ptr<ZigbeeStack> node = g_zigbeeStacks.Get(i)->GetObject<ZigbeeStack>();
            uint32_t numReports = 5;

            for (uint32_t j = 0; j < numReports; j++)
            {
                Simulator::Schedule(Seconds(dataTime + 10.0 + i * 2.0 + j * 20.0),
                                   &SendTemperatureReading, node, coordinator);
            }
        }
    }

    // ===== PERFORMANCE MONITORING =====
    // Print performance metrics at the end
    Simulator::Schedule(Seconds(simTime - 1.0), &PrintPerformanceMetrics, numNodes, simTime);

    // Export to CSV if requested
    if (exportCSV)
    {
        Simulator::Schedule(Seconds(simTime - 0.5), &ExportResultsToCSV,
                           numNodes, simTime, csvFilename);
    }

    // ===== RUN SIMULATION =====
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
