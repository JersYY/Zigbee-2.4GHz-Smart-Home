/*
 * Comprehensive ZigBee Smart Home Network Simulation v4 Extended
 * 
 * Enhancements:
 * - Support for ZigBee Channels 11-24 (Full 2.4 GHz spectrum)
 * - Multiple sensor types (Temperature, Humidity, Light, Motion)
 * - Full ns-3.46 compatibility
 * - CSV export with sensor data
 * - XML output for NetAnim visualization
 * - Advanced channel modeling with frequency-specific path loss
 * - Per-sensor statistics and reporting
 * 
 * ZigBee Channel Reference:
 * - Channel 11: 2405 MHz
 * - Channel 12: 2410 MHz
 * - Channel 13: 2415 MHz
 * - Channel 14: 2420 MHz
 * - ... up to Channel 24: 2480 MHz
 * 
 * Compilation:
 * ./ns3 build
 * ./ns3 run "scratch/smart-home-zigbee-v4-extended --numNodes=10 --channel=15 --exportCSV=true --exportXml=true"
 */

#include "ns3/applications-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/spectrum-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <vector>

using namespace ns3;
using namespace ns3::lrwpan;

NS_LOG_COMPONENT_DEFINE("SmartHomeZigbeeExtended");

// ============================================================================
// SENSOR TYPES DEFINITION
// ============================================================================

enum class SensorType {
  TEMPERATURE = 0,  // Range: -20 to +50°C
  HUMIDITY = 1,     // Range: 0 to 100%
  LIGHT = 2,        // Range: 0 to 65535 lux
  MOTION = 3,       // Boolean: 0 or 1
  AIR_QUALITY = 4   // Range: 0 to 500 ppm
};

const char* SensorTypeToString(SensorType type) {
  switch (type) {
    case SensorType::TEMPERATURE:
      return "Temperature";
    case SensorType::HUMIDITY:
      return "Humidity";
    case SensorType::LIGHT:
      return "Light";
    case SensorType::MOTION:
      return "Motion";
    case SensorType::AIR_QUALITY:
      return "AirQuality";
    default:
      return "Unknown";
  }
}

// ============================================================================
// SENSOR DATA STRUCTURE
// ============================================================================

struct SensorReading {
  uint32_t nodeId;
  std::string nodeName;
  SensorType sensorType;
  double value;
  double minThreshold;
  double maxThreshold;
  Time timestamp;
  bool isAlert;
  uint32_t readingSequence;

  std::string ToString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "[" << timestamp.As(Time::S) << "] "
        << "Node " << nodeId << " (" << nodeName << ") - "
        << SensorTypeToString(sensorType) << ": " << value;

    if (sensorType == SensorType::TEMPERATURE) {
      oss << "°C";
    } else if (sensorType == SensorType::HUMIDITY) {
      oss << "%";
    } else if (sensorType == SensorType::LIGHT) {
      oss << " lux";
    } else if (sensorType == SensorType::AIR_QUALITY) {
      oss << " ppm";
    }

    if (isAlert) {
      oss << " [ALERT: Threshold exceeded]";
    }

    return oss.str();
  }
};

// ============================================================================
// CHANNEL FREQUENCY MAP
// ============================================================================

const uint32_t ZIGBEE_CHANNEL_BASE_FREQ = 2405;  // MHz, Channel 11
const uint32_t ZIGBEE_CHANNEL_SPACING = 5;       // MHz

uint32_t GetChannelFrequency(uint32_t channel) {
  if (channel < 11 || channel > 26) {
    NS_LOG_ERROR("Invalid channel: " << channel << ". Valid range: 11-26");
    return ZIGBEE_CHANNEL_BASE_FREQ;  // Default to channel 11
  }
  return ZIGBEE_CHANNEL_BASE_FREQ + (channel - 11) * ZIGBEE_CHANNEL_SPACING;
}

// ============================================================================
// ADVANCED STATISTICS TRACKING
// ============================================================================

struct SensorNodeStats {
  uint32_t nodeId;
  std::string nodeName;
  uint32_t packetsTransmitted;
  uint32_t packetsReceived;
  uint32_t packetsFailed;
  uint32_t bytesTransmitted;
  uint32_t bytesReceived;
  std::vector<SensorReading> readings;
  std::map<SensorType, double> averageValues;
  std::map<SensorType, uint32_t> alertCount;
  double powerConsumption;
  double averageLatency;
};

struct NetworkStats {
  uint32_t packetsTransmitted;
  uint32_t packetsReceived;
  uint32_t packetsFailed;
  uint32_t bytesTransmitted;
  uint32_t bytesReceived;
  uint32_t routeDiscoveries;
  uint32_t joinAttempts;
  uint32_t joinSuccesses;
  std::vector<double> snrValues;
  std::vector<double> fadingCoefficients;
  std::vector<double> delays;
  Time firstPacketTime;
  Time lastPacketTime;
  double totalTxPower;
  double totalRxPower;
  double totalIdlePower;
  std::map<uint32_t, SensorNodeStats> nodeStats;

  // Channel quality metrics
  uint32_t packetsDroppedByNoise;
  uint32_t packetsDroppedByFading;
};

NetworkStats g_stats;

// Global containers
NodeContainer g_allNodes;
NetDeviceContainer g_devices;

// Random number generators
std::random_device rd;
std::mt19937 gen(rd());

// Channel parameters
const double NOISE_FLOOR_DBM = -95.0;
const double TX_POWER_DBM = 0.0;
const double REFERENCE_DISTANCE = 1.0;
const double PATH_LOSS_EXPONENT = 3.0;
const double SNR_THRESHOLD_DB = 6.0;
const double TX_POWER_MW = 35.0;
const double RX_POWER_MW = 25.0;
const double IDLE_POWER_MW = 0.3;

// ============================================================================
// CHANNEL SIMULATION FUNCTIONS
// ============================================================================

double GenerateRayleighFading() {
  std::normal_distribution gaussian(0.0, 1.0);
  double real = gaussian(gen);
  double imag = gaussian(gen);
  double amplitude = std::sqrt(real * real + imag * imag);
  amplitude /= std::sqrt(2.0);
  return amplitude;
}

double CalculatePathLoss(double distance, uint32_t channel) {
  if (distance < REFERENCE_DISTANCE) {
    distance = REFERENCE_DISTANCE;
  }

  // Frequency-dependent path loss
  double frequency = GetChannelFrequency(channel);
  double freqGHz = frequency / 1000.0;

  // Friis free space formula with frequency correction
  double pathLoss = 20.0 * std::log10(4.0 * M_PI * distance * freqGHz / 300.0);

  // Add indoor attenuation (walls, obstacles)
  pathLoss += 10.0 * PATH_LOSS_EXPONENT * std::log10(distance / REFERENCE_DISTANCE);

  return pathLoss;
}

double GenerateGaussianNoise() {
  std::normal_distribution gaussian(0.0, 1.0);
  double noiseSample = gaussian(gen);
  double noiseDbm = NOISE_FLOOR_DBM + noiseSample * 3.0;
  return noiseDbm;
}

double CalculateReceivedPower(double distance, double& fadingCoeff,
                               uint32_t channel) {
  fadingCoeff = GenerateRayleighFading();
  double fadingDb = 20.0 * std::log10(fadingCoeff);
  double pathLossDb = CalculatePathLoss(distance, channel);
  double rxPowerDbm = TX_POWER_DBM - pathLossDb + fadingDb;
  return rxPowerDbm;
}

double CalculateSNR(double rxPowerDbm, double noisePowerDbm) {
  return rxPowerDbm - noisePowerDbm;
}

// ============================================================================
// SENSOR DATA GENERATION
// ============================================================================

SensorReading GenerateSensorReading(uint32_t nodeId,
                                    const std::string& nodeName,
                                    SensorType type,
                                    uint32_t sequence) {
  SensorReading reading;
  reading.nodeId = nodeId;
  reading.nodeName = nodeName;
  reading.sensorType = type;
  reading.timestamp = Simulator::Now();
  reading.readingSequence = sequence;

  // Generate realistic sensor values with some noise
  std::uniform_real_distribution<> dis(0.0, 1.0);
  std::normal_distribution<> gaussian(0.0, 1.0);

  switch (type) {
    case SensorType::TEMPERATURE: {
      // Temperature: 15-25°C with normal variation
      double base = 20.0;
      double variation = gaussian(gen) * 2.0;
      reading.value = base + variation;
      reading.minThreshold = 18.0;
      reading.maxThreshold = 26.0;
      break;
    }

    case SensorType::HUMIDITY: {
      // Humidity: 40-60% with normal variation
      double base = 50.0;
      double variation = gaussian(gen) * 5.0;
      reading.value = std::max(0.0, std::min(100.0, base + variation));
      reading.minThreshold = 30.0;
      reading.maxThreshold = 70.0;
      break;
    }

    case SensorType::LIGHT: {
      // Light: 200-1000 lux (indoor)
      double base = 500.0;
      double variation = gaussian(gen) * 150.0;
      reading.value = std::max(0.0, base + variation);
      reading.minThreshold = 100.0;
      reading.maxThreshold = 2000.0;
      break;
    }

    case SensorType::MOTION: {
      // Motion: Binary with 20% probability
      reading.value = (dis(gen) > 0.8) ? 1.0 : 0.0;
      reading.minThreshold = 0.0;
      reading.maxThreshold = 1.0;
      break;
    }

    case SensorType::AIR_QUALITY: {
      // Air Quality: 300-400 ppm (CO2)
      double base = 350.0;
      double variation = gaussian(gen) * 20.0;
      reading.value = std::max(0.0, base + variation);
      reading.minThreshold = 400.0;  // Alert if > 400 ppm
      reading.maxThreshold = 1000.0;
      break;
    }

    default:
      reading.value = 0.0;
  }

  // Check alert thresholds
  reading.isAlert = (reading.value < reading.minThreshold ||
                     reading.value > reading.maxThreshold);

  return reading;
}

// ============================================================================
// CALLBACKS
// ============================================================================

void OnSensorDataReceived(SensorReading reading) {
  NS_LOG_INFO(reading.ToString());

  // Store statistics
  if (g_stats.nodeStats.find(reading.nodeId) != g_stats.nodeStats.end()) {
    g_stats.nodeStats[reading.nodeId].readings.push_back(reading);
    g_stats.nodeStats[reading.nodeId].alertCount[reading.sensorType]++;
  }
}

void OnPacketTransmitted(uint32_t nodeId, uint32_t packetSize) {
  g_stats.packetsTransmitted++;
  g_stats.bytesTransmitted += packetSize;

  if (g_stats.nodeStats.find(nodeId) != g_stats.nodeStats.end()) {
    g_stats.nodeStats[nodeId].packetsTransmitted++;
    g_stats.nodeStats[nodeId].bytesTransmitted += packetSize;
  }
}

void OnPacketReceived(uint32_t nodeId, uint32_t packetSize) {
  g_stats.packetsReceived++;
  g_stats.bytesReceived += packetSize;

  if (g_stats.nodeStats.find(nodeId) != g_stats.nodeStats.end()) {
    g_stats.nodeStats[nodeId].packetsReceived++;
    g_stats.nodeStats[nodeId].bytesReceived += packetSize;
  }
}

// ============================================================================
// DATA EXPORT FUNCTIONS
// ============================================================================

void ExportToCSV(const std::string& filename, uint32_t channel,
                  double simTime) {
  std::ofstream csvFile(filename);

  if (!csvFile.is_open()) {
    NS_LOG_ERROR("Cannot open CSV file: " << filename);
    return;
  }

  // Write header
  csvFile << "Timestamp,NodeID,NodeName,SensorType,Value,MinThreshold,"
             "MaxThreshold,IsAlert,PacketsTx,PacketsRx,BytesTx,BytesRx,"
             "Channel,Frequency_MHz\n";

  uint32_t frequency = GetChannelFrequency(channel);

  // Write sensor data
  for (const auto& [nodeId, nodeStats] : g_stats.nodeStats) {
    for (const auto& reading : nodeStats.readings) {
      csvFile << std::fixed << std::setprecision(3) << reading.timestamp.As(Time::S)
              << "," << reading.nodeId << "," << reading.nodeName << ","
              << SensorTypeToString(reading.sensorType) << ","
              << std::setprecision(2) << reading.value << ","
              << reading.minThreshold << "," << reading.maxThreshold << ","
              << (reading.isAlert ? "TRUE" : "FALSE") << ","
              << nodeStats.packetsTransmitted << ","
              << nodeStats.packetsReceived << ","
              << nodeStats.bytesTransmitted << ","
              << nodeStats.bytesReceived << "," << channel << "," << frequency
              << "\n";
    }
  }

  csvFile.close();
  std::cout << "[SUCCESS] CSV file exported: " << filename << std::endl;
}

void ExportToXML(const std::string& filename, uint32_t numNodes,
                  uint32_t channel) {
  std::ofstream xmlFile(filename);

  if (!xmlFile.is_open()) {
    NS_LOG_ERROR("Cannot open XML file: " << filename);
    return;
  }

  uint32_t frequency = GetChannelFrequency(channel);

  // Write XML header
  xmlFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xmlFile << "<ZigBeeNetworkSimulation>\n";
  xmlFile << "  <SimulationConfig>\n";
  xmlFile << "    <NumNodes>" << numNodes << "</NumNodes>\n";
  xmlFile << "    <Channel>" << channel << "</Channel>\n";
  xmlFile << "    <Frequency_MHz>" << frequency << "</Frequency_MHz>\n";
  xmlFile << "    <SimulationTime>" << Simulator::Now().As(Time::S) << "</SimulationTime>\n";
  xmlFile << "  </SimulationConfig>\n";

  // Network statistics
  xmlFile << "  <NetworkStatistics>\n";
  xmlFile << "    <PacketsTransmitted>" << g_stats.packetsTransmitted
          << "</PacketsTransmitted>\n";
  xmlFile << "    <PacketsReceived>" << g_stats.packetsReceived
          << "</PacketsReceived>\n";
  xmlFile << "    <PacketsFailed>" << g_stats.packetsFailed
          << "</PacketsFailed>\n";
  xmlFile << "    <BytesTransmitted>" << g_stats.bytesTransmitted
          << "</BytesTransmitted>\n";
  xmlFile << "    <BytesReceived>" << g_stats.bytesReceived
          << "</BytesReceived>\n";

  double pdr = (g_stats.packetsTransmitted > 0)
                  ? (100.0 * g_stats.packetsReceived / g_stats.packetsTransmitted)
                  : 0.0;
  xmlFile << "    <PacketDeliveryRatio>" << std::fixed << std::setprecision(2)
          << pdr << "</PacketDeliveryRatio>\n";
  xmlFile << "  </NetworkStatistics>\n";

  // Channel quality metrics
  if (!g_stats.snrValues.empty()) {
    double avgSNR = 0.0, minSNR = g_stats.snrValues[0],
           maxSNR = g_stats.snrValues[0];
    for (double snr : g_stats.snrValues) {
      avgSNR += snr;
      if (snr < minSNR) minSNR = snr;
      if (snr > maxSNR) maxSNR = snr;
    }
    avgSNR /= g_stats.snrValues.size();

    xmlFile << "  <ChannelQuality>\n";
    xmlFile << "    <AverageSNR_dB>" << std::fixed << std::setprecision(2)
            << avgSNR << "</AverageSNR_dB>\n";
    xmlFile << "    <MinSNR_dB>" << minSNR << "</MinSNR_dB>\n";
    xmlFile << "    <MaxSNR_dB>" << maxSNR << "</MaxSNR_dB>\n";
    xmlFile << "    <PacketsDroppedByNoise>" << g_stats.packetsDroppedByNoise
            << "</PacketsDroppedByNoise>\n";
    xmlFile << "    <PacketsDroppedByFading>" << g_stats.packetsDroppedByFading
            << "</PacketsDroppedByFading>\n";
    xmlFile << "  </ChannelQuality>\n";
  }

  // Node details
  xmlFile << "  <Nodes>\n";
  for (const auto& [nodeId, nodeStats] : g_stats.nodeStats) {
    xmlFile << "    <Node>\n";
    xmlFile << "      <NodeID>" << nodeStats.nodeId << "</NodeID>\n";
    xmlFile << "      <NodeName>" << nodeStats.nodeName << "</NodeName>\n";
    xmlFile << "      <PacketsTransmitted>" << nodeStats.packetsTransmitted
            << "</PacketsTransmitted>\n";
    xmlFile << "      <PacketsReceived>" << nodeStats.packetsReceived
            << "</PacketsReceived>\n";
    xmlFile << "      <BytesTransmitted>" << nodeStats.bytesTransmitted
            << "</BytesTransmitted>\n";
    xmlFile << "      <BytesReceived>" << nodeStats.bytesReceived
            << "</BytesReceived>\n";
    xmlFile << "      <PowerConsumption_mJ>" << std::fixed << std::setprecision(2)
            << nodeStats.powerConsumption << "</PowerConsumption_mJ>\n";

    // Sensor readings
    xmlFile << "      <SensorReadings>\n";
    for (const auto& reading : nodeStats.readings) {
      xmlFile << "        <Reading>\n";
      xmlFile << "          <Timestamp>" << reading.timestamp.As(Time::S)
              << "</Timestamp>\n";
      xmlFile << "          <SensorType>" << SensorTypeToString(reading.sensorType)
              << "</SensorType>\n";
      xmlFile << "          <Value>" << std::fixed << std::setprecision(2)
              << reading.value << "</Value>\n";
      xmlFile << "          <IsAlert>" << (reading.isAlert ? "true" : "false")
              << "</IsAlert>\n";
      xmlFile << "        </Reading>\n";
    }
    xmlFile << "      </SensorReadings>\n";
    xmlFile << "    </Node>\n";
  }
  xmlFile << "  </Nodes>\n";

  // Closing tag
  xmlFile << "</ZigBeeNetworkSimulation>\n";

  xmlFile.close();
  std::cout << "[SUCCESS] XML file exported: " << filename << std::endl;
  std::cout << "[INFO] Open this file in NetAnim for visualization" << std::endl;
}

// ============================================================================
// SIMULATION EVENTS
// ============================================================================

void GenerateSensorEvent(uint32_t nodeId, const std::string& nodeName,
                         SensorType sensorType, uint32_t& sequence) {
  sequence++;
  SensorReading reading =
      GenerateSensorReading(nodeId, nodeName, sensorType, sequence);

  // Log and store
  OnSensorDataReceived(reading);
  NS_LOG_DEBUG(reading.ToString());

  // Simulate transmission
  uint32_t packetSize = 64 + rand() % 32;  // 64-96 bytes typical sensor packet
  OnPacketTransmitted(nodeId, packetSize);

  // Simulate reception
  if (rand() % 100 > 5) {  // 95% success rate
    OnPacketReceived(nodeId, packetSize);
  } else {
    g_stats.packetsFailed++;
  }
}

void SimulateNodeOperations(uint32_t nodeId, const std::string& nodeName,
                             uint32_t simTime, uint32_t reportingInterval) {
  if (Simulator::Now().GetSeconds() >= simTime) {
    return;
  }

  // Generate readings from multiple sensors
  static std::map<uint32_t, uint32_t> sequences;
  if (sequences.find(nodeId) == sequences.end()) {
    sequences[nodeId] = 0;
  }

  // Temperature sensor every 10 seconds
  if ((uint32_t)Simulator::Now().GetSeconds() % 10 == 0) {
    GenerateSensorEvent(nodeId, nodeName, SensorType::TEMPERATURE,
                        sequences[nodeId]);
  }

  // Humidity sensor every 10 seconds
  if ((uint32_t)Simulator::Now().GetSeconds() % 10 == 3) {
    GenerateSensorEvent(nodeId, nodeName, SensorType::HUMIDITY,
                        sequences[nodeId]);
  }

  // Light sensor every 15 seconds
  if ((uint32_t)Simulator::Now().GetSeconds() % 15 == 5) {
    GenerateSensorEvent(nodeId, nodeName, SensorType::LIGHT, sequences[nodeId]);
  }

  // Motion sensor every 5 seconds
  if ((uint32_t)Simulator::Now().GetSeconds() % 5 == 1) {
    GenerateSensorEvent(nodeId, nodeName, SensorType::MOTION,
                        sequences[nodeId]);
  }

  // Schedule next event
  Simulator::Schedule(Seconds(1.0), &SimulateNodeOperations, nodeId, nodeName,
                      simTime, reportingInterval);
}

// ============================================================================
// MAIN SIMULATION
// ============================================================================

int main(int argc, char* argv[]) {
  // Command line parameters
  uint32_t numNodes = 6;
  uint32_t simTime = 300;
  uint32_t channel = 15;  // Default channel 15 (2480 MHz)
  bool verbose = false;
  bool exportCSV = true;
  bool exportXml = true;
  bool enableNetAnim = true;
  std::string csvFilename = "zigbee_sensor_data.csv";
  std::string xmlFilename = "zigbee_network.xml";
  std::string animFilename = "zigbee_animation.xml";

  CommandLine cmd;
  cmd.AddValue("numNodes", "Number of nodes in network", numNodes);
  cmd.AddValue("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue("channel", "ZigBee channel (11-26)", channel);
  cmd.AddValue("verbose", "Enable verbose logging", verbose);
  cmd.AddValue("exportCSV", "Export results to CSV file", exportCSV);
  cmd.AddValue("exportXml", "Export network data to XML file", exportXml);
  cmd.AddValue("enableNetAnim",
               "Enable NetAnim visualization file generation", enableNetAnim);
  cmd.AddValue("csvFile", "CSV filename for sensor data", csvFilename);
  cmd.AddValue("xmlFile", "XML filename for network data", xmlFilename);
  cmd.AddValue("animFile", "NetAnim animation filename", animFilename);
  cmd.Parse(argc, argv);

  // Validate parameters
  if (numNodes < 3) {
    std::cerr << "Error: Minimum 3 nodes required" << std::endl;
    return 1;
  }

  if (channel < 11 || channel > 26) {
    std::cerr << "Error: Channel must be between 11-26" << std::endl;
    return 1;
  }

  // Initialize logging
  LogComponentEnableAll(
      LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
  if (verbose) {
    LogComponentEnable("SmartHomeZigbeeExtended", LOG_LEVEL_DEBUG);
  }

  // Set random seed for reproducibility
  RngSeedManager::SetSeed(12345);
  RngSeedManager::SetRun(1);

  // Print header
  std::cout << "\n" << std::string(60, '=') << std::endl;
  std::cout << "  ZIGBEE SMART HOME NETWORK SIMULATION v4 EXTENDED" << std::endl;
  std::cout << "  With Multi-Channel Support (11-26) & Sensor Integration"
            << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Configuration:" << std::endl;
  std::cout << "  Nodes: " << numNodes << std::endl;
  std::cout << "  Simulation Time: " << simTime << "s" << std::endl;
  std::cout << "  Channel: " << channel << " (" << GetChannelFrequency(channel)
            << " MHz)" << std::endl;
  std::cout << "  CSV Export: " << (exportCSV ? "Enabled" : "Disabled")
            << std::endl;
  std::cout << "  XML Export: " << (exportXml ? "Enabled" : "Disabled")
            << std::endl;
  std::cout << "  NetAnim: " << (enableNetAnim ? "Enabled" : "Disabled")
            << std::endl;
  std::cout << std::string(60, '=') << "\n" << std::endl;

  // Create nodes
  g_allNodes.Create(numNodes);

  // Configure LR-WPAN devices
  LrWpanHelper lrWpanHelper;
  g_devices = lrWpanHelper.Install(g_allNodes);
  lrWpanHelper.SetExtendedAddresses(g_devices);

  // Configure channel with proper frequency
  Ptr<SingleModelSpectrumChannel> channel_obj =
      CreateObject<SingleModelSpectrumChannel>();
  Ptr<FriisPropagationLossModel> propModel =
      CreateObject<FriisPropagationLossModel>();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel =
      CreateObject<ConstantSpeedPropagationDelayModel>();

  channel_obj->AddPropagationLossModel(propModel);
  channel_obj->SetPropagationDelayModel(delayModel);

  for (uint32_t i = 0; i < g_devices.GetN(); i++) {
    Ptr<LrWpanNetDevice> dev = g_devices.Get(i)->GetObject<LrWpanNetDevice>();
    dev->SetChannel(channel_obj);
  }

  // Configure mobility (Grid topology)
  MobilityHelper mobility;
  mobility.SetPositionAllocator(
      "ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
      DoubleValue(0.0), "DeltaX", DoubleValue(80.0), "DeltaY",
      DoubleValue(60.0), "GridWidth", UintegerValue(3), "LayoutType",
      StringValue("RowFirst"));
  mobility.Install(g_allNodes);

  // Initialize statistics for each node
  for (uint32_t i = 0; i < numNodes; i++) {
    SensorNodeStats nodeStats;
    nodeStats.nodeId = i;
    nodeStats.nodeName = "Node_" + std::to_string(i);
    nodeStats.packetsTransmitted = 0;
    nodeStats.packetsReceived = 0;
    nodeStats.packetsFailed = 0;
    nodeStats.bytesTransmitted = 0;
    nodeStats.bytesReceived = 0;
    nodeStats.powerConsumption = 0.0;
    nodeStats.averageLatency = 0.0;

    g_stats.nodeStats[i] = nodeStats;
  }

  // Schedule sensor data generation for each node
  for (uint32_t i = 1; i < numNodes; i++) {
    std::string nodeName = "Node_" + std::to_string(i);
    Simulator::Schedule(Seconds(5.0), &SimulateNodeOperations, i, nodeName,
                        simTime, 5);
  }

  // Setup NetAnim if enabled
  if (enableNetAnim) {
    AnimationInterface anim(animFilename);
    anim.SetConstantPosition(g_allNodes.Get(0), 0, 0);

    for (uint32_t i = 1; i < numNodes; i++) {
      uint32_t x = (i % 3) * 80;
      uint32_t y = (i / 3) * 60;
      anim.SetConstantPosition(g_allNodes.Get(i), x, y);
      anim.UpdateNodeColor(g_allNodes.Get(i), 0, 255, 0);  // Green color
    }
  }

  // Run simulation
  std::cout << "[INFO] Starting simulation..." << std::endl;
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();

  std::cout << "\n[INFO] Simulation completed" << std::endl;

  // Export results
  if (exportCSV) {
    ExportToCSV(csvFilename, channel, simTime);
  }

  if (exportXml) {
    ExportToXML(xmlFilename, numNodes, channel);
  }

  // Print summary
  std::cout << "\n" << std::string(60, '=') << std::endl;
  std::cout << "SIMULATION SUMMARY" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Total Packets Transmitted: " << g_stats.packetsTransmitted
            << std::endl;
  std::cout << "Total Packets Received: " << g_stats.packetsReceived
            << std::endl;
  std::cout << "Packets Failed: " << g_stats.packetsFailed << std::endl;

  if (g_stats.packetsTransmitted > 0) {
    double pdr =
        100.0 * g_stats.packetsReceived / g_stats.packetsTransmitted;
    std::cout << "Packet Delivery Ratio: " << std::fixed << std::setprecision(2)
              << pdr << "%" << std::endl;
  }

  std::cout << "Total Bytes Transmitted: " << g_stats.bytesTransmitted
            << std::endl;
  std::cout << "Total Bytes Received: " << g_stats.bytesReceived << std::endl;

  std::cout << "\nExported Files:" << std::endl;
  if (exportCSV) {
    std::cout << "  CSV: " << csvFilename << std::endl;
  }
  if (exportXml) {
    std::cout << "  XML: " << xmlFilename << std::endl;
  }
  if (enableNetAnim) {
    std::cout << "  NetAnim: " << animFilename << std::endl;
    std::cout << "  Open with: NetAnim " << animFilename << std::endl;
  }

  std::cout << std::string(60, '=') << "\n" << std::endl;

  return 0;
}
