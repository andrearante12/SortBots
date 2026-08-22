/*
 * 802.11s mesh bridge for SortBots.
 *
 * Creates N mesh nodes (dot11s + HWMP), one per robot, and attaches a
 * TapBridge in UseLocal mode to each node.  The tap devices must already
 * exist inside the target network namespaces before this program starts
 * (created by scripts/mesh_netns_setup.sh).
 *
 * Node i bridges tap-robot_<i> and gets IP 10.66.0.<i+1>/24 (assigned by
 * the OS on the tap inside the netns; ns-3 itself does not assign IPs here
 * because UseLocal inherits them from the host tap).
 *
 * Build:
 *   ln -sf $REPO/network/ns3_mesh_bridge.cc $NS3_HOME/scratch/ns3_mesh_bridge.cc
 *   cd $NS3_HOME && ./ns3 build ns3_mesh_bridge
 *
 * Run (via scripts/run_mesh.sh, not directly):
 *   ./ns3 run "ns3_mesh_bridge --robots=2 --config=/path/to/mesh_config.yaml"
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SortBotsMeshBridge");

// ---------------------------------------------------------------------------
// Minimal YAML-like config parser — reads only the keys we care about so we
// don't need a YAML library dependency inside ns-3 scratch.
// ---------------------------------------------------------------------------
struct RobotEntry
{
    std::string id;
    std::string tap;
    double x{0.0};
    double y{0.0};
};

struct MeshConfig
{
    double pathLossExp{3.0};
    double lossRefDb{46.6777};
    double txPowerDbm{16.0};
    std::string hwmpRoot{"ff:ff:ff:ff:ff:ff"};
    std::vector<RobotEntry> robots;
};

static std::string
Trim(const std::string& s)
{
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos)
        return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static MeshConfig
LoadConfig(const std::string& path, int nRobots)
{
    MeshConfig cfg;
    if (path.empty())
    {
        // Build defaults: tap-robot_0, tap-robot_1, … at 30 m spacing
        for (int i = 0; i < nRobots; ++i)
        {
            RobotEntry e;
            e.id = "robot_" + std::to_string(i);
            e.tap = "tap-robot_" + std::to_string(i);
            e.x = static_cast<double>(i) * 30.0;
            e.y = 0.0;
            cfg.robots.push_back(e);
        }
        return cfg;
    }

    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "[ns3_mesh_bridge] cannot open config: " << path << "\n";
        std::exit(1);
    }

    // Simple line-by-line parser: indentation tracked by leading spaces/dashes
    bool inMesh = false;
    bool inRobots = false;
    bool inRobotEntry = false;
    RobotEntry cur;

    std::string line;
    while (std::getline(f, line))
    {
        // Strip comments
        auto hash = line.find('#');
        if (hash != std::string::npos)
            line = line.substr(0, hash);
        if (Trim(line).empty())
            continue;

        // Detect section headers (no leading spaces)
        if (line[0] != ' ' && line[0] != '-')
        {
            if (inRobotEntry && !cur.id.empty())
            {
                cfg.robots.push_back(cur);
                cur = RobotEntry{};
                inRobotEntry = false;
            }
            inMesh = (Trim(line) == "mesh:");
            inRobots = (Trim(line) == "robots:");
            continue;
        }

        std::string trimmed = Trim(line);

        if (inMesh)
        {
            auto colon = trimmed.find(':');
            if (colon == std::string::npos)
                continue;
            std::string key = Trim(trimmed.substr(0, colon));
            std::string val = Trim(trimmed.substr(colon + 1));
            if (key == "path_loss_exp")
                cfg.pathLossExp = std::stod(val);
            else if (key == "loss_ref_db")
                cfg.lossRefDb = std::stod(val);
            else if (key == "tx_power_dbm")
                cfg.txPowerDbm = std::stod(val);
            else if (key == "hwmp_root")
                cfg.hwmpRoot = val.substr(
                    val[0] == '"' ? 1 : 0,
                    val.size() - (val[0] == '"' ? 2 : 0));
            continue;
        }

        if (inRobots)
        {
            // New robot entry starts with "  - id:"
            if (trimmed[0] == '-')
            {
                if (inRobotEntry && !cur.id.empty())
                    cfg.robots.push_back(cur);
                cur = RobotEntry{};
                inRobotEntry = true;
                trimmed = Trim(trimmed.substr(1));
            }
            if (!inRobotEntry)
                continue;
            auto colon = trimmed.find(':');
            if (colon == std::string::npos)
                continue;
            std::string key = Trim(trimmed.substr(0, colon));
            std::string val = Trim(trimmed.substr(colon + 1));
            if (key == "id")
                cur.id = val;
            else if (key == "tap")
                cur.tap = val;
            else if (key == "x")
                cur.x = std::stod(val);
            else if (key == "y")
                cur.y = std::stod(val);
            continue;
        }
    }
    if (inRobotEntry && !cur.id.empty())
        cfg.robots.push_back(cur);

    // Clamp to nRobots
    if (static_cast<int>(cfg.robots.size()) > nRobots)
        cfg.robots.resize(nRobots);

    // Pad with defaults if config has fewer entries than --robots
    for (int i = static_cast<int>(cfg.robots.size()); i < nRobots; ++i)
    {
        RobotEntry e;
        e.id = "robot_" + std::to_string(i);
        e.tap = "tap-robot_" + std::to_string(i);
        e.x = static_cast<double>(i) * 30.0;
        e.y = 0.0;
        cfg.robots.push_back(e);
    }

    return cfg;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int
main(int argc, char* argv[])
{
    int nRobots = 2;
    std::string configPath = "";
    std::string pcapDir = "";
    double simTime = 86400.0; // 24 h — terminated by SIGTERM in practice

    CommandLine cmd(__FILE__);
    cmd.AddValue("robots", "Number of robots (mesh nodes)", nRobots);
    cmd.AddValue("config", "Path to network/mesh_config.yaml", configPath);
    cmd.AddValue("pcap-dir", "Directory for .pcap traces (empty = disabled)", pcapDir);
    cmd.AddValue("sim-time", "Simulation wall-clock duration in seconds", simTime);
    cmd.Parse(argc, argv);

    // Realtime mode — mandatory for bridging into a live Isaac Sim session
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    MeshConfig cfg = LoadConfig(configPath, nRobots);

    std::cout << "[ns3_mesh_bridge] starting with " << nRobots << " robots\n";
    std::cout << "[ns3_mesh_bridge] path_loss_exp=" << cfg.pathLossExp
              << " tx_power_dbm=" << cfg.txPowerDbm
              << " hwmp_root=" << cfg.hwmpRoot << "\n";

    // ── Nodes ────────────────────────────────────────────────────────────
    NodeContainer nodes;
    nodes.Create(nRobots);

    // ── PHY / Channel ────────────────────────────────────────────────────
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                   "Exponent",
                                   DoubleValue(cfg.pathLossExp),
                                   "ReferenceLoss",
                                   DoubleValue(cfg.lossRefDb),
                                   "ReferenceDistance",
                                   DoubleValue(1.0));

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.Set("TxPowerStart", DoubleValue(cfg.txPowerDbm));
    wifiPhy.Set("TxPowerEnd", DoubleValue(cfg.txPowerDbm));

    // ── Mesh (dot11s + HWMP) ─────────────────────────────────────────────
    MeshHelper mesh = MeshHelper::Default();
    Mac48Address rootMac(cfg.hwmpRoot.c_str());
    if (!rootMac.IsBroadcast())
        mesh.SetStackInstaller("ns3::Dot11sStack",
                               "Root",
                               Mac48AddressValue(rootMac));
    else
        mesh.SetStackInstaller("ns3::Dot11sStack");

    mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
    mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
    mesh.SetNumberOfInterfaces(1);

    NetDeviceContainer meshDevices = mesh.Install(wifiPhy, nodes);

    // ── Mobility ─────────────────────────────────────────────────────────
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (const auto& robot : cfg.robots)
        posAlloc->Add(Vector(robot.x, robot.y, 0.0));
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // ── Internet stack (required by TapBridge at runtime) ───────────────
    // TapBridge accesses the IPv4 interface both during setup (CreateTap)
    // and at runtime (frame dispatch).  We use a non-routable 172.31.99.x
    // range so ns-3's ARP stack never conflicts with the 10.66.0.x IPs on
    // the real kernel taps; UseLocal mode inherits MAC/IP from the kernel tap.
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4Addr;
    ipv4Addr.SetBase("172.31.99.0", "255.255.255.0", "0.0.0.1");
    ipv4Addr.Assign(meshDevices);

    // ── Emit mesh node MACs ───────────────────────────────────────────────
    // HWMP stores each node's address at Install() time (the ns-3 random MAC,
    // M0/M1).  Static ARP in the test netns must use M0/M1 — not the kernel
    // tap device MAC — so that HWMP path discovery succeeds when frames arrive
    // with dst=M1.  Taps run in promisc mode and accept frames regardless of
    // the dst MAC.  scripts/test_mesh_phase1.sh reads these lines.
    for (int i = 0; i < nRobots; ++i)
    {
        Mac48Address meshMac = Mac48Address::ConvertFrom(
            meshDevices.Get(static_cast<uint32_t>(i))->GetAddress());
        std::cout << "[ns3_mesh_bridge] node-mac "
                  << cfg.robots[static_cast<size_t>(i)].tap << " " << meshMac << "\n";
        std::cout.flush();
    }

    // ── TapBridge — one per robot ─────────────────────────────────────────
    // UseLocal: ns-3 inherits the tap's existing MAC and IP from the OS.
    // The tap device must already exist in the root namespace BEFORE ns-3
    // starts (created by scripts/mesh_netns_setup.sh).
    TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", StringValue("UseLocal"));

    for (int i = 0; i < nRobots; ++i)
    {
        const std::string& tapName = cfg.robots[static_cast<size_t>(i)].tap;
        tapBridge.SetAttribute("DeviceName", StringValue(tapName));
        tapBridge.Install(nodes.Get(i), meshDevices.Get(i));
        std::cout << "[ns3_mesh_bridge] TapBridge: attached " << tapName
                  << " to mesh node " << i << "\n";
        std::cout.flush();
    }

    // ── PCAP (optional) ──────────────────────────────────────────────────
    if (!pcapDir.empty())
    {
        wifiPhy.EnablePcapAll(pcapDir + "/mesh");
        std::cout << "[ns3_mesh_bridge] PCAP traces -> " << pcapDir << "/mesh-*.pcap\n";
    }

    // ── Run ──────────────────────────────────────────────────────────────
    Simulator::Stop(Seconds(simTime));
    std::cout << "[ns3_mesh_bridge] mesh running (sim-time=" << simTime << "s)\n";
    std::cout.flush();
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
