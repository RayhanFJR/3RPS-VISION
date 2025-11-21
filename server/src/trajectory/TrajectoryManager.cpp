//==================================================================
// FILE : server/src/trajectory/TrajectoryManager.cpp
//==================================================================
#include "TrajectoryManager.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
TrajectoryManager::TrajectoryManager() {
active_trajectory = nullptr;
// Initialize trajectory structures
trajectory1.trajectory_id = 1;
trajectory2.trajectory_id = 2;
trajectory3.trajectory_id = 3;
}
TrajectoryManager::~TrajectoryManager() {
// Cleanup vector memory
trajectory1.points.clear();
trajectory2.points.clear();
trajectory3.points.clear();
}
void TrajectoryManager::configureTrajectory1() {
trajectory1.total_points = 816;
trajectory1.gait_start_index = 101;
trajectory1.gait_end_index = 715;
trajectory1.graph_start_index = 101;
trajectory1.graph_end_index = 715;
}
void TrajectoryManager::configureTrajectory2() {
trajectory2.total_points = 1370;
trajectory2.gait_start_index = 165;
trajectory2.gait_end_index = 1177;
trajectory2.graph_start_index = 1;
trajectory2.graph_end_index = 1370;
}
void TrajectoryManager::configureTrajectory3() {
trajectory3.total_points = 1370;
trajectory3.gait_start_index = 165;
trajectory3.gait_end_index = 1177;
trajectory3.graph_start_index = 1;
trajectory3.graph_end_index = 1370;
}
bool TrajectoryManager::loadAllTrajectories() {
std::cout << "\n=== Loading All Trajectory Data ===" << std::endl;
bool success = true;

// Configure
configureTrajectory1();
configureTrajectory2();
configureTrajectory3();

// Load Trajectory 1
std::cout << "\n[Trajectory 1 - 816 points]" << std::endl;
success &= loadTrajectory(1, "../data");

// Load Trajectory 2
std::cout << "\n[Trajectory 2 - 1370 points]" << std::endl;
success &= loadTrajectory(2, "../data");

// Load Trajectory 3
std::cout << "\n[Trajectory 3 - 1370 points]" << std::endl;
success &= loadTrajectory(3, "../data");

if (success) {
    std::cout << "\n=== All trajectory data loaded successfully ===" << std::endl;
    switchTrajectory(1);  // Default: Trajectory 1
} else {
    std::cerr << "\n=== Error: Failed to load some trajectory data ===" << std::endl;
}

return success;
}
bool TrajectoryManager::loadTrajectory(int trajectory_id, const std::string& data_dir) {
TrajectoryData* traj = getTrajectoryData(trajectory_id);
if (!traj) return false;
// Build file paths
std::string traj_dir = data_dir + "/trajectory_" + std::to_string(trajectory_id);
std::string graph_file = traj_dir + "/grafik.txt";
std::string pos1_file = traj_dir + "/pos1.txt";
std::string pos2_file = traj_dir + "/pos2.txt";
std::string pos3_file = traj_dir + "/pos3.txt";
std::string velo1_file = traj_dir + "/velo1.txt";
std::string velo2_file = traj_dir + "/velo2.txt";
std::string velo3_file = traj_dir + "/velo3.txt";
std::string fc1_file = traj_dir + "/fc1.txt";
std::string fc2_file = traj_dir + "/fc2.txt";
std::string fc3_file = traj_dir + "/fc3.txt";

// Load points
if (!loadPointsFromFile(*traj, pos1_file) ||
    !loadPointsFromFile(*traj, pos2_file) ||
    !loadPointsFromFile(*traj, pos3_file) ||
    !loadPointsFromFile(*traj, velo1_file) ||
    !loadPointsFromFile(*traj, velo2_file) ||
    !loadPointsFromFile(*traj, velo3_file) ||
    !loadPointsFromFile(*traj, fc1_file) ||
    !loadPointsFromFile(*traj, fc2_file) ||
    !loadPointsFromFile(*traj, fc3_file)) {
    return false;
}

// Load graph data
if (!loadGraphData(*traj, graph_file)) {
    return false;
}

std::cout << "Loaded Trajectory " << trajectory_id << ": " 
          << traj->points.size() << " points" << std::endl;

return true;
}
bool TrajectoryManager::loadPointsFromFile(TrajectoryData& traj,
const std::string& filename) {
std::ifstream file(filename);
if (!file.is_open()) {
std::cerr << "Error: Cannot open file " << filename << std::endl;
return false;
}
// Resize points vector if needed
if (traj.points.empty()) {
    traj.points.resize(traj.total_points);
}

// Determine which parameter we're loading
std::string param_type;
if (filename.find("pos1") != std::string::npos) param_type = "pos1";
else if (filename.find("pos2") != std::string::npos) param_type = "pos2";
else if (filename.find("pos3") != std::string::npos) param_type = "pos3";
else if (filename.find("velo1") != std::string::npos) param_type = "velo1";
else if (filename.find("velo2") != std::string::npos) param_type = "velo2";
else if (filename.find("velo3") != std::string::npos) param_type = "velo3";
else if (filename.find("fc1") != std::string::npos) param_type = "fc1";
else if (filename.find("fc2") != std::string::npos) param_type = "fc2";
else if (filename.find("fc3") != std::string::npos) param_type = "fc3";

// Load values
int count = 0;
std::string line;
while (std::getline(file, line) && count < traj.total_points) {
    float value = std::stof(line);
    
    if (param_type == "pos1") traj.points[count].pos1 = value;
    else if (param_type == "pos2") traj.points[count].pos2 = value;
    else if (param_type == "pos3") traj.points[count].pos3 = value;
    else if (param_type == "velo1") traj.points[count].velo1 = value;
    else if (param_type == "velo2") traj.points[count].velo2 = value;
    else if (param_type == "velo3") traj.points[count].velo3 = value;
    else if (param_type == "fc1") traj.points[count].fc1 = value;
    else if (param_type == "fc2") traj.points[count].fc2 = value;
    else if (param_type == "fc3") traj.points[count].fc3 = value;
    
    count++;
}

file.close();
return count == traj.total_points;
}
bool TrajectoryManager::loadGraphData(TrajectoryData& traj,
const std::string& filename) {
std::ifstream file(filename);
if (!file.is_open()) {
std::cerr << "Error: Cannot open graph file " << filename << std::endl;
return false;
}
traj.graph_x.clear();
traj.graph_y.clear();

std::string line;
int count = 0;
while (std::getline(file, line)) {
    size_t comma_pos = line.find(',');
    if (comma_pos == std::string::npos) continue;
    
    float x = std::stof(line.substr(0, comma_pos));
    float y = std::stof(line.substr(comma_pos + 1));
    
    traj.graph_x.push_back(x);
    traj.graph_y.push_back(y);
    count++;
}

file.close();
return count > 0;
}
void TrajectoryManager::switchTrajectory(int trajectory_id) {
TrajectoryData* traj = getTrajectoryData(trajectory_id);
if (traj) {
active_trajectory = traj;
std::cout << "\n*** TRAJECTORY " << trajectory_id << " ACTIVE ***" << std::endl;
std::cout << "Total points: " << traj->total_points << std::endl;
std::cout << "Gait range: " << traj->gait_start_index << " to "
<< traj->gait_end_index << std::endl;
}
}
int TrajectoryManager::getCurrentTrajectoryId() {
return active_trajectory ? active_trajectory->trajectory_id : 1;
}
TrajectoryPoint TrajectoryManager::getPoint(int index) {
if (active_trajectory && isValidIndex(index)) {
return active_trajectory->points[index];
}
return TrajectoryPoint{0, 0, 0, 0, 0, 0, 0, 0, 0};
}
int TrajectoryManager::getTotalPoints() {
return active_trajectory ? active_trajectory->total_points : 0;
}
int TrajectoryManager::getGaitStartIndex() {
return active_trajectory ? active_trajectory->gait_start_index : 0;
}
int TrajectoryManager::getGaitEndIndex() {
return active_trajectory ? active_trajectory->gait_end_index : 0;
}
int TrajectoryManager::getGraphStartIndex() {
return active_trajectory ? active_trajectory->graph_start_index : 0;
}
int TrajectoryManager::getGraphEndIndex() {
return active_trajectory ? active_trajectory->graph_end_index : 0;
}
float* TrajectoryManager::getGraphDataX() {
if (active_trajectory && !active_trajectory->graph_x.empty()) {
return active_trajectory->graph_x.data();
}
return nullptr;
}
float* TrajectoryManager::getGraphDataY() {
if (active_trajectory && !active_trajectory->graph_y.empty()) {
return active_trajectory->graph_y.data();
}
return nullptr;
}
int TrajectoryManager::getGraphPointCount() {
if (active_trajectory) {
return active_trajectory->graph_x.size();
}
return 0;
}
bool TrajectoryManager::isValidTrajectory(int trajectory_id) {
return trajectory_id >= 1 && trajectory_id <= 3;
}
bool TrajectoryManager::isValidIndex(int index) {
return active_trajectory &&
index >= 0 &&
index < (int)active_trajectory->points.size();
}
TrajectoryData* TrajectoryManager::getTrajectoryData(int trajectory_id) {
if (trajectory_id == 1) return &trajectory1;
if (trajectory_id == 2) return &trajectory2;
if (trajectory_id == 3) return &trajectory3;
return nullptr;
}
void TrajectoryManager::printTrajectoryInfo(int trajectory_id) {
TrajectoryData* traj = getTrajectoryData(trajectory_id);
if (!traj) return;
std::cout << "\n========== TRAJECTORY " << trajectory_id << " ==========" << std::endl;
std::cout << "Total Points: " << traj->total_points << std::endl;
std::cout << "Gait Range: " << traj->gait_start_index << " - " 
          << traj->gait_end_index << std::endl;
std::cout << "Graph Range: " << traj->graph_start_index << " - " 
          << traj->graph_end_index << std::endl;
std::cout << "Graph Points Loaded: " << traj->graph_x.size() << std::endl;

if (!traj->points.empty()) {
    std::cout << "First Point: " << traj->points[0].pos1 << ", " 
              << traj->points[0].pos2 << ", " 
              << traj->points[0].pos3 << std::endl;
}
std::cout << "====================================\n" << std::endl;
}
void TrajectoryManager::printLoadStatus() {
std::cout << "\n========== TRAJECTORY LOAD STATUS ==========" << std::endl;
std::cout << "T1 Points: " << trajectory1.points.size() << std::endl;
std::cout << "T2 Points: " << trajectory2.points.size() << std::endl;
std::cout << "T3 Points: " << trajectory3.points.size() << std::endl;
std::cout << "Current Active: T" << getCurrentTrajectoryId() << std::endl;
std::cout << "==========================================\n" << std::endl;
}