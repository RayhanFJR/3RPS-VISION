//==================================================================
// FILE : server/src/trajectory/TrajectoryManager.h
//==================================================================
#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H
#include <vector>
#include <string>
#include <cstring>
// ============ TRAJECTORY STRUCTURE ============
struct TrajectoryPoint {
float pos1, pos2, pos3;      // Position for 3 motors
float velo1, velo2, velo3;   // Velocity
float fc1, fc2, fc3;         // Feedforward force
};
struct TrajectoryData {
int trajectory_id;           // 1, 2, or 3
int total_points;            // Total points in trajectory
int gait_start_index;        // Start of main gait cycle
int gait_end_index;          // End of main gait cycle
int graph_start_index;       // HMI display start
int graph_end_index;         // HMI display end
std::vector<TrajectoryPoint> points;
std::vector<float> graph_x;
std::vector<float> graph_y;
};
// ============ CLASS DEFINITION ============
class TrajectoryManager {
public:
TrajectoryManager();
~TrajectoryManager();
// ========== INITIALIZATION ==========
bool loadAllTrajectories();
bool loadTrajectory(int trajectory_id, const std::string& data_dir);

// ========== TRAJECTORY SWITCHING ==========
void switchTrajectory(int trajectory_id);
int getCurrentTrajectoryId();

// ========== POINT ACCESS ==========
TrajectoryPoint getPoint(int index);
int getTotalPoints();
int getGaitStartIndex();
int getGaitEndIndex();
int getGraphStartIndex();
int getGraphEndIndex();

// ========== GRAPH DATA ==========
float* getGraphDataX();
float* getGraphDataY();
int getGraphPointCount();

// ========== VALIDATION ==========
bool isValidTrajectory(int trajectory_id);
bool isValidIndex(int index);

// ========== DEBUG ==========
void printTrajectoryInfo(int trajectory_id);
void printLoadStatus();
private:
// ========== TRAJECTORY STORAGE ==========
TrajectoryData trajectory1;
TrajectoryData trajectory2;
TrajectoryData trajectory3;
TrajectoryData* active_trajectory;
// ========== HELPER FUNCTIONS ==========
TrajectoryData* getTrajectoryData(int trajectory_id);
bool loadPointsFromFile(TrajectoryData& traj, const std::string& filename);
bool loadGraphData(TrajectoryData& traj, const std::string& filename);

// ========== CONFIGURATION ==========
void configureTrajectory1();
void configureTrajectory2();
void configureTrajectory3();
};
#endif  // TRAJECTORY_MANAGER_H