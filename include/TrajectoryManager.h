#ifndef TRAJECTORYMANAGER_H
#define TRAJECTORYMANAGER_H

#include <string>

// Forward declarations
struct TrajectoryData;

class TrajectoryManager {
public:
    TrajectoryManager();
    ~TrajectoryManager();
    
    // Loading functions
    bool loadAllTrajectoryData(const std::string& basePath = "../data");
    bool loadTrajectory(int trajNum, const std::string& basePath);
    
    // Trajectory switching
    void switchTrajectory(int trajNum);
    int getActiveTrajectory() const { return activeTrajectory; }
    
    // Data access
    float (*getActiveGraphData())[2];
    double* getActivePos1() const;
    double* getActivePos2() const;
    double* getActivePos3() const;
    double* getActiveVelo1() const;
    double* getActiveVelo2() const;
    double* getActiveVelo3() const;
    double* getActiveFc1() const;
    double* getActiveFc2() const;
    double* getActiveFc3() const;
    
    // Configuration access
    int getActivePointCount() const;
    int getGraphStartIndex() const;
    int getGraphEndIndex() const;
    int getGraphPointCount() const;
    int getGaitStartIndex() const;
    int getGaitEndIndex() const;
    int getGaitPointCount() const;

private:
    TrajectoryData* trajectory1;
    TrajectoryData* trajectory2;
    TrajectoryData* trajectory3;
    
    TrajectoryData* activeTrajectoryData;
    int activeTrajectory;
    
    // Helper functions
    bool loadDataGrafik(const std::string& filename, float array[][2], int expectedCount);
    bool loadDoubleArray(const std::string& filename, double* array, int size);
};

#endif // TRAJECTORYMANAGER_H

