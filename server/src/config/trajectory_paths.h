#ifndef TRAJECTORY_PATHS_H
#define TRAJECTORY_PATHS_H

#include <string>

namespace TrajectoryPaths {
    // Base path untuk data trajektori
    const std::string BASE_PATH = "data/";
    
    // Path untuk setiap trajektori
    const std::string TRAJECTORY_1_DIR = BASE_PATH + "trajectory_1/";
    const std::string TRAJECTORY_2_DIR = BASE_PATH + "trajectory_2/";
    const std::string TRAJECTORY_3_DIR = BASE_PATH + "trajectory_3/";
    
    // Nama file data
    const std::string GRAFIK_FILE = "grafik.txt";
    const std::string POS1_FILE = "pos1.txt";
    const std::string POS2_FILE = "pos2.txt";
    const std::string POS3_FILE = "pos3.txt";
    const std::string VELO1_FILE = "velo1.txt";
    const std::string VELO2_FILE = "velo2.txt";
    const std::string VELO3_FILE = "velo3.txt";
    const std::string FC1_FILE = "fc1.txt";
    const std::string FC2_FILE = "fc2.txt";
    const std::string FC3_FILE = "fc3.txt";
    
    // Helper function untuk mendapatkan full path
    inline std::string getFilePath(const std::string& trajectory_dir, const std::string& filename) {
        return trajectory_dir + filename;
    }
}

#endif // TRAJECTORY_PATHS_H