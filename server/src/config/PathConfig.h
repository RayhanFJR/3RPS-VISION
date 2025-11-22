//==================================================================
// FILE: server/src/config/PathConfig.h
// Configuration untuk path management
//==================================================================

#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H

#include <string>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

class PathConfig {
public:
    static std::string getProjectRoot() {
        // Method 1: Try environment variable
        const char* project_root = std::getenv("REHAB_PROJECT_ROOT");
        if (project_root) {
            std::string root(project_root);
            if (fs::exists(root)) {
                return root;
            }
        }
        
        // Method 2: Assume running from server/build
        // Go up 2 levels: build -> server -> project_root
        fs::path current = fs::current_path();
        
        // If we're in build directory
        if (current.filename() == "build") {
            fs::path project_root = current.parent_path().parent_path();
            if (fs::exists(project_root / "data")) {
                return project_root.string();
            }
        }
        
        // Method 3: Look for 'data' folder in parent directories
        fs::path search_path = current;
        for (int i = 0; i < 5; ++i) {
            if (fs::exists(search_path / "data")) {
                return search_path.string();
            }
            search_path = search_path.parent_path();
        }
        
        // Method 4: Default to current directory
        return current.string();
    }
    
    static std::string getDataDir() {
        return getProjectRoot() + "/data";
    }
    
    static std::string getTrajectoryDir(int traj_num) {
        return getDataDir() + "/trajectory_" + std::to_string(traj_num);
    }
    
    static void printPaths() {
        std::cout << "\n========== PATH CONFIGURATION ==========" << std::endl;
        std::cout << "Current directory: " << fs::current_path() << std::endl;
        std::cout << "Project root: " << getProjectRoot() << std::endl;
        std::cout << "Data dir: " << getDataDir() << std::endl;
        std::cout << "Trajectory 1: " << getTrajectoryDir(1) << std::endl;
        std::cout << "Trajectory 2: " << getTrajectoryDir(2) << std::endl;
        std::cout << "Trajectory 3: " << getTrajectoryDir(3) << std::endl;
        std::cout << "========================================\n" << std::endl;
    }
};

#endif  // PATH_CONFIG_H