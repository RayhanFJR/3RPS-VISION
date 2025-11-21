#include <gtest/gtest.h>
#include "../src/trajectory/TrajectoryManager.h"

class TrajectoryManagerTest : public ::testing::Test {
protected:
    TrajectoryManager traj;
};

TEST_F(TrajectoryManagerTest, LoadTrajectory) {
    EXPECT_TRUE(traj.loadAllTrajectories());
    EXPECT_EQ(traj.getTotalPoints(), 816);  // Trajectory 1
}

TEST_F(TrajectoryManagerTest, SwitchTrajectory) {
    traj.switchTrajectory(1);
    EXPECT_EQ(traj.getCurrentTrajectoryId(), 1);
    
    traj.switchTrajectory(2);
    EXPECT_EQ(traj.getCurrentTrajectoryId(), 2);
}

TEST_F(TrajectoryManagerTest, GetPoint) {
    traj.switchTrajectory(1);
    
    TrajectoryPoint pt = traj.getPoint(50);
    
    EXPECT_GE(pt.pos1, 0.0);
    EXPECT_GE(pt.velo1, 0.0);
}

TEST_F(TrajectoryManagerTest, InvalidIndex) {
    traj.switchTrajectory(1);
    
    EXPECT_FALSE(traj.isValidIndex(10000));
}