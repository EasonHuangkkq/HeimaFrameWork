/*
 * Robot Interface - Abstract interface for both simulator and real robot
 */

#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <vector>
#include <memory>
#include <string>

// Abstract robot interface
class RobotInterface {
public:
    virtual ~RobotInterface() = default;
    
    // Initialize the robot interface
    virtual bool init() = 0;
    
    // Fetch current observations from robot
    // Returns: rpy (3), base_ang_vel (3), joint_pos (12), joint_vel (12)
    virtual bool fetchObs(
        std::vector<float>& rpy,
        std::vector<float>& base_ang_vel,
        std::vector<float>& joint_pos,
        std::vector<float>& joint_vel) = 0;
    
    // Write torques to robot (12 leg joints)
    virtual bool writeTorque(const std::vector<float>& torques) = 0;
    
    // Shutdown the interface
    virtual void shutdown() = 0;
    
    // Get motor count
    virtual int getMotorCount() const = 0;
};

// Factory function to create appropriate interface
std::unique_ptr<RobotInterface> createRobotInterface(
    const std::string& mode,
    const std::string& config = "");

#endif // ROBOT_INTERFACE_H

