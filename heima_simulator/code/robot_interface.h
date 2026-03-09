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
    
    // Write torques to robot (CST mode 10)
    virtual bool writeTorque(const std::vector<float>& torques) = 0;
    
    // Write positions to robot (CSP mode 8)
    virtual bool writePosition(const std::vector<float>& positions){ return false; }
    
    // Write PVT targets to robot (PVT mode 5: pos, vel, tor, kp, kd)
    // Driver executes: torque = kp*(pos-actPos) + kd*(vel-actVel) + tor
    virtual bool writePVT(const std::vector<float>& pos,
                          const std::vector<float>& vel,
                          const std::vector<float>& tor,
                          const std::vector<float>& kp,
                          const std::vector<float>& kd){ return false; }
    
    // Shutdown the interface
    virtual void shutdown() = 0;
    
    // Get motor count
    virtual int getMotorCount() const = 0;
};

// Factory function to create appropriate interface
// opMode: 5=PVT, 8=CSP, 10=CST (default)
std::unique_ptr<RobotInterface> createRobotInterface(
    const std::string& mode,
    const std::string& config = "",
    const std::vector<unsigned short>& maxCurrent = std::vector<unsigned short>(),
    int ecatCpu = -1,
    int opMode = 10);

#endif // ROBOT_INTERFACE_H

