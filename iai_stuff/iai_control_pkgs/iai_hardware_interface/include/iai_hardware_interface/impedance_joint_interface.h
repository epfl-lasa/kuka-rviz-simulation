#ifndef IAI_HARDWARE_INTERFACE_IMPEDANCE_JOINT_INTERFACE_H
#define IAI_HARDWARE_INTERFACE_IMPEDANCE_JOINT_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{
  class ImpedanceJointHandle : public JointHandle
  {
    public :
      ImpedanceJointHandle() 
        : JointHandle(), stiffness_(0), damping_(0) {}

      ImpedanceJointHandle(const JointHandle& jh, double* stiffness, double* damping)
        : JointHandle(jh), stiffness_(stiffness), damping_(damping)
      {
        if (!stiffness_)
        {
          throw HardwareInterfaceException("Cannot create impedance handle '" + jh.getName() + "'. Stiffness data pointer is null.");
        }
        if (!damping_)
        {
          throw HardwareInterfaceException("Cannot create impedance handle '" + jh.getName() + "'. Damping data pointer is null.");
        }
      }

      void setStiffness(double stiffness) {assert(stiffness_); *stiffness_ = stiffness;}
      double getStiffness() const {assert(stiffness_); return *stiffness_;}

      void setDamping(double damping) {assert(damping_); *damping_ = damping;}
      double getDamping() const {assert(damping_); return *damping_;}

    private:
      double *stiffness_, *damping_;
  };

  class ImpedanceJointInterface : public HardwareResourceManager<ImpedanceJointHandle, ClaimResources> {};

  class EffortImpedanceJointInterface : public ImpedanceJointInterface {};
  class VelocityImpedanceJointInterface : public ImpedanceJointInterface {};
  class PositionImpedanceJointInterface : public ImpedanceJointInterface {};
}

#endif 
