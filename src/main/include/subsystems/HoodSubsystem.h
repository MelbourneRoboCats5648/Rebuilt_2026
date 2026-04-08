#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <subsystems/DriveSubsystem.h>
#include <constants/HoodConstants.h>

class HoodSubsystem : public frc2::SubsystemBase {
    public:
    HoodSubsystem(DriveSubsystem& drive);

    private:
    
};