// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AimTarget extends CommandBase {
    /** Creates a new AimTarget. */
    public AimTarget() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.m_Targeting.isTrackingTarget()) {
            // DataLogManager.log("*** Target FOUND.");
            // Get the Yaw to the target from the targeting subsystem
            // and send it to the turret subsystem
            RobotContainer.m_Turret.setYawDegreesRelative(
                    RobotContainer.m_Targeting.getTargetOffsetYaw());
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Turret.setPitchDegrees(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
