// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TrackTarget extends CommandBase {
    /** Creates a new TrackTarget. */
    public TrackTarget(Turret t) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(t);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getTheRobot().m_Targeting.isTrackingTarget()) {
            // Get the Yaw to the target from the targeting subsystem
            // and send it to the turret subsystem
            RobotContainer.getTheRobot().m_Turret.setYawDegreesRelative(
                    RobotContainer.getTheRobot().m_Targeting.getTargetOffsetYaw());

        } else {
            // we have lost the target so hold steady for a bit and see if we get it back
            CommandBase cmd = new ReAcquireTarget().withTimeout(2.0);
            cmd.schedule();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
