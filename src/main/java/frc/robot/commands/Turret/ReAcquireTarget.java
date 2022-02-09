// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ReAcquireTarget extends CommandBase {
    /** Creates a new ReAcquireTarget. */
    public ReAcquireTarget() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.getTheRobot().m_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getTheRobot().m_Targeting.isTrackingTarget()) {
            // we have found the target again so go back to tracking the target
            CommandBase cmd = new TrackTarget(RobotContainer.getTheRobot().m_Turret);
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
