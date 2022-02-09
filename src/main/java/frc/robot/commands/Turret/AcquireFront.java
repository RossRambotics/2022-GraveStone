// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AcquireFront extends CommandBase {

    /** Creates a new AcquireFront. */
    public AcquireFront() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.getTheRobot().m_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Move the turret to zero
        RobotContainer.getTheRobot().m_Turret.setYawDegreesFront(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if the targeting camera has found the target transition to the TrackTarget
        // command
        if (RobotContainer.getTheRobot().m_Targeting.isTrackingTarget()) {
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
