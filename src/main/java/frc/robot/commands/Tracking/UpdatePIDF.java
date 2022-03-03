// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tracking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Tracking;

public class UpdatePIDF extends CommandBase {
    private Tracking m_tracking = null;

    /** Creates a new UpdatePIDF. */
    public UpdatePIDF() {
        // Use addRequirements() here to declare subsystem dependencies.
        m_tracking = RobotContainer.getTheRobot().m_Tracking;
        ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
