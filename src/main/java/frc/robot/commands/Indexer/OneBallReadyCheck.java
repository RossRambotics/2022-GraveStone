// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class OneBallReadyCheck extends CommandBase {
    private boolean m_ballFound = false;

    /** Creates a new OneBallReadyCheck. */
    public OneBallReadyCheck() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Indexer);
        addRequirements(RobotContainer.m_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (RobotContainer.m_Indexer.getSensorIndexerMiddle()) {
            m_ballFound = true;
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.m_Indexer.getSensorIndexerEntry()) {
            RobotContainer.m_Intake.stop();
            RobotContainer.m_Intake.retract();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_ballFound;
    }
}
