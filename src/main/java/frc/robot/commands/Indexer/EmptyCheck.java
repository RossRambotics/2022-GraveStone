// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class EmptyCheck extends CommandBase {
    boolean m_isFinished = false;
    boolean m_found = false;

    /** Creates a new EmptyCheck. */
    public EmptyCheck() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.getTheRobot().m_Indexer);
        // addRequirements(RobotContainer.m_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (RobotContainer.m_Indexer.getSensorIndexerEntry()) {
            RobotContainer.m_Indexer.slow();
            RobotContainer.m_Intake.slow();
            m_found = true;
            m_isFinished = false;
        } else {
            m_isFinished = true;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.m_Indexer.getSensorIndexerMiddle()) {
            m_isFinished = true;
        }
        // System.out.println(RobotContainer.m_Indexer.getSensorIndexerMiddle());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Indexer.stop();
        if (m_found)
            RobotContainer.m_Intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
