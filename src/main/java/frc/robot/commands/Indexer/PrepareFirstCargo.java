// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PrepareFirstCargo extends CommandBase {
    boolean m_isFinished = false;

    /** Creates a new PrepareFirstCargo. */
    public PrepareFirstCargo() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // move the ball forward until the 2nd sensor sees it
        if (!RobotContainer.m_Indexer.getSensorIndexerMiddle()) {
            RobotContainer.m_Indexer.slow();
        } else {
            RobotContainer.m_Indexer.stop();
            m_isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
