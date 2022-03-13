// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DefaultIndexer extends CommandBase {
    /** Creates a new DefaultIndexer. */
    public DefaultIndexer() {
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
        if (!RobotContainer.m_Indexer.getSensorIndexerEntry()
                && (!RobotContainer.m_Indexer.getSensorIndexerMiddle())
                && (!RobotContainer.m_Indexer.getSensorIndexerExit())) {
            CommandBase cmd = new frc.robot.commands.Indexer.EmptyCheck();
            cmd.schedule();
        } else if (RobotContainer.m_Indexer.getSensorIndexerMiddle()
                && (!RobotContainer.m_Indexer.getSensorIndexerEntry())) {
            CommandBase cmd = new frc.robot.commands.Indexer.OneBallReadyCheck();
            cmd.schedule();
        } else if (RobotContainer.m_Indexer.getSensorIndexerMiddle()
                && (RobotContainer.m_Indexer.getSensorIndexerEntry())) {
            CommandBase cmd = new frc.robot.commands.Indexer.TwoBallCheck();
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
