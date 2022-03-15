// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class ReverseIndexer extends CommandBase {
    private Indexer m_indexer = null;

    /** Creates a new ShootCargo. */
    public ReverseIndexer() {
        m_indexer = RobotContainer.m_Indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(m_indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_indexer.reverse();
        System.out.println("Running reverse indexer");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
