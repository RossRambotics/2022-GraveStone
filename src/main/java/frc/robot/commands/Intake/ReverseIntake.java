// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends CommandBase {
    private Intake m_intake = null;

    /** Creates a new StartIntake. */
    public ReverseIntake() {
        m_intake = RobotContainer.m_Intake;
        ;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(RobotContainer.m_Intake,
                RobotContainer.m_Indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.reverse();
        RobotContainer.m_Indexer.reverse();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        RobotContainer.m_Indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
