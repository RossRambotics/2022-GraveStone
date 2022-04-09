// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCargo extends CommandBase {
    /** Creates a new IntakeCargo. */
    public IntakeCargo() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_Intake.extend();
        RobotContainer.m_Intake.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Intake.retract();
        RobotContainer.m_Intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
