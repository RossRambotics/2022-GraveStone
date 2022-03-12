// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDStrip;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BallNotFound extends CommandBase {
    /** Creates a new BallNotFound. */
    public BallNotFound() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_LEDStrip.m_tracking);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Inside Ball Not Found method");
        RobotContainer.m_LEDStrip.setBallBlack();
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
