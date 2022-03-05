// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootHigh extends CommandBase {
    private Timer m_timer = new Timer();

    /** Creates a new Shoot. */
    public ShootHigh() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Shooter, RobotContainer.m_Indexer, RobotContainer.m_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_Intake.retract();
        RobotContainer.m_Shooter.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.m_Shooter.isSpunUp()) {
            RobotContainer.m_Indexer.shoot();
            m_timer.start();
        }

        // pause briefly then turn on the intake
        if (m_timer.advanceIfElapsed(0.5) == false) {
            return;
        }

        RobotContainer.m_Intake.start();
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
