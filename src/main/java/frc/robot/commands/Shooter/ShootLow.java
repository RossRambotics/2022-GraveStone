// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootLow extends CommandBase {
    private Timer m_timer = new Timer();

    /** Creates a new ShootLow. */
    public ShootLow() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Shooter,
                RobotContainer.m_Turret,
                RobotContainer.m_Indexer,
                RobotContainer.m_Intake.m_roller);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        DataLogManager.log("ShootLow Initialize.");
        RobotContainer.m_Turret.setYawDegreesFront(0);
        RobotContainer.m_Shooter.shootLow();
        RobotContainer.m_Turret.setPitchDegrees(19);
        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // pause briefly to allow turret to center
        // allow shooter to spin up
        if (!m_timer.hasElapsed(0.2)) {
            return;
        }
        DataLogManager.log("ShootLow Shoot1.");
        RobotContainer.m_Indexer.shoot();

        if (!m_timer.hasElapsed(0.5)) {
            DataLogManager.log("ShootLow Shoot2.");
            RobotContainer.m_Intake.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Shooter.stop();
        RobotContainer.m_Indexer.stop();
        RobotContainer.m_Intake.stop();
        RobotContainer.m_Turret.setPitchDegrees(0);
        DataLogManager.log("ShootLow End.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
