// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootHigh extends CommandBase {
    private Timer m_timer = new Timer();
    private boolean m_isShooting = false;

    /** Creates a new Shoot. */
    public ShootHigh() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Shooter, RobotContainer.m_Indexer,
                RobotContainer.m_Intake, RobotContainer.m_Intake.m_roller);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        DataLogManager.log("ShootHigh Initialize:" +
                " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());
        RobotContainer.m_Intake.retract();
        RobotContainer.m_Shooter.start();
        m_isShooting = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!m_isShooting && RobotContainer.m_Shooter.isSpunUp()) {
            m_isShooting = true;
            RobotContainer.m_Indexer.shoot();
            m_timer.reset();
            m_timer.start();

            DataLogManager.log("ShootHigh Shoot1:" +
                    " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                    " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                    " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                    " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                    " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                    " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                    " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                    " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());
        } else {
            return;
        }

        // pause briefly then turn on the intake
        if (m_timer.hasElapsed(0.5) == false) {
            return;
        }

        DataLogManager.log("ShootHigh Shoot2:" +
                " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());

        RobotContainer.m_Intake.start();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
