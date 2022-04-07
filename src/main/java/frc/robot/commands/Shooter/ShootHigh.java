// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import frc.robot.RobotContainer;

public class ShootHigh extends CommandBase {
    private Timer m_timer = new Timer();
    private boolean m_isShooting = false;
    private boolean m_isShooting2 = false;
    private boolean m_isCompacting = true;
    private CommandBase m_AimCmd = null;

    // extra
    /** Creates a new Shoot. */
    public ShootHigh() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Shooter, RobotContainer.m_Indexer,
                RobotContainer.m_Intake, RobotContainer.m_Intake.m_roller, RobotContainer.m_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_Targeting.ResetSnapshot();
        m_timer.reset();
        m_timer.start();
        DataLogManager.log("ShootHigh Initialize:" +
                " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                " Indexer (a) RPM: " + RobotContainer.m_Indexer.getfrountwheelrpm() +
                " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());
        RobotContainer.m_Intake.retract();
        RobotContainer.m_Intake.stop();
        RobotContainer.m_Shooter.shootHigh();

        m_isShooting = false;
        m_isShooting2 = false;
        m_isCompacting = true;

        m_AimCmd = new frc.robot.commands.Turret.AimTarget().withTimeout(5.0);
        m_AimCmd.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // update firing solution
        if (m_timer.hasElapsed(0.2)) {
            RobotContainer.m_Shooter.shootHigh();
        }

        if (m_isCompacting && (m_timer.hasElapsed(0.2) == false)) {
            RobotContainer.m_Indexer.reverse();
            return;
        } else if (m_isCompacting) {
            m_isCompacting = false;
            RobotContainer.m_Indexer.stop();
            m_timer.reset();
            m_timer.start();
        }

        if (m_timer.hasElapsed(0.3) == false) {
            return;
        }

        if (RobotContainer.m_Turret.getIsOnTarget() == false) {
            DataLogManager.log("Shoot High: Not on target yet...");
            return;
        }

        if (!m_isShooting && RobotContainer.m_Shooter.isSpunUp()) {
            m_isShooting = true;
            m_isShooting2 = true;
            RobotContainer.m_Indexer.shoot();
            m_timer.reset();
            m_timer.start();
            RobotContainer.m_Targeting.TakeSnapshot();

            DataLogManager.log("ShootHigh Shoot1:" +
                    " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                    " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                    " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                    " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                    " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                    " Indexer (a) RPM: " + RobotContainer.m_Indexer.getfrountwheelrpm() +
                    " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                    " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                    " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());
        } else if (!m_isShooting2) {

            return;
        }

        if (m_timer.hasElapsed(0.25) == false) {
            return;
        }

        RobotContainer.m_Indexer.stop();

        // update firing solution
        RobotContainer.m_Shooter.shootHigh();

        // pause briefly then turn on the intake
        if (m_timer.hasElapsed(0.75) == false) {

            return;
        }
        // update firing solution
        RobotContainer.m_Shooter.shootHigh();
        RobotContainer.m_Indexer.shoot();

        DataLogManager.log("ShootHigh Shoot2:" +
                " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                " Indexer (a) RPM: " + RobotContainer.m_Indexer.getfrountwheelrpm() +
                " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());

        RobotContainer.m_Intake.start();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_AimCmd.cancel();
        RobotContainer.m_Shooter.stop();
        RobotContainer.m_Intake.stop();
        RobotContainer.m_Turret.setYawDegreesFront(0);
        RobotContainer.m_Turret.setPitchDegrees(0);

        DataLogManager.log("ShootHigh End.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;

    }
}
