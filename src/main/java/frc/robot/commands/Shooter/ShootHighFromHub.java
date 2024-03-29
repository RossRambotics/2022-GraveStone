// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootHighFromHub extends CommandBase {
    private Timer m_timer = new Timer();
    private boolean m_isShooting = false;
    private boolean m_isShooting2 = false;
    private boolean m_isCompacting = true;
    private CommandBase m_AimCmd = null;

    /** Creates a new Shoot. */
    public ShootHighFromHub() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Shooter, RobotContainer.m_Indexer,
                RobotContainer.m_Intake, RobotContainer.m_Intake.m_roller, RobotContainer.m_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        DataLogManager.log("ShootHighFromHub Initialize:" +
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
        RobotContainer.m_Shooter.shootHighFromHub();
        RobotContainer.m_Turret.setYawDegreesFront(-2.6);
        RobotContainer.m_Turret.setPitchDegrees(3.5);

        m_isShooting = false;
        m_isShooting2 = false;
        m_isCompacting = true;

        // m_AimCmd = new frc.robot.commands.Turret.AimTarget().withTimeout(2.0);
        // m_AimCmd.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (m_isCompacting && (m_timer.hasElapsed(0.2) == false)) {
            RobotContainer.m_Indexer.reverse();
            return;
        } else if (m_isCompacting) {
            m_isCompacting = false;
            RobotContainer.m_Indexer.stop();
            m_timer.reset();
            m_timer.start();
        }

        if (m_timer.hasElapsed(0.2) == false) {
            return;
        }

        if (!m_isShooting) {
            m_isShooting = true;
            m_isShooting2 = true;
            RobotContainer.m_Indexer.shoot();
            m_timer.reset();
            m_timer.start();

            DataLogManager.log("ShootHighFromHub Shoot1:" +
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

        // RobotContainer.m_Indexer.stop();

        // pause briefly then turn on the intake
        // if (m_timer.hasElapsed(0.75) == false) {

        // return;
        // }

        RobotContainer.m_Indexer.shoot();

        DataLogManager.log("ShootHighFromHub Shoot2:" +
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
        RobotContainer.m_Shooter.stop();
        RobotContainer.m_Intake.stop();
        RobotContainer.m_Turret.setYawDegreesFront(0);
        DataLogManager.log("ShootHighFromHub End.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;

    }
}
