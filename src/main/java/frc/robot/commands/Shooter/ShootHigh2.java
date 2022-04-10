// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootHigh2 extends CommandBase {
    private Timer m_timer = new Timer();
    private Timer m_shooterUpdateTimer = new Timer(); // used to determine how frequently to update firing solution
    private int m_phase = 0;
    private CommandBase m_AimCmd = null;
    private boolean m_finished = false;

    // extra
    /** Creates a new Shoot. */
    public ShootHigh2() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_Shooter, RobotContainer.m_Indexer,
                RobotContainer.m_Intake, RobotContainer.m_Intake.m_roller, RobotContainer.m_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_shooterUpdateTimer.reset();
        m_shooterUpdateTimer.start();
        DataLogManager.log("ShootHigh2 Initialize:" +
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

        m_phase = 0;

        m_AimCmd = new frc.robot.commands.Turret.AimTarget().withTimeout(5.0);
        m_AimCmd.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // update firing solution
        if (m_shooterUpdateTimer.advanceIfElapsed(0.2)) {
            RobotContainer.m_Shooter.shootHigh();
        }

        if (m_phase == 0) { // phase 0 so initializing
            m_phase = 1;
        } else if (m_phase == 1) {
            // get shooter up to speed && compact indexer
            DataLogManager.log("ShootHigh2 Phase 1: Compacting Indexer...");
            if (m_timer.hasElapsed(0.2) == false) {
                RobotContainer.m_Indexer.reverse();
            } else {
                RobotContainer.m_Indexer.stop();
                m_phase = 2;
            }
        } else if (m_phase == 2) {
            // check shooter if lined up and ready!
            if (RobotContainer.m_Turret.getIsOnTarget() == false) {
                DataLogManager.log("ShootHigh2 Phase 2: Turrent yaw not on target yet...");
                return;
            }
            if (RobotContainer.m_Shooter.isSpunUp() == false) {
                DataLogManager.log("ShootHigh2 Phase 2: Shooter RPMs not up to speed yet...");
                return;
            }
            m_phase = 3;
            m_timer.reset();
            RobotContainer.m_Indexer.shoot();
        } else if (m_phase == 3) {
            // shoot 1st cargo
            DataLogManager.log("ShootHigh2 Phase 3 Shoot1:" +
                    " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                    " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                    " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                    " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                    " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                    " Indexer (a) RPM: " + RobotContainer.m_Indexer.getfrountwheelrpm() +
                    " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                    " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                    " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());

            if (m_timer.hasElapsed(0.25) == false) {
                return;
            }

            // should be done shooting so stop indexer and move to next phase
            RobotContainer.m_Indexer.stop();
            m_phase = 4;
            m_timer.reset();
        } else if (m_phase == 4) {
            // pause briefly then shoot the 2nd cargo
            if (m_timer.hasElapsed(0.75) == false) {
                DataLogManager.log("ShootHigh2 Phase 4:  Waiting for indexer to stop.");
                return;
            }
            RobotContainer.m_Indexer.shoot();
            RobotContainer.m_Intake.start();
            m_timer.reset();
            m_phase = 5;
        } else if (m_phase == 5) {
            // shoot the 2nd cargo & then end
            if (m_timer.hasElapsed(0.25) == false) {
                DataLogManager.log("ShootHigh2 Phase 5: Shoot2:" +
                        " Target Yaw: " + RobotContainer.m_Targeting.getTargetOffsetYaw() +
                        " Target Distance: " + RobotContainer.m_Targeting.getTargetDistance() +
                        " Turret Pitch: " + RobotContainer.m_Turret.getPitch() +
                        " Turret Yaw: " + RobotContainer.m_Turret.getYaw() +
                        " Target Found: " + RobotContainer.m_Turret.getIsOnTarget() +
                        " Indexer (a) RPM: " + RobotContainer.m_Indexer.getfrountwheelrpm() +
                        " Indexer (e) RPM: " + RobotContainer.m_Indexer.getIndexError() +
                        " Shooter (a) RPM: " + RobotContainer.m_Shooter.getRPM() +
                        " Shooter (e) RPM: " + RobotContainer.m_Shooter.getErrorRPM());
                return;
            }
            m_finished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_AimCmd.cancel();
        RobotContainer.m_Shooter.stop();
        RobotContainer.m_Intake.stop();
        RobotContainer.m_Indexer.stop();
        RobotContainer.m_Turret.setYawDegreesFront(0);
        RobotContainer.m_Turret.setPitchDegrees(0);
        DataLogManager.log("ShootHigh2 End.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_finished;

    }
}
