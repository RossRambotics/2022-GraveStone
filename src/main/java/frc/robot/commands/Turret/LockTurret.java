// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class LockTurret extends CommandBase {
    private boolean m_isTurretInitialized = false;
    private boolean m_isTurretInLockPosition = false;
    private boolean m_isTurretLocked = false;
    private CommandBase m_setLEDColor = null;

    /** Creates a new LockTurret. */
    public LockTurret() {
        addRequirements(RobotContainer.m_Turret, RobotContainer.m_Targeting);
        m_setLEDColor = new frc.robot.commands.LEDStrip.TurretLock();

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isTurretInitialized = false;
        m_isTurretInLockPosition = false;
        m_isTurretLocked = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // initialize the turret
        if (!m_isTurretInitialized) {
            m_isTurretInitialized = RobotContainer.m_Turret.initializeTurretLock();
            return;
        }

        // find the lock position
        if (!m_isTurretInLockPosition) {
            m_isTurretInLockPosition = RobotContainer.m_Turret.searchTurretLock();
            return;
        }

        // lock the turret
        m_isTurretLocked = true;
        RobotContainer.m_Turret.lockTurret();

        m_setLEDColor.schedule();
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
