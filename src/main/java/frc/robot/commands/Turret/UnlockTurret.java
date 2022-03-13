// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class UnlockTurret extends CommandBase {
    private Turret m_Turret = null;

    /** Creates a new UnlockTurret. */
    public UnlockTurret() {
        m_Turret = RobotContainer.m_Turret;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_Turret.unlockTurret();
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
        return true;
    }
}
