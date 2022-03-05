// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class UpdatePIDF extends CommandBase {

    private Intake m_intake = null;

    /** Creates a new UpdatePIDF. */
    public UpdatePIDF() {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = RobotContainer.m_Intake;
        ;

        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.updatePIDF();
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
