// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import frc.robot.subsystems.Climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultClimb extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Climb m_Climb;
    private DoubleSupplier m_AxisSupplier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DefaultClimb(Climb subsystem, DoubleSupplier ClimbAxis) {
        m_Climb = subsystem;
        m_AxisSupplier = ClimbAxis;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double power = m_AxisSupplier.getAsDouble();

        // DataLogManager.log("Climb power: " + power
        // + " climb position: " + m_Climb.getClimbEncoderPosition());

        if (power > 0) {
            if (m_Climb.getClimbEncoderPosition() < m_Climb.getMaxClimbPosition()) {
                m_Climb.setClimbPower(-power);
            } else {
                m_Climb.setClimbPower(0);
            }
        } else {
            if (m_Climb.getClimbEncoderPosition() > m_Climb.getMinClimbPosition()) {
                m_Climb.setClimbPower(-power);
            } else {
                m_Climb.setClimbPower(0);
            }
        }

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
