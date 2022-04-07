// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto3;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.ResetIntake;

public class S2_C1_W1_High extends CommandBase {
    /** Creates a new S3_Low_C3_High. */
    public S2_C1_W1_High() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Ben
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Log the auto command name
        DataLogManager.log("Auto command: " + this.getName());

        // Set Starting Pose
        AutoPoses.SetStartPose(AutoPoses.S2);

        // Create command group for the auto routine
        SequentialCommandGroup command = new SequentialCommandGroup(
                new ResetIntake().withTimeout(0.2),
                AutoPoses.CaptureCargo(AutoPoses.C1).withTimeout(7.0),
                AutoPoses.ShootHigh(AutoPoses.W1).withTimeout(10.0));

        command.schedule();
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
