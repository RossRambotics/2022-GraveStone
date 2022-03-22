// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.ShootHigh;
import frc.robot.commands.Shooter.ShootLow;

public class Start3LowBackHigh extends CommandBase {
    /** Creates a new Start3LowBackHigh. */
    public Start3LowBackHigh() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // setup odometry
        // Pose2d start = new Pose2d(7.14, 1.55, new Rotation2d(Math.toRadians(69.13)));
        Pose2d start = new Pose2d(5.0 * 1.15, 1, new Rotation2d(Math.toRadians(0)));
        RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(start,
                RobotContainer.m_drivetrainSubsystem.getGyroHeading());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // CommandBase gotoPose1 = new
        // SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
        // new Pose2d(7.63, 0.41, new Rotation2d(Math.toRadians(90.00))));
        CommandBase gotoPose1 = new SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
                new Pose2d(0, 1, new Rotation2d(Math.toRadians(0.00))));

        SequentialCommandGroup command = new SequentialCommandGroup(
                new ShootLow().withTimeout(2.6),
                new ExtendIntake().withTimeout(0.1),
                new StartIntake().withTimeout(0.1), gotoPose1, new ShootHigh().withTimeout(5),
                new StopIntake().withTimeout(0.1), new RetractIntake().withTimeout(0.1));

        command.schedule();
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
