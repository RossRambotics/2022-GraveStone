// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDriveToCargo;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Shooter.ShootHigh;

/** Add your docs here. */
public class AutoPoses {
    public final static Pose2d S1 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d S2 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d S3 = new Pose2d(7.85, 3.00, new Rotation2d(Math.toRadians(69.13)));
    public final static Pose2d C1 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d C2 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d C3 = new Pose2d(7.63, 0.41, new Rotation2d(Math.toRadians(90.00)));
    public final static Pose2d C4 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d C5 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d W1 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d W2 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE
    public final static Pose2d W3 = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90.0))); // TODO FIX & REMOVE

    public static CommandBase CaptureCargo(Pose2d pose) {
        CommandBase cmd = new SnapDriveToPoseField(
                RobotContainer.m_drivetrainSubsystem,
                pose,
                0.2);

        cmd = cmd.andThen(new ExtendIntake().withTimeout(0.01))
                .andThen(new StartIntake().withTimeout(0.01))
                .andThen(new SnapDriveToCargo(
                        RobotContainer.m_drivetrainSubsystem,
                        new Rotation2d()));

        return cmd;
    }

    public static CommandBase ShootHigh(Pose2d pose) {
        CommandBase cmd = new SnapDriveToPoseField(
                RobotContainer.m_drivetrainSubsystem,
                pose,
                0.2);

        cmd = cmd.andThen(new ShootHigh());

        return cmd;
    }

    public static void SetStartPose(Pose2d pose) {
        RobotContainer.m_drivetrainSubsystem.setGyroScope(pose.getRotation().getDegrees());
        RobotContainer.m_drivetrainSubsystem.resetOdometry();
        RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(pose,
                RobotContainer.m_drivetrainSubsystem.getGyroHeading());
    }
}
