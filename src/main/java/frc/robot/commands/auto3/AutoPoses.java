// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDriveToCargo;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.ShootHigh;

/** Add your docs here. */
public class AutoPoses {
        public final static Pose2d S1 = new Pose2d(6.53, 5.37, new Rotation2d(Math.toRadians(-41.01)));
        public final static Pose2d S2 = new Pose2d(7.03, 4.57, new Rotation2d(Math.toRadians(-22.11)));
        public final static Pose2d S3 = new Pose2d(7.7, 2.75, new Rotation2d(Math.toRadians(69)));
        public final static Pose2d C1 = new Pose2d(5.77, 5.86, new Rotation2d(Math.toRadians(-22.84)));
        public final static Pose2d C2 = new Pose2d(5.81, 2.31, new Rotation2d(Math.toRadians(29.75)));
        public final static Pose2d C3 = new Pose2d(7.32, 1.68, new Rotation2d(Math.toRadians(90.00)));
        public final static Pose2d C4 = new Pose2d(1.77, 1.66, new Rotation2d(Math.toRadians(45.0)));
        public final static Pose2d C5 = new Pose2d(1.77, 1.66, new Rotation2d(Math.toRadians(45.0)));
        public final static Pose2d W1 = new Pose2d(5.54, 5.17, new Rotation2d(Math.toRadians(-17.98)));
        public final static Pose2d W2 = new Pose2d(4.46, 4.1, new Rotation2d(Math.toRadians(3.91)));
        public final static Pose2d W3 = new Pose2d(7.19, 1.36, new Rotation2d(Math.toRadians(70.94)));
        public final static Pose2d W4 = new Pose2d(6.08, 1.21, new Rotation2d(Math.toRadians(52.74)));

        public static CommandBase CaptureCargo(Pose2d pose) {
                CommandBase cmd = new SnapDriveToPoseField(
                                RobotContainer.m_drivetrainSubsystem,
                                pose,
                                0.2);

                cmd = cmd.andThen(new ExtendIntake().withTimeout(0.01))
                                .andThen(new StartIntake().withTimeout(0.01))
                                .andThen(new SnapDriveToCargo(
                                                RobotContainer.m_drivetrainSubsystem,
                                                new Rotation2d())
                                                                .andThen(new RetractIntake().withTimeout(0.01))
                                                                .andThen(new StopIntake().withTimeout(0.01)));

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
                // RobotContainer.m_drivetrainSubsystem.zeroGyroscope();
                DataLogManager
                                .log("Setting Start Pose: Current Gyro: "
                                                + RobotContainer.m_drivetrainSubsystem.getGyroscopeRotation());
                RobotContainer.m_drivetrainSubsystem.setGyroScope(pose.getRotation().getDegrees());
                DataLogManager
                                .log("Setting Start Pose: Updated Gyro: "
                                                + RobotContainer.m_drivetrainSubsystem.getGyroscopeRotation());
                // RobotContainer.m_drivetrainSubsystem.resetOdometry();
                DataLogManager.log(
                                "Setting Start Pose: Odometry Orignal Pose: "
                                                + RobotContainer.m_drivetrainSubsystem.getOdometryPose());

                if (Robot.isSimulation()) {
                        RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(pose,
                                        new Rotation2d(Math.toRadians(69.0)));
                } else {
                        RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(pose,
                                        pose.getRotation()); // uses pose, not gyro because gyro update above is async
                                                             // and may not be
                                                             // updated by now
                }
                DataLogManager.log(
                                "Setting Start Pose: Odometry Updated Pose: "
                                                + RobotContainer.m_drivetrainSubsystem.getOdometryPose());
        }
}
