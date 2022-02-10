// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWhileTracking extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    PIDController m_PIDTracking = null;

    /** Creates a new DriveWhileTracking. */
    public DriveWhileTracking(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        final double ANGULAR_P = 0.1;
        final double ANGULAR_D = 0.0;
        m_PIDTracking = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        if (RobotContainer.getTheRobot().m_Tracking.isTrackingTarget()) {
            // since we are tracking a target use the targets Yaw to spin the robot towards
            // the target
            // gets the offset in degrees
            double p = RobotContainer.getTheRobot().m_Tracking.getHeadingOffset();

            // convert p from degrees to motor power
            double rotationSpeed = m_PIDTracking.calculate(p, 0);

            // System.out.println(
            // "auto turning to target: " + p + " offset error: "
            // + RobotContainer.getTheRobot().m_Tracking.getHeadingOffset()
            // + " Gyro: "
            // +
            // RobotContainer.getTheRobot().m_drivetrainSubsystem.getGyroHeading().getDegrees());

            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            rotationSpeed,
                            m_drivetrainSubsystem.getGyroscopeRotation()));
        } else {
            // if we aren't tracking a target then do normal drive...
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}