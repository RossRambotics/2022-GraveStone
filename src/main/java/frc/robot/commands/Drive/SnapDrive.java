// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SnapDrive extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_goalSupplier;
    private double m_goalDegrees;
    ProfiledPIDController m_PIDTracking = null;

    /** Creates a new DriveWhileTracking. */
    public SnapDrive(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            double goalDegrees) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_goalDegrees = goalDegrees;
        this.m_goalSupplier = null;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    public SnapDrive(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier goalSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_goalDegrees = 720;
        this.m_goalSupplier = goalSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        double ANGULAR_P = RobotContainer.m_Tracking.getAngleP();
        double ANGULAR_D = RobotContainer.m_Tracking.getAngleD();

        TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                3, 3);
        m_PIDTracking = new ProfiledPIDController(
                ANGULAR_P, 0, ANGULAR_D, kThetaControllerConstraints);
        // m_PIDTracking = new PIDController(ANGULAR_P, 0, ANGULAR_D);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        m_PIDTracking.enableContinuousInput(0, 360);
        // m_PIDTracking.reset(getError());
    }

    private double getError() {
        if (m_goalSupplier == null) {
            return m_goalDegrees - RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees();
        }

        // 720 is a special number than can be set as the angle to instantly
        // make the error zero so the robot doesn't change headings.
        if (Math.abs(720.0 - m_goalSupplier.getAsDouble()) < 0.1) {
            return 0;
        }

        return m_goalSupplier.getAsDouble() - RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double p = getError();

        // convert p from degrees to motor power
        double rotationSpeed = -m_PIDTracking.calculate(p, 0);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -3.0, 3.0);
        // DataLogManager.log("Snap: Goal: " + m_goalDegrees + " Error: " + p + "
        // Rotation Speed: " + rotationSpeed);

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        rotationSpeed,
                        m_drivetrainSubsystem.getGyroscopeRotation()),
                rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0), 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}