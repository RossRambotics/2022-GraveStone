// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SnapDriveToCargo extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private Rotation2d m_goal;
    ProfiledPIDController m_rotationPID = null;
    ProfiledPIDController m_xPID = null;
    private Rotation2d m_rotError = null;
    private double m_xError;
    private int m_lostCargoFrames = 0;

    /** Creates a new DriveWhileTracking. */
    /**
     * 
     * @param drivetrainSubsystem Drive subsystem
     * @param goal                the goal angle that the robot should move to
     * 
     */
    public SnapDriveToCargo(DrivetrainSubsystem drivetrainSubsystem,
            Rotation2d goal) {

        this.m_drivetrainSubsystem = drivetrainSubsystem;
        m_goal = goal;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                1.0, 0.5);
        m_rotationPID = new ProfiledPIDController(
                SnapConstants.kANGULAR_P, 0, SnapConstants.kANGULAR_D, rotationConstraints);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        m_rotationPID.enableContinuousInput(0, 360);

        TrapezoidProfile.Constraints translateConstraints = new TrapezoidProfile.Constraints(
                SnapConstants.kMAX_TRANSLATE_VELOCITY, SnapConstants.kMAX_TRANSLATE_ACCEL);
        m_xPID = new ProfiledPIDController(SnapConstants.kTRANSLATE_P, 0,
                SnapConstants.kTRANSLATE_D, translateConstraints);

        // setup the X/Y PIDs
        double error = getXError();
        m_xPID.reset(error);

        m_lostCargoFrames = 0;

        RobotContainer.m_Tracking.enableSearchLight();
    }

    private double getXError() {
        double error = 0;
        if (RobotContainer.m_Tracking.isTrackingTarget()) {
            error = RobotContainer.m_Tracking.getHeadingOffset();
        }

        DataLogManager.log("SnapDriveToCargo: isTracking?: "
                + RobotContainer.m_Tracking.isTrackingTarget()
                + " X Error: " + error);

        // TODO tune the factor below
        return error * 0.05;
    }

    private Rotation2d getRotError() {
        Pose2d current = RobotContainer.m_drivetrainSubsystem.getOdometryPose();

        Rotation2d error = m_goal.minus(current.getRotation());

        DataLogManager.log("SnapDriveToCargo: Rotation Current: " + current + " Error: " + error);

        return error;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // update error
        m_rotError = getRotError();
        m_xError = getXError();

        // update PIDs
        double rotationSpeed = m_rotationPID.calculate(m_rotError.getDegrees(), 0);

        rotationSpeed = MathUtil.clamp(rotationSpeed, -3.0, 3.0);

        if (Math.abs(rotationSpeed) < 0.05) {
            rotationSpeed = 0;
        }

        double translateSpeedX = m_xPID.calculate(m_xError, 0);
        translateSpeedX = MathUtil.clamp(translateSpeedX, -SnapConstants.kMAX_TRANSLATE_VELOCITY,
                SnapConstants.kMAX_TRANSLATE_VELOCITY);

        double TRANSLATE_FF = 0.1;
        if (translateSpeedX > 0) {
            translateSpeedX += TRANSLATE_FF;
        } else {
            translateSpeedX -= TRANSLATE_FF;
        }

        double translateSpeedY = 0;
        if (translateSpeedX < 0.2) {
            translateSpeedY = -0.2;
        }

        DataLogManager.log("SnapDriveToCargo: Corrections: X: " + translateSpeedX + " Y: " + translateSpeedY
                + " Rot: " + rotationSpeed);

        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                        -translateSpeedX,
                        -translateSpeedY,
                        rotationSpeed),
                rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0), 0.0);
        RobotContainer.m_Tracking.disableSearchLight();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // end if we go 5 frames without tracking a cargo
        if (RobotContainer.m_Tracking.isTrackingTarget()) {
            m_lostCargoFrames = 0;
            return false;
        }

        if (m_lostCargoFrames++ > 4) {
            return true;
        }

        return false;
    }
}