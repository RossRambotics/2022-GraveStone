// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SnapDriveToPoseField extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Pose2d m_goal;
    ProfiledPIDController m_rotationPID = null;
    ProfiledPIDController m_xPID = null;
    ProfiledPIDController m_yPID = null;
    private Pose2d m_error = getError();

    /** Creates a new DriveWhileTracking. */
    public SnapDriveToPoseField(DrivetrainSubsystem drivetrainSubsystem,
            Pose2d goal) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_goal = goal;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        double ANGULAR_P = 0.1;
        double ANGULAR_D = 0.001;
        double TRANSLATE_P = 0.1;
        double TRANSLATE_D = 0.001;

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                3, 3);
        m_rotationPID = new ProfiledPIDController(
                ANGULAR_P, 0, ANGULAR_D, rotationConstraints);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        m_rotationPID.enableContinuousInput(0, 360);

        TrapezoidProfile.Constraints translateConstraints = new TrapezoidProfile.Constraints(
                1, 1);
        m_xPID = new ProfiledPIDController(TRANSLATE_P, 0, TRANSLATE_D, translateConstraints);
        m_yPID = new ProfiledPIDController(TRANSLATE_P, 0, TRANSLATE_D, translateConstraints);

    }

    private Pose2d getError() {
        Pose2d current = RobotContainer.m_drivetrainSubsystem.getOdometryPose();
        Pose2d error = new Pose2d(m_goal.getX() - current.getX(),
                m_goal.getY() - current.getY(),
                m_goal.getRotation().minus(current.getRotation()));

        DataLogManager.log("SnapDriveToPoseField: Current: " + current + " Error: " + error);

        return error;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // update error
        m_error = getError();

        // update PIDs
        double rotationSpeed = -m_rotationPID.calculate(m_error.getRotation().getDegrees(), 0);

        rotationSpeed = MathUtil.clamp(rotationSpeed, -3.0, 3.0);

        double translateSpeedX = m_xPID.calculate(m_error.getX(), 0);
        double translateSpeedY = m_yPID.calculate(m_error.getY(), 0);

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translateSpeedX,
                        translateSpeedY,
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
        // end if we have reached target Pose
        if ((Math.abs(m_error.getX()) < 0.01)
                && (Math.abs(m_error.getY()) < 0.01)
                && (Math.abs(m_error.getRotation().getDegrees()) < 1.0)) {
            return true;
        }
        return false;
    }
}