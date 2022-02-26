// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Tracking extends SubsystemBase {
    private boolean m_isTesting = false;

    /** Creates a new Tracking. */
    public Tracking() {
        // create the camera
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/creating-photon-camera.html

        // set the appropriate pipeline for the color of the ball based
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }

    public double getHeadingOffset() {
        // TODO Implement this!

        // Get yaw from tracking camera
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/creating-photon-camera.html
        // Use yaw & gyro to calculate target gyro reading
        // return the difference between current gyro reading and target gyro reading

        return m_testTarget.minus(RobotContainer.getTheRobot().m_drivetrainSubsystem.getGyroscopeRotation())
                .getDegrees();
    }

    // used only for testing
    private Rotation2d m_testTarget = new Rotation2d();

    // d is in degrees
    public void setTestTarget(double degrees) {

        m_isTesting = true;
        Rotation2d rotRobot = RobotContainer.getTheRobot().m_drivetrainSubsystem.getGyroscopeRotation();
        Rotation2d testTarget = new Rotation2d(Units.degreesToRadians(degrees));
        Rotation2d rot = rotRobot.plus(testTarget);
    }

    public boolean isTrackingTarget() {
        return true;
    }
}
