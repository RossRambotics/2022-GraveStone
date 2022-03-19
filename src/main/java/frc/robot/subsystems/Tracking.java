// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWhileTracking;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Tracking extends SubsystemBase {

    private NetworkTableEntry m_currentYaw = null;
    private NetworkTableEntry m_goalYaw = null;
    private NetworkTableEntry m_testTargetYaw = null;
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Tracking");
    private PhotonCamera m_camera;
    private NetworkTableEntry m_pEntry = null;
    private NetworkTableEntry m_dEntry = null;
    private PowerDistribution m_PDH = null;

    private boolean m_isTesting = false;
    private boolean m_isBlueAlliance = true;
    private final int kBLUE_PIPELINE = 2;
    private final int kRED_PIPELINE = 1;
    // set to -1
    private int m_currentPipeline = -1;

    /** Creates a new Tracking. */
    public Tracking() {
        // create the camera
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/creating-photon-camera.html
        // TODO Chester!
        // something like
        m_camera = new PhotonCamera("ballcam");
        m_PDH = new PowerDistribution(Constants.PDH, ModuleType.kRev);
        m_camera.setDriverMode(true);

        // set the appropriate pipeline for the color of the ball based
        // createShuffleBoardTab();

    }

    @Override
    public void periodic() {
        // make sure we are using the appropriate vision pipeline
        // if (DriverStation.getAlliance() != DriverStation.Alliance.Invalid &&
        // m_currentPipeline == -1) {
        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        // m_currentPipeline = kBLUE_PIPELINE;
        // m_camera.setPipelineIndex(kBLUE_PIPELINE);
        // DataLogManager.log("Tracking: We are BLUE alliance.");
        // } else {
        // m_currentPipeline = kRED_PIPELINE;
        // m_camera.setPipelineIndex(kRED_PIPELINE);
        // DataLogManager.log("Tracking: We are RED alliance.");
        // }
        // }

        // TODO remove this
        // if (true)
        // return;
        // This method will be called once per scheduler run
        m_currentYaw.setDouble(RobotContainer.m_drivetrainSubsystem.getGyroHeading()
                .getDegrees());
        m_goalYaw.setDouble(m_currentYaw.getDouble(0) + getHeadingOffset());
        this.setTestTarget(m_testTargetYaw.getDouble(0));

        if (m_isTesting) {
            return;
        }

        PhotonPipelineResult result = m_camera.getLatestResult();

    }

    public double getHeadingOffset() {
        if (m_isTesting) {
            return m_testTargetYaw.getDouble(0) - m_currentYaw.getDouble(0);
        }

        // TODO Chester fix this! this!

        // Get yaw from tracking camera
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/creating-photon-camera.html
        // Use yaw & gyro to calculate target gyro reading
        // return the difference between current gyro reading and target gyro reading
        double yaw = 0;

        // set yaw equal to yaw from photonvision
        // something like
        PhotonPipelineResult result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            yaw = result.getBestTarget().getYaw();
        }

        return yaw;

    }

    // used only for testing
    private Rotation2d m_testTarget = new Rotation2d();

    // d is in degrees
    public void setTestTarget(double degrees) {

        m_testTarget = new Rotation2d(Units.radiansToDegrees(degrees));
    }

    public boolean isTrackingTarget() {
        if (m_isTesting) {
            return true;
        }

        PhotonPipelineResult result = m_camera.getLatestResult();

        return result.hasTargets();
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Tracking.UpdatePIDF();
        c.setName("Update PIDF");
        commands.add(c);

        c = new frc.robot.commands.Tracking.EnableTestMode();
        c.setName("Test Mode");
        commands.add(c);

        m_testTargetYaw = m_shuffleboardTab.add("Test Target Yaw", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 0).withProperties(Map.of("min", -100.0, "max", 100.0)).getEntry();

        m_currentYaw = m_shuffleboardTab.add("Current Yaw", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 1).withProperties(Map.of("min", -100.0, "max", 100.0)).getEntry();

        m_goalYaw = m_shuffleboardTab.add("Goal Yaw", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 2).withProperties(Map.of("min", -100.0, "max", 100.0)).getEntry();
        m_pEntry = m_shuffleboardTab.add("Angle P", 0.1)
                .withSize(1, 1)
                .withPosition(6, 0).getEntry();
        m_dEntry = m_shuffleboardTab.add("Angle D", 0.001)
                .withSize(1, 1)
                .withPosition(6, 1).getEntry();
        m_testTargetYaw.setDouble(45);
    }

    public double getAngleP() {
        return m_pEntry.getDouble(0);
    }

    public double getAngleD() {
        return m_dEntry.getDouble(0);
    }

    public void EnableTestMode() {
        m_isTesting = true;
    }

    public void enableSearchLight() {
        m_PDH.setSwitchableChannel(true);
    }

    public void disableSearchLight() {
        m_PDH.setSwitchableChannel(false);
    }
}
