// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.LEDStrip.HubFound;
import frc.robot.commands.LEDStrip.HubNotFound;
import frc.robot.commands.LEDStrip.HubTargeted;

public class Targeting extends SubsystemBase {
    private double m_testTargetYaw = 0.0;
    private double m_testTargetPitch = 0.0;
    private boolean m_isTestMode = false;
    private NetworkTable m_limelight_nt = null;
    private double m_targetOffsetAngle_Horizontal = 0.0;
    private double m_targetOffsetAngle_Vertical = 0.0;
    private double m_distanceToTarget = 0;
    private boolean m_hasTarget = false;

    private HubFound m_cmdHubFound = new HubFound();
    private HubNotFound m_cmdHubNotFound = new HubNotFound();
    private HubTargeted m_cmdHubTargeted = new HubTargeted();
    private static final double kHEIGHT = 1.87; // the difference in height of the robot to the target ring
    private static final double kMOUNT_PITCH = 32.5; // the angle of the camera mount pitch
    // private ShuffleboardTab m_shuffleboardTab =
    // Shuffleboard.getTab("Sub.Targeting");

    /** Creates a new Targeting. */
    public Targeting() {
        m_limelight_nt = NetworkTableInstance.getDefault().getTable("limelight-rambot");

    }

    public void TakeSnapshot() {
        m_limelight_nt.getEntry("snapshot").setNumber(1);

    }

    public void ResetSnapshot() {
        m_limelight_nt.getEntry("snapshot").setNumber(0);
    }

    @Override
    public void periodic() {
        // get updates from limelight
        int tv = m_limelight_nt.getEntry("tv").getNumber(0).intValue();

        // if the target isn't found don't update the values
        if (RobotContainer.m_Turret.isTurretLocked()) {
            return;
        }

        if (tv == 1) {
            m_hasTarget = true;
        } else {
            m_hasTarget = false;
            m_cmdHubNotFound.schedule();
            return;
        }

        if (RobotContainer.m_Turret.getIsOnTarget()) {
            m_cmdHubTargeted.schedule();
        } else {
            m_cmdHubFound.schedule();
        }

        m_targetOffsetAngle_Horizontal = m_limelight_nt.getEntry("tx").getDouble(0);
        m_targetOffsetAngle_Vertical = m_limelight_nt.getEntry("ty").getDouble(0);

        // get the pitch of the target
        // calculate the distance
        /**
         * tan t = o / a
         * 
         * t: needs to be adjusted to account for the pitch of the camera mount
         * o: is the height difference between the camera lens and the hub ring
         * a: is the distance of the robot from the hub
         */
        m_distanceToTarget = kHEIGHT / Math.tan(Math.toRadians(kMOUNT_PITCH + m_targetOffsetAngle_Vertical));
    }

    public double getTargetDistance() {
        if (m_hasTarget) {
            return m_distanceToTarget;
        }
        return 0.0;
    }

    public double getTargetOffsetYaw() {
        if (m_isTestMode) {
            return m_testTargetYaw;
        }

        // get result from camera
        double tuning = RobotContainer.m_Turret.getTuningYawOffset();
        double base = m_targetOffsetAngle_Horizontal;

        return base + tuning;
    }

    public double getTargetOffsetPitch() {
        if (m_isTestMode) {
            return m_testTargetPitch;
        }

        return m_targetOffsetAngle_Vertical;
    }

    public void setTestMode(boolean b) {
        m_isTestMode = b;
    }

    public boolean isTrackingTarget() {
        if (m_isTestMode) {
            return true;
        }

        return m_hasTarget;
    }

    public void setTestTargetYaw(double d) {
        m_testTargetYaw = d;
    }

    public void setTestTargetPitch(double d) {
        m_testTargetPitch = d;
    }

    public void createShuffleBoardTab() {

        // m_pred_distance = m_shuffleboardTab.add("Distance Look Ahead",
        // 1.0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(2, 0).withProperties(Map.of("min", 0.0, "max",
        // 5.0)).getEntry();

        // m_pred_yaw = m_shuffleboardTab.add("Yaw Look Ahead",
        // 1.0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(2, 1).withProperties(Map.of("min", 0.0, "max",
        // 5.0)).getEntry();

        // m_pred_yaw_distance = m_shuffleboardTab.add("Yaw Look Distance Weight", 0)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(2, 2).withProperties(Map.of("min", 0.0, "max",
        // 100.0)).getEntry();
    }
}
