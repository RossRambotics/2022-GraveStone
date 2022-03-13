// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    private double m_distanceToTarget = 0.0;
    private NetworkTable table = null;
    private double m_targetOffsetAngle_Horizontal = 0.0;
    private double m_targetOffsetAngle_Vertical = 0.0;
    private double m_targetArea = 0.0;
    private double m_targetSkew = 0.0;
    private boolean m_hasTarget = false;
    private double[] m_yawHistory = new double[3];
    private double[] m_distanceHistory = new double[3];
    private HubFound m_cmdHubFound = new HubFound();
    private HubNotFound m_cmdHubNotFound = new HubNotFound();
    private HubTargeted m_cmdHubTargeted = new HubTargeted();
    private static final double kHEIGHT = 2.0; // the difference in height of the robot to the target ring
    private static final double kMOUNT_PITCH = 31.4; // the angle of the camera mount pitch
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Targeting");
    private NetworkTableEntry m_pred_distance = null;
    private NetworkTableEntry m_pred_yaw = null;
    private NetworkTableEntry m_pred_yaw_distance = null;

    /** Creates a new Targeting. */
    public Targeting() {
        table = NetworkTableInstance.getDefault().getTable("limelight-rambot");

    }

    @Override
    public void periodic() {
        // get updates from limelight
        int tv = table.getEntry("tv").getNumber(0).intValue();

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

        m_targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0);
        m_targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);
        m_targetArea = table.getEntry("ta").getDouble(0);
        m_targetSkew = table.getEntry("ts").getDouble(0);

        // get the pitch of the target
        // calculate the distance
        /**
         * tan t = o / a
         * 
         * t: needs to be adjusted to account for the pitch of the camera mount
         * o: is the height difference between the camera lens and the hub ring
         * a: is the distance of the robot from the hub
         */
        m_distanceToTarget = kHEIGHT / Math.atan(Math.toRadians(kMOUNT_PITCH + m_targetOffsetAngle_Vertical));

        // save off the history
        for (int c = 0; c < 2; c++) {
            m_distanceHistory[c] = m_distanceHistory[c + 1];
            m_yawHistory[c] = m_yawHistory[c + 1];
        }
        m_distanceHistory[2] = m_distanceToTarget;
        m_yawHistory[2] = m_targetOffsetAngle_Horizontal;

        // save off the history
        for (int c = 0; c < 2; c++) {
            m_distanceHistory[c] = m_distanceHistory[c + 1];
            m_yawHistory[c] = m_yawHistory[c + 1];
        }
        m_distanceHistory[2] = m_distanceToTarget;
        m_yawHistory[2] = m_targetOffsetAngle_Horizontal;

    }

    public double getTargetDistance() {
        if (m_hasTarget) {
            return this.getPredictedDistance(m_pred_distance.getDouble(0.0));
        }
        return 0.0;
    }

    public double getTargetOffsetYaw() {
        if (m_isTestMode) {
            return m_testTargetYaw;
        }

        // get result from camera
        double tuning = RobotContainer.m_Turret.getTuningYawOffset();
        double time = m_pred_yaw.getDouble(0.0)
                + (m_pred_yaw_distance.getDouble(0.0) / 100.0 * this.getTargetDistance());
        double base = this.getPredictedTargetOffsetYaw(time);

        return base + tuning;
    }

    // returns the predicted yaw of the target given a time in seconds into the
    // future
    // so, if the time of flight (TOF) of the cargo is 2s in the future, pass in 2.0
    public double getPredictedTargetOffsetYaw(double time) {
        double v = m_yawHistory[2] - m_yawHistory[1];

        return m_yawHistory[2] + (v * time);
    }

    // returns the predicted distance of the target given a time in seconds into the
    // future
    // so, if the time of flight (TOF) of the cargo is 2s in the future, pass in 2.0
    public double getPredictedDistance(double time) {
        double v = m_distanceHistory[2] - m_distanceHistory[1];

        return m_distanceHistory[2] + (v * time);
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

        m_pred_distance = m_shuffleboardTab.add("Distance Look Ahead", 1.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(3, 1)
                .withPosition(2, 0).withProperties(Map.of("min", 0.0, "max", 5.0)).getEntry();

        m_pred_yaw = m_shuffleboardTab.add("Yaw Look Ahead", 1.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(3, 1)
                .withPosition(2, 1).withProperties(Map.of("min", 0.0, "max", 5.0)).getEntry();

        m_pred_yaw_distance = m_shuffleboardTab.add("Yaw Look Distance Weight", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(3, 1)
                .withPosition(2, 2).withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    }
}
