// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    /** Creates a new Targeting. */
    public Targeting() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    @Override
    public void periodic() {
        // get updates from limelight
        int tv = table.getEntry("tv").getNumber(0).intValue();

        // if the target isn't found don't update the values
        if (tv == 1) {
            m_hasTarget = true;
        } else {
            m_hasTarget = false;
            return;
        }

        m_targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0);
        m_targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);
        m_targetArea = table.getEntry("ta").getDouble(0);
        m_targetSkew = table.getEntry("ts").getDouble(0);

        // get the pitch of the target
        // calculate the distance
        m_distanceToTarget = 0.0; // TODO

    }

    public double getTargetDistance() {
        return m_distanceToTarget;
    }

    public double getTargetOffsetYaw() {
        if (m_isTestMode) {
            return m_testTargetYaw;
        }

        // TODO implement this
        // get result from camera

        return m_targetOffsetAngle_Horizontal;
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

        // TODO implement this
        // get result from camera

        return m_hasTarget;
    }

    public void setTestTargetYaw(double d) {
        m_testTargetYaw = d;
    }

    public void setTestTargetPitch(double d) {
        m_testTargetPitch = d;
    }
}
