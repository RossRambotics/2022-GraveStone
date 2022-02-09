// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Targeting extends SubsystemBase {
    private double m_testTargetYaw = 0.0;
    private boolean m_isTestMode = false;

    /** Creates a new Targeting. */
    public Targeting() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getTargetOffset() {
        if (m_isTestMode) {
            return m_testTargetYaw;
        }

        // TODO implement this
        return 0;

    }

    public void setTestMode(boolean b) {
        m_isTestMode = b;
    }

    public boolean isTrackingTarget() {
        if (m_isTestMode) {
            return true;
        }

        // TODO implement this
        return false;
    }

    public void setTestTargetYaw(double d) {
        m_testTargetYaw = d;
    }
}
