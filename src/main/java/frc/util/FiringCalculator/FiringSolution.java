// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.FiringCalculator;

/** Stores an optimate distance, pitch and speed */
public class FiringSolution {
    public double m_distance = 0;
    public double m_pitch = 0;
    public double m_speed = 0;

    public FiringSolution() {

    }

    /**
     * Creates and inializes a FiringSolution
     * 
     * @param distance
     * @param pitch
     * @param speed
     */
    public FiringSolution(double distance, double pitch, double speed) {
        m_distance = distance;
        m_pitch = pitch;
        m_speed = speed;
    }
}
