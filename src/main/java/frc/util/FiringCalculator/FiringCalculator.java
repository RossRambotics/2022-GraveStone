// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.FiringCalculator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Takes a series of firing solution and performs linear interpolation between
 * them to provide a Firing Solution at any distance
 */
public class FiringCalculator {
    private FiringSolution m_defaultFiringSolution = new FiringSolution(0.0, 10.0, 500);
    private boolean m_sorted = false;

    private List<FiringSolution> m_data = new ArrayList<FiringSolution>();

    public void addSolution(FiringSolution f) {
        m_data.add(f);
        m_sorted = false;
    }

    private void sort() {
        Collections.sort(m_data, new FiringSolutionSortByDistance());
    }

    public void setDefaultFiringSolution(FiringSolution f) {
        m_defaultFiringSolution = f;
    }

    /**
     * Returns a FiringSolution with the pitch and distance interpolated. If a
     * FiringSolution can't be found return the default firing solution.
     * 
     * @param distance
     * @return
     */
    public FiringSolution compute(double distance) {
        if (!m_sorted) {
            this.sort();
            m_sorted = true;
        }

        // make sure we have at least 2 data points
        if (m_data.size() < 2) {
            return m_defaultFiringSolution;
        }

        // make sure that the distance is in range
        // and handle short special cases
        if (distance < m_data.get(0).m_distance) {
            return m_defaultFiringSolution;
        } else if (m_data.get(0).m_distance == distance) {
            return m_data.get(0);
        }

        // find the 2 closest firing solutions that the distance is between
        // once found interpolate between them
        FiringSolution low, high;
        for (int c = 1; c < m_data.size(); c++) {
            low = m_data.get(c - 1);
            high = m_data.get(c);

            if (low.m_distance < distance && high.m_distance >= distance) {
                return interpolate(distance, low, high);
            }
        }

        return m_defaultFiringSolution;
    }

    private FiringSolution interpolate(double distance, FiringSolution low, FiringSolution high) {
        double deltaDistance = high.m_distance - low.m_distance;
        double deltaPitch = high.m_pitch - low.m_pitch;
        double deltaSpeed = high.m_speed - low.m_speed;

        double slopePitch = deltaPitch / deltaDistance;
        double slopeSpeed = deltaSpeed / deltaDistance;

        FiringSolution answer = new FiringSolution();
        answer.m_distance = distance;
        answer.m_pitch = low.m_pitch + ((distance - low.m_distance) * slopePitch);
        answer.m_speed = low.m_speed + ((distance - low.m_distance) * slopeSpeed);

        return answer;
    }
}
