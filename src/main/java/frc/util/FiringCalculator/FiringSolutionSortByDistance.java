// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.FiringCalculator;

import java.util.Comparator;

/** Add your docs here. */
public class FiringSolutionSortByDistance implements Comparator<FiringSolution> {

    @Override
    public int compare(FiringSolution arg0, FiringSolution arg1) {
        FiringSolution one = (FiringSolution) arg0;
        FiringSolution two = (FiringSolution) arg1;

        if (one.m_distance < two.m_distance) {
            return -1;
        } else if (one.m_distance > two.m_distance) {
            return 1;
        }
        return 0;
    }
}
