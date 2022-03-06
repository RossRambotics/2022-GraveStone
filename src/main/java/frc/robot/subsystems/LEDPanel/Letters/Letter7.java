// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDPanel.Letters;

/** Add your docs here. */
public class Letter7 extends LetterBase {
    public Letter7() {
        m_leds = new int[][] {
                { 1, 1, 1, 1, 1, 1 },
                { 1, 1, 0, 0, 1, 1 },
                { 0, 0, 0, 0, 1, 1 },
                { 0, 0, 0, 1, 1, 0 },
                { 0, 0, 1, 1, 0, 0 },
                { 0, 0, 1, 1, 0, 0 },

        };
    }
}
