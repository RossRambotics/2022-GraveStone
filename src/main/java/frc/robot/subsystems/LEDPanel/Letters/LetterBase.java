// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDPanel.Letters;

/** Add your docs here. */
public abstract class LetterBase {
    protected int m_leds[][];

    public void print() {
        System.out.println("Printing LEDs:");
        for (int y = 0; y < m_leds.length; y++) {
            for (int x = 0; x < m_leds.length; x++) {
                System.out.print(m_leds[y][x]);
            }
            System.out.println("");
        }
    }

    public int[][] getLEDs() {
        return m_leds;
    }

}
