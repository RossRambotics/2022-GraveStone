// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDStrip;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDStrip extends SubsystemBase {

    private AddressableLEDBuffer m_ledBuffer = null;
    static private final int kLED_COLUMNS = 125;
    // static private final int kLED_COLUMNS = 2;
    static private final int kLED_ROWS = 1;
    // number of LEDs
    static public final int m_noLEDs = kLED_ROWS * kLED_COLUMNS;

    static public final LEDStrip_targeting m_targeting = new LEDStrip_targeting();
    static public final LEDStrip_tracking m_tracking = new LEDStrip_tracking();

    /** Creates a new LEDPanel. */
    public LEDStrip() {

        // PWM port 7
        // Must be a PWM header, not MXP or DIO
        // Moved to Robot Container
        // m_RioLEDs = new AddressableLED(7);

        // Reuse buffer
        // Default to a length the size of LED Panel, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(m_noLEDs);
    }

    @Override
    public void periodic() {
        // if (getCurrentCommand() == null) {
        // for (int c = 0; c < m_noLEDs; c++) {
        // m_ledBuffer.setRGB(c, 91, 14, 43);
        // }
        // RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
        // }
    }

    public void setBallWhite() {

        // Red: (255, 0, 0) - 1
        // White: (255, 255, 255) - 1
        // Green:(0, 128, 0) - 3
        // Yellow:(255, 255, 0) -2
        // Black: (0, 0, 0) - 0

        // ballDetection(off, white)
        // hubDetection(off, red, yellow, green)

        // Set the data
        // m_ledBuffer.setRGB(5, 255, 0, 0);

        // String ballDetection = "white";
        String hubDetection = "red";

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red

            if ((i % 2) == 0) {

                m_ledBuffer.setRGB(i, 100, 100, 100);

            } else {

            }

        }

        RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
    }

    public void setBallBlack() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

                m_ledBuffer.setRGB(a, 0, 0, 0);

            } else {

            }

        }

        RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
    }

    public void setHubRed() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                // 115, 6, 35
                m_ledBuffer.setRGB(a, 115, 6, 35);
            }

        }

        RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
    }

    public void setHubYellow() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                // 128, 124, 9
                m_ledBuffer.setRGB(a, 128, 124, 9);
            }

        }

        RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
    }

    public void setHubPink() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                m_ledBuffer.setRGB(a, 117, 35, 106);
            }

        }

        RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
    }

    public void setHubGreen() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                     // 2, 94, 31
                m_ledBuffer.setRGB(a, 2, 94, 31);
            }

        }

        RobotContainer.m_RioLEDs.setDataStrip(m_ledBuffer);
    }

    /**
     * Dummy classes use for requirements...
     * You can use AddRequirements(LEDStrip_targeting) if you need the targeting
     * LEDs or
     * AddRequirements(LEDStrip_tracking) if you need the tracking LEDS
     */
    static private class LEDStrip_targeting extends SubsystemBase {
    }

    static private class LEDStrip_tracking extends SubsystemBase {
    }
}
