// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDStrip;

/** Add your docs here. */
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDStrip.Input;

public class LEDStrip extends SubsystemBase {
    private AddressableLED m_led = null;
    private AddressableLEDBuffer m_ledBuffer = null;
    private final int kLED_COLUMNS = 32;
    private final int kLED_ROWS = 1;
    // number of LEDs
    private final int m_noLEDs = kLED_ROWS * kLED_COLUMNS;

    /** Creates a new LEDPanel. */
    public LEDStrip() {

        // PWM port 7
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(7);

        // Reuse buffer
        // Default to a length the size of LED Panel, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(m_noLEDs);
        m_led.setLength(m_ledBuffer.getLength());

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

                m_ledBuffer.setRGB(i, 255, 255, 255);

            } else {

            }

        }

        /*
         * for (var o = 0; o < m_ledBuffer.getLength(); o++) {
         * 
         * if (o == 1) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 3) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 5) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 7) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 9) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 11) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 13) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 15) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 17) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 19) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 21) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 23) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 25) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 27) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 29) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * if (o == 31) {
         * if (hubDetection == "red") {
         * m_ledBuffer.setRGB(o, 255, 0, 0);
         * } else {
         * if (hubDetection == "yellow") {
         * m_ledBuffer.setRGB(o, 255, 255, 0);
         * } else {
         * if (hubDetection == "green") {
         * m_ledBuffer.setRGB(o, 0, 128, 0);
         * } else {
         * m_ledBuffer.setRGB(o, 0, 0, 0);
         * }
         * }
         * }
         * }
         * 
         * }
         */
        m_led.setData(m_ledBuffer);
        m_led.start();

        // m_timer.start();

    }

    public void setBallBlack() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

                m_ledBuffer.setRGB(a, 0, 0, 0);

            } else {

            }

        }

        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setHubRed() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                m_ledBuffer.setRGB(a, 255, 0, 0);
            }

        }

        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setHubYellow() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                m_ledBuffer.setRGB(a, 255, 255, 0);
            }

        }

        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setHubGreen() {
        for (var a = 0; a < m_ledBuffer.getLength(); a++) {
            // Sets the specified LED to the RGB values for red

            if ((a % 2) == 0) {

            } else { // odd leds
                m_ledBuffer.setRGB(a, 0, 128, 0);
            }

        }

        m_led.setData(m_ledBuffer);
        m_led.start();

    }

}
