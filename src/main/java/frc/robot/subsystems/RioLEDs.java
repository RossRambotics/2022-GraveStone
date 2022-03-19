// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.LEDPanel.LEDPanel;
import frc.robot.subsystems.LEDStrip.LEDStrip;

/**
 * This is a little ugly, but the Roborio only supports one LED driver via PWM
 * so this class combines
 * the LED subsystems.
 */
public class RioLEDs {
    static public final AddressableLED m_RioLEDs = new AddressableLED(8); // PWM port 8
    static public final int m_simLEDAdjust = 16 - (LEDStrip.m_noLEDs % 16);
    static public final int m_noLEDs = LEDStrip.m_noLEDs + LEDPanel.m_noLEDs + m_simLEDAdjust;
    static public final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(m_noLEDs);
    static public final int m_LEDStripOffset = 0;
    static public final int m_LEDPanelOffset = m_LEDStripOffset + LEDStrip.m_noLEDs;

    public RioLEDs() {
        m_RioLEDs.setLength(m_noLEDs);
    }

    public void setDataStrip(AddressableLEDBuffer buff) {
        for (int c1 = m_LEDStripOffset, c2 = 0; c2 < LEDStrip.m_noLEDs; c1++, c2++) {
            m_ledBuffer.setLED(c1, buff.getLED(c2));
        }
        m_RioLEDs.setData(m_ledBuffer);
        m_RioLEDs.start();
    }

    public void setDataPanel(AddressableLEDBuffer buff) {
        int firstLED = m_LEDPanelOffset;

        // just move over some pixels so it can be read in the simulator
        if (RobotBase.isSimulation()) {
            firstLED += m_simLEDAdjust;
        }

        for (int c1 = firstLED, c2 = 0; c2 < LEDPanel.m_noLEDs; c1++, c2++) {
            m_ledBuffer.setLED(c1, buff.getLED(c2));
        }
        m_RioLEDs.setData(m_ledBuffer);
        m_RioLEDs.start();
    }
}
