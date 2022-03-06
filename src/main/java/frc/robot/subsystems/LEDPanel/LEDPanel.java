// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDPanel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDPanel.Letters.Phrases;

public class LEDPanel extends SubsystemBase {
  private AddressableLED m_led = null;
  private AddressableLEDBuffer m_ledBuffer = null;
  private final int kLED_COLUMNS = 32;
  private final int kLED_ROWS = 8;
  // number of LEDs
  private final int m_noLEDs = kLED_ROWS * kLED_COLUMNS;

  /** Creates a new LEDPanel. */
  public LEDPanel() {

    // PWM port 8
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(8);

    // Reuse buffer
    // Default to a length the size of LED Panel, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(m_noLEDs);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_ledBuffer.setRGB(5, 255, 0, 0);
    m_led.setData(m_ledBuffer);
    m_led.start();

    m_timer.start();

    // We are no strangers to love You know the rules and so do I A full commitments
    // what I am thinking of You would not get this from any other guy I just wanna
    // tell you how I am feeling Gotta make you understand Never gonna give you up
    // Never gonna let you down Never gonna run around and desert you Never gonna
    // make you cry Never gonna say goodbye Never gonna tell a lie and hurt you We
    // have known each other for so long Your hearts been aching but you are too shy
    // to say it Inside we both know what has been going on We know the game and we
    // are gonna play it And if you ask me how I am feeling Do not tell me you are
    // too blind to see Never gonna give you up Never gonna let you down Never gonna
    // run around and desert you Never gonna make you cry Never gonna say goodbye
    // Never gonna tell a lie and hurt you Never gonna give you up Never gonna let
    // you down Never gonna run around and desert you Never gonna make you cry Never
    // gonna say goodbye Never gonna tell a lie and hurt you Never gonna give never
    // gonna give We have known each other for so long Your hearts been aching but
    // you are too shy to say it Inside we both know what has been going on We know
    // the game and we are gonna play it I just wanna tell you how I am feeling
    // Gotta make you understand Never gonna give you up Never gonna let you down
    // Never gonna run around and desert you Never gonna make you cry Never gonna
    // say goodbye Never gonna tell a lie and hurt you Never gonna give you up Never
    // gonna let you down Never gonna run around and desert you Never gonna make you
    // cry Never gonna say goodbye Never gonna tell a lie and hurt you Never gonna
    // give you up Never gonna let you down Never gonna run around and desert you
    // Never gonna make you cry Never gonna say goodbye

    // NASA Battelle
    // Intuitive Foundation
    // P&G
    // Lawson Products
    // Watson Gravel
    // Triple D Performance and Repair
    // Cincinnati Radiator
    // Butler Rural Community Connection
    // JP Wood Studio LLC
    // Schuler Family

    phraseVals[0] = "  Thanks to Our Sponsers!";
    phraseVals[1] = "  NASA Battelle";
    phraseVals[2] = "  Watson Gravel";
    phraseVals[3] = "  Intuitive Foundation";
    phraseVals[4] = "  Cincinnati Radiator";
    phraseVals[5] = "  Procter and Gamble";
    phraseVals[6] = "  Lawson Products";
    phraseVals[7] = "  Triple D Performance and Repair";
    phraseVals[8] = "  Butler Rural Community Connection";
    phraseVals[9] = "  JP Wood Studio LLC";
    phraseVals[10] = "  Schuler Family";
    phraseVals[11] = "  Ross Local Schools";
    phraseVals[12] = "  Happy Birthday to Carrie!";
    phraseVals[13] = "  And Happy Saint Patricks Day!";

    pColors[0] = Color.kYellow;
    pColors[1] = Color.kBlue;
    pColors[2] = Color.kWhite;
    pColors[3] = Color.kBlue;
    pColors[4] = Color.kRed;
    pColors[5] = Color.kBlue;
    pColors[6] = Color.kYellow;
    pColors[7] = Color.kBlue;
    pColors[8] = Color.kGreen;
    pColors[9] = Color.kBrown;
    pColors[10] = Color.kOrange;
    pColors[11] = Color.kMaroon;
    pColors[12] = Color.kPurple;
    pColors[13] = Color.kGreen;

    p = new Phrases(phraseVals[0].toUpperCase());
    p.phraseColor = pColors[0];

  }

  private int m_index = 0;
  private int phraseIndex = 0;
  Phrases p = null;
  private Timer m_timer = new Timer();
  String phraseVals[] = new String[14];
  Color pColors[] = new Color[14];

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // only do this every x seconds
    // if (m_timer.advanceIfElapsed(0.1) == false) {
    // return;
    // }
    if (m_timer.advanceIfElapsed(0.1) == false) {
      return;
    }

    // m_ledBuffer.setRGB(m_index, 255, 0, 0);

    // System.out.println("Phrase Index = " + phraseIndex);

    // String phraseVals[] = new String[4];
    // phraseVals[0] = "__FIRST_Phrase";
    // phraseVals[1] = "__SECOND_Phrase";
    // phraseVals[2] = "__THIRD_PHRASE";

    // Color pColors[] = new Color[4];
    // pColors[0] = Color.kRed;
    // pColors[1] = Color.kGreen;
    // pColors[2] = Color.kBlue;

    // Phrases p = new Phrases("__THIS_IS_A_DIFFERENT_TEST");
    // System.out.print("My is val is" + phraseVals[0].toUpperCase());

    // Phrases p = new Phrases(phraseVals[phraseIndex].toUpperCase());
    // p.phraseColor = pColors[phraseIndex];
    // p.phraseColor = Color.kBlue;
    Color color[][] = p.getColors();

    m_index++;
    if (m_index >= p.size() * 7) {
      m_index = 0;

      phraseIndex++;
      if (phraseIndex >= phraseVals.length) {
        phraseIndex = 0;
      }
      p = new Phrases(phraseVals[phraseIndex].toUpperCase());
      p.phraseColor = pColors[phraseIndex];
    }

    int col = 0;
    int row = 0;
    int i = 0;

    // Make sure that we don't write out more than the number of columns on the LED
    // Panel
    // Use the smaller of the number of columns in the phrase or the number of the
    // columns left in the phrase
    int columnsToUpdate = kLED_COLUMNS;
    int columnsInPhrase = p.size() * 7;
    if (columnsInPhrase - m_index < kLED_COLUMNS) {
      columnsToUpdate = columnsInPhrase - m_index;
    }

    for (col = 0; col < columnsToUpdate; col++) {
      for (row = 0; row < 8; row++) {
        i = (col * 8) + row;
        if (col % 2 == 0) {
          m_ledBuffer.setLED(i, color[col + m_index][row]);
        } else {
          m_ledBuffer.setLED(i, color[col + m_index][7 - row]);
        }
      }
    }

    m_led.setData(m_ledBuffer);

    // phraseIndex++;
    // if (phraseIndex >= phraseVals.length) {
    // phraseIndex = 0;
    // }
    // p = new Phrases(phraseVals[phraseIndex].toUpperCase());
    // p.phraseColor = pColors[phraseIndex];

  }

  public void print(Phrases phrase) {

  }
}