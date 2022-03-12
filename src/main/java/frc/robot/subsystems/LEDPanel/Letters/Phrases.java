// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDPanel.Letters;

import java.util.*;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class Phrases {

    List<LetterBase> phrase;

    public Color phraseColor;

    public Phrases(String p) {

        phrase = LetterFactory.getLetterList(p);

    }

    public void print() {

        ListIterator<LetterBase> itr = phrase.listIterator();
        while (itr.hasNext()) {

            System.out.println("index:" + itr.nextIndex() + "value: ");
            itr.next().print();

        }

    }

    public Color[][] getColors() {
        // Creating an array of colors. The 1st dimension is corresponds to the x-axis
        // on the LED panel.
        // The 2nd dimension corresponds to the y-axis on the LED panel

        Color colors[][] = new Color[phrase.size() * 7][8];

        for (int col = 0; col < phrase.size() * 7; col++) {
            for (int row = 0; row < 8; row++) {
                colors[col][row] = Color.kBlack;
            }

        }

        for (int index = 0; index < phrase.size(); index++) {

            LetterBase l = phrase.get(index);
            int[][] LEDs = l.getLEDs();

            for (int col = 0; col < 6; col++) {
                for (int row = 0; row < 6; row++) {
                    if (LEDs[row][col] == 1) {
                        colors[col + (index * 7)][row] = phraseColor;
                    }
                }
            }
        }

        // test
        // colors[0][0] = Color.kOrange;
        // colors[1][0] = Color.kOrange;
        // colors[2][0] = Color.kOrange;
        // colors[3][0] = Color.kOrange;
        // colors[4][0] = Color.kOrange;
        // colors[1][1] = Color.kOrange;
        // colors[0][1] = Color.kOrange;
        // colors[5][1] = Color.kOrange;
        // colors[4][1] = Color.kOrange;
        // colors[0][2] = Color.kOrange;
        // colors[1][2] = Color.kOrange;
        // colors[2][2] = Color.kOrange;
        // colors[3][2] = Color.kOrange;
        // colors[4][2] = Color.kOrange;
        // colors[0][3] = Color.kOrange;
        // colors[1][3] = Color.kOrange;
        // colors[4][3] = Color.kOrange;
        // colors[5][3] = Color.kOrange;
        // colors[0][4] = Color.kOrange;
        // colors[1][4] = Color.kOrange;
        // colors[5][4] = Color.kOrange;
        // colors[4][4] = Color.kOrange;
        // colors[1][5] = Color.kOrange;
        // colors[0][5] = Color.kOrange;
        // colors[5][5] = Color.kOrange;
        // colors[4][5] = Color.kOrange;
        // colors[2][5] = Color.kOrange;
        // colors[3][5] = Color.kOrange;
        return colors;

    }

    public int size() {
        return phrase.size();
    }
}
