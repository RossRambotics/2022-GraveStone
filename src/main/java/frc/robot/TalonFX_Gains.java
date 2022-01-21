// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class TalonFX_Gains {
	public double kP;
	public double kI;
	public double kD;
	public double kF;
	public int kIzone;
	public double kPeakOutput;

	public TalonFX_Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}
