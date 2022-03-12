// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  WPI_TalonFX m_ClimbWinch = new WPI_TalonFX(Constants.CLIMB_WINCH);

  double defaultPowerDown;
  double defaultPowerUp;

  /** Creates a new ExampleSubsystem. */
  public Climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setClimbPower(double Power) {
    m_ClimbWinch.set(TalonFXControlMode.PercentOutput, Power);
  }
}
