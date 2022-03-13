// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {

    WPI_TalonFX m_ClimbWinch = new WPI_TalonFX(Constants.CLIMB_WINCH, "usb");
    TalonFXSensorCollection m_ClimbEncoder;

    double defaultPowerDown;
    double defaultPowerUp;
    double m_MaxClimbPosition = 506000;
    double m_MinClimbPosition = -m_MaxClimbPosition;

    /** Creates a new ExampleSubsystem. */
    public Climb() {
        m_ClimbEncoder = m_ClimbWinch.getSensorCollection();
        m_ClimbWinch.setInverted(TalonFXInvertType.CounterClockwise);

        m_ClimbWinch.setSelectedSensorPosition(0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setClimbPower(double Power) {
        // only move climb if the turret is locked

        if (RobotContainer.m_Turret.isTurretLocked()) {
            m_ClimbWinch.set(TalonFXControlMode.PercentOutput, Power);
        }
    }

    public double getClimbEncoderPosition() {
        return -m_ClimbEncoder.getIntegratedSensorPosition();
    }

    public double getMaxClimbPosition() {
        return m_MaxClimbPosition;
    }

    public double getMinClimbPosition() {
        return m_MinClimbPosition;
    }
}
