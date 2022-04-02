// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// ID 41 for Indexer front (5 sets of green wheels) ID 42 for Indexer back (3 green & 1 gray)

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonFX_Gains;

public class Indexer extends SubsystemBase {

    WPI_TalonFX m_frontwheels = new WPI_TalonFX(Constants.INDEXER_MOTOR_FRONT, "usb");
    WPI_TalonFX m_backwheels = new WPI_TalonFX(Constants.INDEXER_MOTOR_BACK, "usb");
    private TalonFX_Gains m_gainsVelocity = new TalonFX_Gains(0.1, 0, 5, 1023.0 / 20660.0, 300, 1.00);

    public AnalogInput m_Sensor_IndexerEntry = new AnalogInput(Constants.INDEXER_ENTRY);
    public AnalogInput m_Sensor_IndexerMiddle = new AnalogInput(Constants.INDEXER_MIDDLE);
    public AnalogInput m_Sensor_IndexerExit = new AnalogInput(Constants.INDEXER_EXIT);

    private double m_wheelSpeed = 3000.0;
    private double m_diffWheelSpeed = 0.0;

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control
     * loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
     * units at 100% output
     * 
     * kP kI kD kF Iz PeakOut
     */
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Indexer");

    /** Creates a new Intake. */
    public Indexer() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        m_frontwheels.configFactoryDefault();
        m_backwheels.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        m_frontwheels.configNeutralDeadband(0.001);
        m_backwheels.configNeutralDeadband(0.001);

        /* Config the peak and nominal outputs */
        m_frontwheels.configNominalOutputForward(0, IndexerConstants.kTimeoutMs);
        m_frontwheels.configNominalOutputReverse(0, IndexerConstants.kTimeoutMs);
        m_frontwheels.configPeakOutputForward(1, IndexerConstants.kTimeoutMs);
        m_frontwheels.configPeakOutputReverse(-1, IndexerConstants.kTimeoutMs);

        m_backwheels.configNominalOutputForward(0, IndexerConstants.kTimeoutMs);
        m_backwheels.configNominalOutputReverse(0, IndexerConstants.kTimeoutMs);
        m_backwheels.configPeakOutputForward(1, IndexerConstants.kTimeoutMs);
        m_backwheels.configPeakOutputReverse(-1, IndexerConstants.kTimeoutMs);

        m_frontwheels.setInverted(TalonFXInvertType.Clockwise);
        m_backwheels.setInverted(TalonFXInvertType.CounterClockwise);

        /* Config the Velocity closed loop gains in slot0 */
        m_frontwheels.config_kF(0, m_gainsVelocity.kF, 30);
        m_frontwheels.config_kP(0, m_gainsVelocity.kP, 30);
        m_frontwheels.config_kI(0, m_gainsVelocity.kI, 30);
        m_frontwheels.config_kD(0, m_gainsVelocity.kD, 30);

        m_backwheels.config_kF(0, m_gainsVelocity.kF, 30);
        m_backwheels.config_kP(0, m_gainsVelocity.kP, 30);
        m_backwheels.config_kI(0, m_gainsVelocity.kI, 30);
        m_backwheels.config_kD(0, m_gainsVelocity.kD, 30);

    }

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control
     * loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
     * units at 100% output
     * 
     * kP kI kD kF Iz PeakOut
     */

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_diffWheelSpeed = Math.abs(m_frontwheels.getSelectedSensorVelocity(0)) - Math.abs(m_wheelSpeed);
    }

    public boolean getSensorIndexerEntry() {
        if (m_Sensor_IndexerEntry.getValue() < 10) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getSensorIndexerMiddle() {
        if (m_Sensor_IndexerMiddle.getValue() < 10) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getSensorIndexerExit() {
        if (m_Sensor_IndexerExit.getValue() < 10) {
            return true;
        } else {
            return false;
        }
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout indexerCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Indexer.ShootCargo();
        c.setName("Shoot Speed");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

        c = new frc.robot.commands.Indexer.StartWheels();
        c.setName("Start Indexer");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

        c = new frc.robot.commands.Indexer.StopWheels();
        c.setName("Stop Indexer");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

        c = new frc.robot.commands.Indexer.ReverseWheels();
        c.setName("Reverse Indexer");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

        // m_wheelSpeed = m_shuffleboardTab.add("Indexer Shoot RPM",
        // 2500).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(4, 1)
        // .withPosition(2, 0).withProperties(Map.of("min", 0, "max", 7000)).getEntry();

        // m_diffWheelSpeed = m_shuffleboardTab.add("Indexer Error RPM",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(4, 1)
        // .withPosition(2, 1).withProperties(Map.of("min", -100, "max",
        // 100)).getEntry();

    }

    public double getIndexError() {
        return m_diffWheelSpeed;
    }

    class IndexerConstants {
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

    }

    public void start() {
        // Start intake
        double p = 0.25;

        m_frontwheels.set(TalonFXControlMode.PercentOutput, p);
        m_backwheels.set(TalonFXControlMode.PercentOutput, p);
    }

    public void slow() {
        // Start intake
        double p = 0.15;

        m_frontwheels.set(TalonFXControlMode.PercentOutput, p);
        m_backwheels.set(TalonFXControlMode.PercentOutput, p);
    }

    public void shoot() {
        /**
         * Convert RPM to units / 100ms.
         * 2048 Units/Rev * RPM / 600 100ms/min in either direction:
         * velocity setpoint is in units/100ms
         */
        double targetVelocity_UnitsPer100ms = m_wheelSpeed * 2048.0 / 600.0;
        /* 2000 RPM in either direction */

        m_frontwheels.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
        m_backwheels.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

        // m_frontwheels.setVoltage(5.0);
        // m_backwheels.setVoltage(5.0);
    }

    // public void shoot() {
    // // Start intake
    // double p = 0.35;

    // m_frontwheels.set(TalonFXControlMode.PercentOutput, p);
    // m_backwheels.set(TalonFXControlMode.PercentOutput, -p);
    // }

    public void stop() {
        // Stop intake
        m_frontwheels.set(TalonFXControlMode.PercentOutput, 0);
        m_backwheels.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void reverse() {
        // Reverse intake
        m_frontwheels.set(TalonFXControlMode.PercentOutput, -0.25);
        m_backwheels.set(TalonFXControlMode.PercentOutput, -0.25);
    }

    public double getfrountwheelrpm() {
        return m_frontwheels.getSelectedSensorVelocity();

    }

}
