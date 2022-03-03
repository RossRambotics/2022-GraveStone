// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// ID 41 for Indexer front (5 sets of green wheels) ID 42 for Indexer back (3 green & 1 gray)


import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonFX_Gains;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  WPI_TalonFX m_frontwheels = new WPI_TalonFX(Constants.INDEXER_MOTOR_FRONT);
  WPI_TalonFX m_backwheels = new WPI_TalonFX(Constants.INDEXER_MOTOR_BACK);

  public AnalogInput m_Sensor_IndexerEntry = new AnalogInput(Constants.INDEXER_ENTRY);
  public AnalogInput m_Sensor_IndexerMiddle = new AnalogInput(Constants.INDEXER_MIDDLE);
  public AnalogInput m_Sensor_IndexerExit = new AnalogInput(Constants.INDEXER_EXIT);
  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop.
   * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
   * units at 100% output
   * 
   * kP kI kD kF Iz PeakOut
   */
  private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Intake");

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

    this.createShuffleBoardTab();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void createShuffleBoardTab() {
    ShuffleboardTab tab = m_shuffleboardTab;
    ShuffleboardLayout indexerCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase c = new frc.robot.commands.Indexer.ShootCargo(this);
    c.setName("Start Indexer");
    SmartDashboard.putData(c);
    indexerCommands.add(c);

    c = new frc.robot.commands.Indexer.StopWheels(this);
    c.setName("Stop Indexer");
    SmartDashboard.putData(c);
    indexerCommands.add(c);

    c = new frc.robot.commands.Indexer.ReverseWheels(this);
    c.setName("Reverse Indexer");
    SmartDashboard.putData(c);
    indexerCommands.add(c);

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

  public void shoot() {
    // Start intake
    m_frontwheels.set(TalonFXControlMode.PercentOutput, 0.25);
    m_backwheels.set(TalonFXControlMode.PercentOutput, -0.25);
  }

  public void stop() {
    // Stop intake
    m_frontwheels.set(TalonFXControlMode.PercentOutput, 0);
    m_backwheels.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void reverse() {
    // Reverse intake
    m_frontwheels.set(TalonFXControlMode.PercentOutput, -0.25);
    m_backwheels.set(TalonFXControlMode.PercentOutput, 0.25);
  }

  public boolean getSensorIndexerEntry() {
    if (m_Sensor_IndexerEntry.getValue() > 10) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getSensorIndexerMiddle() {
    if (m_Sensor_IndexerMiddle.getValue() > 10) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getSensorIndexerExit() {
    if (m_Sensor_IndexerExit.getValue() > 10) {
      return true;
    } else {
      return false;
    }
  }
=======
    WPI_TalonFX m_frontwheels = new WPI_TalonFX(Constants.INDEXER_MOTOR_FRONT, "usb");
    WPI_TalonFX m_backwheels = new WPI_TalonFX(Constants.INDEXER_MOTOR_BACK, "usb");

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

        // set orientation
        m_frontwheels.setInverted(TalonFXInvertType.Clockwise);
        m_backwheels.setInverted(TalonFXInvertType.Clockwise);

        this.createShuffleBoardTab();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout indexerCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Indexer.ShootCargo(this);
        c.setName("Start Indexer");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

        c = new frc.robot.commands.Indexer.StopWheels(this);
        c.setName("Stop Indexer");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

        c = new frc.robot.commands.Indexer.ReverseWheels(this);
        c.setName("Reverse Indexer");
        // SmartDashboard.putData(c);
        indexerCommands.add(c);

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

    public void shoot() {
        // Start intake
        double p = 0.25;

        m_frontwheels.set(TalonFXControlMode.PercentOutput, p);
        m_backwheels.set(TalonFXControlMode.PercentOutput, -p);
    }

    public void stop() {
        // Stop intake
        m_frontwheels.set(TalonFXControlMode.PercentOutput, 0);
        m_backwheels.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void reverse() {
        // Reverse intake
        m_frontwheels.set(TalonFXControlMode.PercentOutput, -0.25);
        m_backwheels.set(TalonFXControlMode.PercentOutput, 0.25);
    }


}
