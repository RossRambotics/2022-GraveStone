// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// ID 46 for arm ID 45 for Intake motor

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
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

public class Intake extends SubsystemBase {
    WPI_TalonFX m_rollerMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR_ROLLER, "usb");
    WPI_TalonFX m_extensionMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR_EXTENSION, "usb");

    static public final Intake_extension m_extension = new Intake_extension();
    static public final Intake_roller m_roller = new Intake_roller();

    // Default Intake Extension
    private double m_extensionTargetDegrees = 120;

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control
     * loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
     * units at 100% output
     * 
     * kP kI kD kF Iz PeakOut
     */
    private TalonFX_Gains m_gainsVelocity = new TalonFX_Gains(0.1, 0, 5, -0.05, 300, 1.00);
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Intake");
    private NetworkTableEntry m_pid_kP = null;
    private NetworkTableEntry m_pid_kI = null;
    private NetworkTableEntry m_pid_kD = null;
    private NetworkTableEntry m_pid_kFF = null;
    private NetworkTableEntry m_targetExtensionDegrees = null;

    /** Creates a new Intake. */
    public Intake() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        m_rollerMotor.configFactoryDefault();
        m_extensionMotor.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        m_rollerMotor.configNeutralDeadband(0.001);
        m_extensionMotor.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        m_extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                IntakeConstants.kPIDLoopIdx,
                IntakeConstants.kTimeoutMs);

        // initializes the encorder to zero the starting position
        m_extensionMotor.setSelectedSensorPosition(0);
        m_extensionMotor.setInverted(TalonFXInvertType.Clockwise);

        /* Config the peak and nominal outputs */
        m_rollerMotor.configNominalOutputForward(0, IntakeConstants.kTimeoutMs);
        m_rollerMotor.configNominalOutputReverse(0, IntakeConstants.kTimeoutMs);
        m_rollerMotor.configPeakOutputForward(0.5, IntakeConstants.kTimeoutMs);
        m_rollerMotor.configPeakOutputReverse(-0.5, IntakeConstants.kTimeoutMs);

        m_extensionMotor.configNominalOutputForward(0, IntakeConstants.kTimeoutMs);
        m_extensionMotor.configNominalOutputReverse(0, IntakeConstants.kTimeoutMs);
        m_extensionMotor.configPeakOutputForward(0.1, IntakeConstants.kTimeoutMs);
        m_extensionMotor.configPeakOutputReverse(-0.1, IntakeConstants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_extensionMotor.config_kF(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kF, IntakeConstants.kTimeoutMs);
        m_extensionMotor.config_kP(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kP, IntakeConstants.kTimeoutMs);
        m_extensionMotor.config_kI(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kI, IntakeConstants.kTimeoutMs);
        m_extensionMotor.config_kD(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kD, IntakeConstants.kTimeoutMs);

        // this.createShuffleBoardTab();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout intakeCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 3)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Intake.StartIntake();
        c.setName("Start Intake");
        // SmartDashboard.putData(c);
        intakeCommands.add(c);

        c = new frc.robot.commands.Intake.StopIntake();
        c.setName("Stop Intake");
        // SmartDashboard.putData(c);
        intakeCommands.add(c);

        c = new frc.robot.commands.Intake.UpdatePIDF();
        c.setName("Update PIDF");
        // SmartDashboard.putData(c);
        intakeCommands.add(c);

        c = new frc.robot.commands.Intake.ReverseIntake();
        c.setName("Reverse Intake");
        // SmartDashboard.putData(c);
        intakeCommands.add(c);

        c = new frc.robot.commands.Intake.RetractIntake();
        c.setName("Retract Arm");
        // SmartDashboard.putData(c);
        intakeCommands.add(c);

        c = new frc.robot.commands.Intake.ExtendIntake();
        c.setName("Extend Arm");
        // SmartDashboard.putData(c);
        intakeCommands.add(c);

        // "Y-axis/Automatic bounds": false,
        // "Y-axis/Upper bound": 7000.0,
        // "Y-axis/Lower bound": 0.0,
        // "Y-axis/Unit": "RPM",
        m_pid_kFF = m_shuffleboardTab.add("Extension PID kFF",
                m_gainsVelocity.kF).withSize(2, 1).withPosition(3, 0).getEntry();
        m_pid_kP = m_shuffleboardTab.add("Extension PID kP",
                m_gainsVelocity.kP).withSize(2, 1).withPosition(3, 1).getEntry();
        m_pid_kD = m_shuffleboardTab.add("Extension PID kD",
                m_gainsVelocity.kD).withSize(2, 1).withPosition(3, 2).getEntry();
        m_pid_kI = m_shuffleboardTab.add("Extension PID kI",
                m_gainsVelocity.kI).withSize(2, 1).withPosition(3, 3).getEntry();

        m_targetExtensionDegrees = m_shuffleboardTab.add("Extension Target", m_extensionTargetDegrees).withSize(1, 1)
                .withPosition(2, 3).getEntry();

    }

    class IntakeConstants {
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

        // Number of ticks required to extend the intake
        public static final double m_defaultExtensionDegrees = 120;
        public static final double m_ticksPerDegree = 2048 * 16 / 360;
    }

    public void start() {
        // Start intake
        // System.out.println("Starting Intake");
        m_rollerMotor.set(TalonFXControlMode.PercentOutput, 0.25);
        // m_rollerMotor.setVoltage(2);
    }

    public void stop() {
        // Stop intake
        m_rollerMotor.set(TalonFXControlMode.PercentOutput, 0);
        // m_rollerMotor.setVoltage(0);

    }

    public void reverse() {
        // Reverse intake
        m_rollerMotor.set(TalonFXControlMode.PercentOutput, -0.25);
    }

    public void extend() {
        // Extends arm
        double targetTicks = IntakeConstants.m_ticksPerDegree * m_targetExtensionDegrees.getDouble(120);

        m_extensionMotor.set(TalonFXControlMode.Position, targetTicks);
    }

    public void retract() {
        // Retracts arm
        m_extensionMotor.set(TalonFXControlMode.Position, 0);
        // Sets back to initial position
    }

    public void updatePIDF() {
        // Get PIDF values from the dashboard
        m_gainsVelocity.kF = m_pid_kFF.getDouble(0);
        m_gainsVelocity.kP = m_pid_kP.getDouble(0);
        m_gainsVelocity.kD = m_pid_kD.getDouble(0);
        m_gainsVelocity.kI = m_pid_kI.getDouble(0);

        /* Config the Velocity closed loop gains in slot0 */
        m_extensionMotor.config_kF(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kF, IntakeConstants.kTimeoutMs);
        m_extensionMotor.config_kP(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kP, IntakeConstants.kTimeoutMs);
        m_extensionMotor.config_kI(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kI, IntakeConstants.kTimeoutMs);
        m_extensionMotor.config_kD(IntakeConstants.kPIDLoopIdx, m_gainsVelocity.kD, IntakeConstants.kTimeoutMs);
    }

    /**
     * Dummy classes use for requirements...
     * You can use AddRequirements(Intake_roller) if you need the roller motor
     * (wheels) and
     * AddRequirements(Intake_extension) if you need the extension motor (arm)
     */
    static private class Intake_roller extends SubsystemBase {
    }

    static private class Intake_extension extends SubsystemBase {
    }
}
