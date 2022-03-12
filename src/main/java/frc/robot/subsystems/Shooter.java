// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.util.FiringCalculator.FiringCalculator;
import frc.util.FiringCalculator.FiringSolution;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
    WPI_TalonFX m_frontMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_FRONT, "usb");
    WPI_TalonFX m_backMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_BACK, "usb");

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control
     * loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
     * units at 100% output
     * 
     * kP kI kD kF Iz PeakOut
     */
    private TalonFX_Gains m_gainsVelocity = new TalonFX_Gains(0.1, 0, 5, 1023.0 / 20660.0, 300, 1.00);
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Shooter");
    private NetworkTableEntry m_goalRPM = null;
    private NetworkTableEntry m_actualFrontRPM = null;
    private NetworkTableEntry m_actualBackRPM = null;
    private NetworkTableEntry m_diffFrontRPM = null;
    private NetworkTableEntry m_diffBackRPM = null;
    private NetworkTableEntry m_pid_kP = null;
    private NetworkTableEntry m_pid_kI = null;
    private NetworkTableEntry m_pid_kD = null;
    private NetworkTableEntry m_pid_kFF = null;
    private NetworkTableEntry m_nt_distance = null;
    private NetworkTableEntry m_nt_rpmreturn = null;
    private NetworkTableEntry m_spinPercent = null;
    private NetworkTableEntry m_tuning_percent_RPM = null;

    private double m_FrontRPM_shooter = 0;
    private double m_BackRPM_shooter = 0;
    private double m_distance = 0;
    private double m_interpolated_RPM = 0;

    private FiringCalculator m_firingCalculator = new FiringCalculator();
    private boolean m_bTestMode = false;

    public boolean isTestMode() {
        return m_bTestMode;
    }

    public void setTestMode(boolean m_bTestMode) {
        this.m_bTestMode = m_bTestMode;
    }

    /** Creates a new Shooter. */
    public Shooter() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        m_frontMotor.configFactoryDefault();
        m_backMotor.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        m_frontMotor.configNeutralDeadband(0.001);
        m_backMotor.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        m_frontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                ShooterConstants.kPIDLoopIdx,
                ShooterConstants.kTimeoutMs);
        m_backMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                ShooterConstants.kPIDLoopIdx,
                ShooterConstants.kTimeoutMs);

        // Reverse the back motor
        m_backMotor.setInverted(TalonFXInvertType.CounterClockwise);
        m_frontMotor.setInverted(TalonFXInvertType.Clockwise);

        /* Config the peak and nominal outputs */
        m_frontMotor.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
        m_frontMotor.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
        m_frontMotor.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
        m_frontMotor.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

        m_backMotor.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
        m_backMotor.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
        m_backMotor.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
        m_backMotor.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_frontMotor.config_kF(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kF, ShooterConstants.kTimeoutMs);
        m_frontMotor.config_kP(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kP, ShooterConstants.kTimeoutMs);
        m_frontMotor.config_kI(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kI, ShooterConstants.kTimeoutMs);
        m_frontMotor.config_kD(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kD, ShooterConstants.kTimeoutMs);

        m_backMotor.config_kF(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kF, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kP(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kP, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kI(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kI, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kD(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kD, ShooterConstants.kTimeoutMs);

        m_firingCalculator.addSolution(new FiringSolution(1, 7.5, 1700));
        m_firingCalculator.addSolution(new FiringSolution(2, 11.26, 1706));
        m_firingCalculator.addSolution(new FiringSolution(3, 13.14, 2069));
        m_firingCalculator.addSolution(new FiringSolution(4, 15, 2337));
        m_firingCalculator.addSolution(new FiringSolution(5, 16.8, 2500));

        // m_firingCalculator.addSolution(new FiringSolution(6, , 1700));
        // m_firingCalculator.addSolution(new FiringSolution(7, 7.5, 1700));
        // m_firingCalculator.addSolution(new FiringSolution(8, 7.5, 1700));

        // this.createShuffleBoardTab();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Get the RPM of the motors
        m_FrontRPM_shooter = Math.abs(m_frontMotor.getSelectedSensorVelocity(ShooterConstants.kPIDLoopIdx));
        m_FrontRPM_shooter = m_FrontRPM_shooter / 2048 * 600;
        m_actualFrontRPM.setDouble(m_FrontRPM_shooter);
        m_diffFrontRPM.setDouble(m_FrontRPM_shooter - m_goalRPM.getDouble(0));

        m_BackRPM_shooter = Math.abs(m_backMotor.getSelectedSensorVelocity(ShooterConstants.kPIDLoopIdx));
        m_BackRPM_shooter = m_BackRPM_shooter / 2048 * 600;
        m_actualBackRPM.setDouble(m_BackRPM_shooter);
        m_diffBackRPM.setDouble(m_BackRPM_shooter - m_goalRPM.getDouble(0));

        // if we are in test mode use the distance provided by shuffleboard
        if (m_bTestMode) {
            m_distance = m_nt_distance.getDouble(0);
        } else {
            // see if we are targeting object and calculate firing solution
            if (!RobotContainer.m_Targeting.isTrackingTarget()) {
                return;
            }
            m_distance = RobotContainer.m_Targeting.getTargetDistance();
        }

        // only do the following if the shooter has a target and is not in test mode
        FiringSolution fs = m_firingCalculator.compute(m_distance);

        m_nt_rpmreturn.setDouble(fs.m_speed);

        if (!m_bTestMode) {
            m_goalRPM.setDouble(fs.m_speed);

            // update Turret pitch for firing solution unless in test mode
            if (!RobotContainer.m_Turret.isTestMode()) {

                RobotContainer.m_Turret.setPitchDegrees(fs.m_pitch);
            }
        }
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout shooterCommands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Shooter.StartShooter();
        c.setName("Start Shooter");
        // SmartDashboard.putData(c);
        shooterCommands.add(c);

        c = new frc.robot.commands.Shooter.StopShooter();
        c.setName("Stop Shooter");
        // SmartDashboard.putData(c);
        shooterCommands.add(c);

        c = new frc.robot.commands.Shooter.UpdatePIDF();
        c.setName("Update PIDF");
        // SmartDashboard.putData(c);
        shooterCommands.add(c);

        c = new frc.robot.commands.Shooter.EnableTestMode();
        c.setName("Enable Test Mode");
        shooterCommands.add(c);

        m_goalRPM = m_shuffleboardTab.add("Shooter Test RPM", 4000).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 0).withProperties(Map.of("min", 0, "max", 7000)).getEntry();

        m_spinPercent = m_shuffleboardTab.add("Spin Percent", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 1).withProperties(Map.of("min", -100, "max", 100)).getEntry();

        m_actualFrontRPM = m_shuffleboardTab.add("Shooter Front Actual RPM", 4000)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(0, 2).withProperties(Map.of("min", 0, "max", 7000)).getEntry();

        m_diffFrontRPM = m_shuffleboardTab.add("Shooter Front Diff RPM", 4000).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(0, 3).withProperties(Map.of("min", -100, "max", 100)).getEntry();

        m_actualBackRPM = m_shuffleboardTab.add("Shooter Back Actual RPM", 4000)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(4, 2).withProperties(Map.of("min", 0, "max", 7000)).getEntry();

        m_diffBackRPM = m_shuffleboardTab.add("Shooter Back Diff RPM", 4000).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(4, 3).withProperties(Map.of("min", -100, "max", 100)).getEntry();

        // "Y-axis/Automatic bounds": false,
        // "Y-axis/Upper bound": 7000.0,
        // "Y-axis/Lower bound": 0.0,
        // "Y-axis/Unit": "RPM",
        m_nt_distance = m_shuffleboardTab.add("Shooter Distance", m_distance).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(2, 1)
                .withPosition(7, 0).withProperties(Map.of("min", 0, "max", 10)).getEntry();
        m_nt_rpmreturn = m_shuffleboardTab.add("Shooter/RPM Return", 0.0)
                .withSize(1, 1)
                .withPosition(6, 1).getEntry();
        m_pid_kFF = m_shuffleboardTab.add("Shooter PID kFF",
                m_gainsVelocity.kF).withSize(2, 1).withPosition(8, 1).getEntry();
        m_pid_kP = m_shuffleboardTab.add("Shooter PID kP",
                m_gainsVelocity.kP).withSize(2, 1).withPosition(8, 2).getEntry();
        m_pid_kD = m_shuffleboardTab.add("Shooter PID kD",
                m_gainsVelocity.kD).withSize(2, 1).withPosition(8, 3).getEntry();
        m_pid_kI = m_shuffleboardTab.add("Shooter PID kI",
                m_gainsVelocity.kI).withSize(2, 1).withPosition(8, 4).getEntry();

        // Add tuning for during Match
        // Adjust Shooter RPM
        m_tuning_percent_RPM = RobotContainer.m_TuningTab.add("Tune RPM %", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 0).withProperties(Map.of("min", -100, "max", 100)).getEntry();

    }

    // constants
    class ShooterConstants {
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

    public void shootLow() {
        /**
         * Convert RPM to units / 100ms.
         * 2048 Units/Rev * RPM / 600 100ms/min in either direction:
         * velocity setpoint is in units/100ms
         */
        double targetVelocity_UnitsPer100ms = 250 * 2048.0 / 600.0;

        m_frontMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
        m_backMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

    }

    public void start() {
        /**
         * Convert RPM to units / 100ms.
         * 2048 Units/Rev * RPM / 600 100ms/min in either direction:
         * velocity setpoint is in units/100ms
         */
        double targetVelocity_UnitsPer100ms = m_goalRPM.getDouble(0) * 2048.0 / 600.0;

        // apply tunning percentage
        targetVelocity_UnitsPer100ms *= 1.0 + (m_tuning_percent_RPM.getDouble(0) / 100.0);

        /* 2000 RPM in either direction */

        if (m_spinPercent.getDouble(0) == 0) {
            m_frontMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
            m_backMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
        } else {
            double spin = m_spinPercent.getDouble(0);
            m_frontMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms * (100.0 + spin) / 100.0);
            m_backMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms * Math.abs(100.0 - spin) / 100.0);
        }
    }

    public void stop() {
        m_frontMotor.set(TalonFXControlMode.PercentOutput, 0);
        m_backMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void simulationInit() {
        RobotContainer.getTheRobot().m_PhysicsSim.getInstance().addTalonFX(m_frontMotor, 0.75, 20660);
        RobotContainer.getTheRobot().m_PhysicsSim.getInstance().addTalonFX(m_backMotor, 0.75, 20660);
    }

    public void updatePIDF() {
        // Get PIDF values from the dashboard
        m_gainsVelocity.kF = m_pid_kFF.getDouble(0);
        m_gainsVelocity.kP = m_pid_kP.getDouble(0);
        m_gainsVelocity.kD = m_pid_kD.getDouble(0);
        m_gainsVelocity.kI = m_pid_kI.getDouble(0);

        /* Config the Velocity closed loop gains in slot0 */
        m_frontMotor.config_kF(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kF, ShooterConstants.kTimeoutMs);
        m_frontMotor.config_kP(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kP, ShooterConstants.kTimeoutMs);
        m_frontMotor.config_kI(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kI, ShooterConstants.kTimeoutMs);
        m_frontMotor.config_kD(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kD, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kF(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kF, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kP(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kP, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kI(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kI, ShooterConstants.kTimeoutMs);
        m_backMotor.config_kD(ShooterConstants.kPIDLoopIdx, m_gainsVelocity.kD, ShooterConstants.kTimeoutMs);
    }

    public double getRPM() {
        double ave = (m_BackRPM_shooter + m_FrontRPM_shooter) / 2;

        return ave;
    }

    public boolean isSpunUp() {
        double error = m_diffBackRPM.getDouble(0) + m_diffFrontRPM.getDouble(0);

        if (error < 200) {
            return true;
        }

        return false;
    }

}
