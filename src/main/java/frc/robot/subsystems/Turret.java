// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.TalonFX_Gains;

public class Turret extends SubsystemBase {
    private int m_kTimeoutMs = 30;
    private int m_kPIDLoopIdx = 0;
    private NetworkTableEntry m_testMode = null;
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Turret");

    // yaw related member variables
    private final double kYAW_TICKS_PER_DEGREE = (54 * 2048.0) / 360; // TODO update with gear
    private final double kTURRET_LOCK_SEARCH_START = 70; // angle the turret moves to start search for lock sensor
    private final double kTURRET_LOCK_SEARCH_END = 120; // maximum angle the turret will look for the lock sensor
    private final double kTURRET_LOCK_SEARCH_SPEED = 0.1; // the speed the turret will move while searching for the
                                                          // correct angle

    public AnalogInput m_Sensor_TurretLock = new AnalogInput(Constants.TURRET_SENSOR);

    private double m_currentYaw = 0; // updated by calaculating it back from the turret motor encoder
    private double m_goalYaw = 0;
    private double m_testTargetYaw = 0;

    private TalonFX_Gains m_yawGains = new TalonFX_Gains(1.2, 0, 0.0, 0, 300, 1.00);
    private double m_yaw_pid_kP = m_yawGains.kP;
    private double m_yaw_pid_kI = m_yawGains.kI;
    private double m_yaw_pid_kD = m_yawGains.kD;
    private double m_yaw_pid_kFF = m_yawGains.kF;

    WPI_TalonFX m_yawMotor = new WPI_TalonFX(Constants.TURRET_MOTOR, "usb");

    // pitch related member variables
    // private final double kPITCH_TICKS_PER_DEGREE = (20 * 2048.0) / 360.0; //
    // 113.77777
    private final double kPITCH_TICKS_PER_DEGREE = 113.7777 * 16 / 10; // we switched from 10:1 gearbox to 16:1
                                                                       // 182.04432
    private double m_currentPitch = 0; // updated by calaculating it back from the turret motor encoder
    private double m_goalPitch = 0;
    private double m_testTargetPitch = 0;
    private NetworkTableEntry m_tuning_percent_pitch = null;
    private NetworkTableEntry m_tuning_offset_yaw = null;

    private TalonFX_Gains m_pitchGains = new TalonFX_Gains(1.75, 0, 2.0, 0, 300, 1.00);
    private double m_pitch_pid_kP = m_pitchGains.kP;
    private double m_pitch_pid_kI = m_pitchGains.kI;
    private double m_pitch_pid_kD = m_pitchGains.kD;
    private double m_pitch_pid_kFF = m_pitchGains.kF;
    private double m_pitchError = 0;
    private double m_yawError = 0;
    private boolean m_isTurretLocked = false;
    private boolean m_isOnTarget = false;

    WPI_TalonFX m_pitchMotor = new WPI_TalonFX(Constants.ANGULAR_MOTOR, "usb");

    /** Creates a new Turret. */
    public Turret() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        m_yawMotor.configFactoryDefault();
        m_pitchMotor.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        m_yawMotor.configNeutralDeadband(0.001);
        m_pitchMotor.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Position] */
        m_yawMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                m_kPIDLoopIdx,
                m_kTimeoutMs);
        m_pitchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                m_kPIDLoopIdx,
                m_kTimeoutMs);

        // Set motor directions
        m_yawMotor.setInverted(TalonFXInvertType.Clockwise);
        m_pitchMotor.setInverted(TalonFXInvertType.CounterClockwise);
        m_pitchMotor.setNeutralMode(NeutralMode.Brake);

        /* Config the peak and nominal outputs */
        m_yawMotor.configNominalOutputForward(0, m_kTimeoutMs);
        m_yawMotor.configNominalOutputReverse(0, m_kTimeoutMs);
        m_yawMotor.configPeakOutputForward(0.1, m_kTimeoutMs);
        m_yawMotor.configPeakOutputReverse(-0.1, m_kTimeoutMs);
        m_pitchMotor.configNominalOutputForward(0, m_kTimeoutMs);
        m_pitchMotor.configNominalOutputReverse(0, m_kTimeoutMs);
        m_pitchMotor.configPeakOutputForward(0.1, m_kTimeoutMs);
        m_pitchMotor.configPeakOutputReverse(-0.1, m_kTimeoutMs);

        /* Config the position closed loop gains in slot0 */
        m_yawMotor.config_kF(m_kPIDLoopIdx, m_yawGains.kF, m_kTimeoutMs);
        m_yawMotor.config_kP(m_kPIDLoopIdx, m_yawGains.kP, m_kTimeoutMs);
        m_yawMotor.config_kI(m_kPIDLoopIdx, m_yawGains.kI, m_kTimeoutMs);
        m_yawMotor.config_kD(m_kPIDLoopIdx, m_yawGains.kD, m_kTimeoutMs);
        m_pitchMotor.config_kF(m_kPIDLoopIdx, m_pitchGains.kF, m_kTimeoutMs);
        m_pitchMotor.config_kP(m_kPIDLoopIdx, m_pitchGains.kP, m_kTimeoutMs);
        m_pitchMotor.config_kI(m_kPIDLoopIdx, m_pitchGains.kI, m_kTimeoutMs);
        m_pitchMotor.config_kD(m_kPIDLoopIdx, m_pitchGains.kD, m_kTimeoutMs);

        // setup shuffleboard
        // createShuffleBoardTab();
    }

    public boolean getIsOnTarget() {
        return m_isOnTarget;
    }

    public void setPitchToGoal() {
        setPitchDegrees(m_goalPitch);
    }

    public void setYawToGoal() {
        setYawDegreesFront(m_goalYaw);
    }

    @Override
    public void periodic() {
        // update angles
        m_currentYaw = m_yawMotor.getSelectedSensorPosition(0) / kYAW_TICKS_PER_DEGREE;
        m_currentPitch = m_pitchMotor.getSelectedSensorPosition(0) / kPITCH_TICKS_PER_DEGREE;
        m_pitchError = m_pitchMotor.getSelectedSensorPosition(0)
                - (m_goalPitch / kPITCH_TICKS_PER_DEGREE);
        m_yawError = m_yawMotor.getSelectedSensorPosition(0)
                - (m_goalYaw / kYAW_TICKS_PER_DEGREE);

        // get the total error
        // double error = Math.abs(m_pitchError.getDouble(0)) +
        // Math.abs(m_yawError.getDouble(0));

        // update targeting error
        if (Math.abs(RobotContainer.m_Targeting.getTargetOffsetYaw()) < 2.0) {
            m_isOnTarget = true;
        } else {
            m_isOnTarget = false;
        }

        // update testing angles if in test mode
        if (m_testMode.getBoolean(false)) {
            RobotContainer.m_Targeting.setTestTargetYaw(m_testTargetYaw
                    - m_currentYaw);

            // this is commented out because the camera's pitch doesn't adjust based on the
            // the pitch of the target. we are using the distance instead
            // RobotContainer.m_Targeting.setTestTargetPitch(m_testTargetPitch.getDouble(0)
            // - m_currentPitch.getDouble(0));
        }

        // *** Taking care of this in the shooter
        // update pitch of turret/shooter based on the distance
        // if (RobotContainer.m_Targeting.isTrackingTarget()) {
        // this.updatePitchUsingDistance();
        // }

        // check if a soft limit is triggered?
        // TODO update soft limits
        m_goalYaw = clampYaw(m_goalYaw);
        m_goalPitch = clampPitch(m_goalPitch);

        // TODO check if hard limit

        // int com.ctre.phoenix.motorcontrol.can.BaseTalon.isFwdLimitSwitchClosed()

    }

    private double clampYaw(double value) {
        return MathUtil.clamp(value, -37.0, 100.0);
    }

    private double clampPitch(double value) {
        return MathUtil.clamp(value, 0.0, 20.0);
    }

    public double getYaw() {
        return m_currentYaw;
    }

    public double getPitch() {
        return m_currentPitch;
    }
    // private void updatePitchUsingDistance() {

    // double distance = RobotContainer.m_Targeting.getTargetDistance();

    // // translate from distance to pitch
    // double pitch = 5; // TODO

    // // make sure pitch is in range

    // m_goalPitch.setDouble(pitch);

    // if (isTurretLocked()) {
    // return;
    // }
    // m_pitchMotor.set(TalonFXControlMode.Position, m_goalPitch.getDouble(0) *
    // kPITCH_TICKS_PER_DEGREE);
    // }

    // sets the yaw of the turret
    // this is relative to the front of the robot
    public void setYawDegreesFront(double goal) {
        // check to make sure the goal is within bounds of the turrent

        // if the goal is outside the bounds, set the goal to the boundary
        goal = clampYaw(goal);
        if (Math.abs(goal - m_currentYaw) < 1.5) {
            m_yawMotor.set(TalonFXControlMode.PercentOutput, 0);

            return;
        }

        m_goalYaw = goal;

        if (isTurretLocked()) {
            return;
        }
        m_yawMotor.set(TalonFXControlMode.Position, m_goalYaw *
                kYAW_TICKS_PER_DEGREE);

        return;
    }

    // returns true if the sensor indicates the turret is in the lock position
    public boolean getSensorTurretLock() {
        if (m_Sensor_TurretLock.getValue() > 10) {
            return false;
        }

        return true;
    }

    // rotates the turret at a relatively slow speed waiting for the sensor to
    // indicate that it is in the lock position
    // returns true if lock position is found
    // returns false if the lock position is not found
    public boolean searchTurretLock() {
        DataLogManager.log("Turret: Turret lock in progress");
        if (getSensorTurretLock()) {
            // lock the turret
            lockTurret();

            return true;
        } else {
            m_yawMotor.set(ControlMode.PercentOutput, kTURRET_LOCK_SEARCH_SPEED);
        }

        // if we are past the search range restart the search
        if (m_currentYaw > kTURRET_LOCK_SEARCH_END) {
            CommandBase cmd = new frc.robot.commands.Turret.LockTurret();
            cmd.schedule();
        }
        return false;
    }

    public void lockTurret() {
        // set the position control to here
        DataLogManager.log("Turret: Locked Turret.");
        m_yawMotor.set(TalonFXControlMode.Position, m_yawMotor.getSelectedSensorPosition(0));

        m_isTurretLocked = true;
    }

    public void unlockTurret() {

        DataLogManager.log("Turret: Turret unlocking");

        // only allow the turret to unlock once the climb has been lowered
        if (RobotContainer.m_Climb.getClimbEncoderPosition() < 1000) {
            DataLogManager.log("Turret: Turret is Unlocked!");
            m_isTurretLocked = false;
        }
    }

    public boolean isTurretLocked() {
        return m_isTurretLocked;
    }

    // moves the turret to the beginning of the turret lock/sensor search range
    // returns true if we are close to search start angle
    // returns false if we are still moving
    public boolean initializeTurretLock() {
        DataLogManager.log("Turret:  Starting turret lock");
        // if we are within 5 degrees true true
        if (Math.abs(this.getYaw() - kTURRET_LOCK_SEARCH_START) < 2) {
            // stop spinning the turret
            m_yawMotor.set(ControlMode.PercentOutput, 0);
            return true;
        }

        m_goalYaw = kTURRET_LOCK_SEARCH_START;
        m_yawMotor.set(TalonFXControlMode.Position, m_goalYaw *
                kYAW_TICKS_PER_DEGREE);

        return false;
    }

    // set the yaw of the turret
    // this method is relative to the current yaw of the turret
    public void setYawDegreesRelative(double goal) {
        // convert to yaw that is relative to the front of the robot
        goal += m_currentYaw;
        goal = clampYaw(goal);
        this.setYawDegreesFront(goal);
    }

    public void setPitchDegrees(double goal) {

        if (Math.abs(goal - m_currentPitch) < 0.5) {
            return;
        }

        m_goalPitch = clampPitch(goal);
        if (isTurretLocked()) {
            return;
        }

        // apply tunning percentage
        goal *= 1.0 + (m_tuning_percent_pitch.getDouble(0) / 100.0);

        m_pitchMotor.set(TalonFXControlMode.Position, goal *
                kPITCH_TICKS_PER_DEGREE);
    }

    public boolean isTestMode() {
        return m_testMode.getBoolean(false);
    }

    public double getTuningYawOffset() {
        return m_tuning_offset_yaw.getDouble(0);
    }

    // set the pitch of the turret
    // this method is relative to the current pitch of the turret
    // public void setPitchDegreesRelative(double goal) {
    // // convert to ptich that is relative to the current position
    // goal += m_currentPitch.getDouble(0);
    // m_goalPitch.setDouble(goal);

    // if (isTurretLocked()) {
    // return;
    // }
    // m_pitchMotor.set(TalonFXControlMode.Position, m_goalPitch.getDouble(0) *
    // kPITCH_TICKS_PER_DEGREE);
    // }

    // this sets that yaw of the turret to a known position
    // intended to be used if a limit switch is hit
    // or to rezero the turret
    public void resetYawDegressAbs(double yaw) {
        // m_currentYaw.setDouble(clampYaw(yaw));

        if (isTurretLocked()) {
            return;
        }

        // update the encoder and set the target to match
        m_yawMotor.setSelectedSensorPosition(m_currentYaw * kYAW_TICKS_PER_DEGREE);
        m_yawMotor.set(TalonFXControlMode.Position, m_goalYaw *
                kYAW_TICKS_PER_DEGREE);
    }

    // this sets that pitch of the turret to a known position
    // intended to be used if a limit switch is hit
    // or to rezero the turret
    public void resetPitchDegressAbs(double pitch) {
        m_currentPitch = pitch;

        if (isTurretLocked()) {
            return;
        }

        // update the encoder and set the target to match
        m_pitchMotor.setSelectedSensorPosition(m_currentPitch * kPITCH_TICKS_PER_DEGREE);
        m_pitchMotor.set(TalonFXControlMode.Position, m_goalPitch *
                kPITCH_TICKS_PER_DEGREE);
    }

    public void simulationInit() {
        RobotContainer.getTheRobot().m_PhysicsSim.getInstance().addTalonFX(m_pitchMotor, 0.75, 20660);
        RobotContainer.getTheRobot().m_PhysicsSim.getInstance().addTalonFX(m_yawMotor, 0.75, 20660);

        RobotContainer.m_Targeting.setTestMode(true);
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 3)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Turret.UpdatePIDF();
        c.setName("Update PIDF");
        commands.add(c);

        c = new frc.robot.commands.Turret.EnableTestMode();
        c.setName("Test Mode");
        commands.add(c);

        c = new frc.robot.commands.Turret.LockTurret();
        c.setName("Lock Turret");
        commands.add(c);

        c = new frc.robot.commands.Turret.UnlockTurret();
        c.setName("Unlock Turret");
        commands.add(c);

        c = new frc.robot.commands.Turret.EnablePitch();
        c.setName("Enable Pitch");
        commands.add(c);

        c = new frc.robot.commands.Turret.EnableYaw();
        c.setName("Enable Yaw");
        commands.add(c);

        c = new frc.robot.commands.Turret.TrackTarget();
        c.setName("Track Target");
        commands.add(c);

        c = new frc.robot.commands.Turret.AcquireFront();
        c.setName("Acquire Front");
        commands.add(c);

        // m_testTargetYaw = m_shuffleboardTab.add("Test Target Yaw",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(2, 0).withProperties(Map.of("min", -100.0, "max",
        // 100.0)).getEntry();

        // m_currentYaw = m_shuffleboardTab.add("Current Yaw",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(2, 1).withProperties(Map.of("min", -100.0, "max",
        // 100.0)).getEntry();

        // m_goalYaw = m_shuffleboardTab.add("Goal Yaw",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(2, 2).withProperties(Map.of("min", -100.0, "max",
        // 100.0)).getEntry();

        // m_testTargetPitch = m_shuffleboardTab.add("Test Target Pitch",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(5, 0).withProperties(Map.of("min", 0, "max", 23.0)).getEntry();

        // m_currentPitch = m_shuffleboardTab.add("Current Pitch",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(5, 1).withProperties(Map.of("min", 0, "max", 23.0)).getEntry();

        // m_goalPitch = m_shuffleboardTab.add("Goal Pitch",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(3, 1)
        // .withPosition(5, 2).withProperties(Map.of("min", 0.0, "max",
        // 23.0)).getEntry();

        m_testMode = m_shuffleboardTab.add("Turret Test Mode",
                false).withSize(2, 1).withPosition(8, 0).getEntry();

        // m_pitchError = m_shuffleboardTab.add("Pitch Error",
        // 0).withSize(1, 1).withPosition(8, 1).getEntry();

        // m_yawError = m_shuffleboardTab.add("Yaw Error",
        // 0).withSize(1, 1).withPosition(8, 2).getEntry();

        // m_yaw_pid_kFF = m_shuffleboardTab.add("Yaw PID kFF",
        // m_yawGains.kF).withSize(1, 1).withPosition(3, 4).getEntry();
        // m_yaw_pid_kP = m_shuffleboardTab.add("Yaw PID kP",
        // m_yawGains.kP).withSize(1, 1).withPosition(3, 3).getEntry();
        // m_yaw_pid_kD = m_shuffleboardTab.add("Yaw PID kD",
        // m_yawGains.kD).withSize(1, 1).withPosition(4, 3).getEntry();
        // m_yaw_pid_kI = m_shuffleboardTab.add("Yaw PID kI",
        // m_yawGains.kI).withSize(1, 1).withPosition(2, 3).getEntry();

        // m_pitch_pid_kFF = m_shuffleboardTab.add("Pitch PID kFF",
        // m_pitchGains.kF).withSize(1, 1).withPosition(6, 4).getEntry();
        // m_pitch_pid_kP = m_shuffleboardTab.add("Pitch PID kP",
        // m_pitchGains.kP).withSize(1, 1).withPosition(6, 3).getEntry();
        // m_pitch_pid_kD = m_shuffleboardTab.add("Pitch PID kD",
        // m_pitchGains.kD).withSize(1, 1).withPosition(7, 3).getEntry();
        // m_pitch_pid_kI = m_shuffleboardTab.add("Pitch PID kI",
        // m_pitchGains.kI).withSize(1, 1).withPosition(5, 3).getEntry();

        resetYawDegressAbs(0);

        // Add tuning for during Match
        // Pitch Tuning
        m_tuning_percent_pitch = RobotContainer.m_TuningTab.add("Tune Pitch %", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 1).withProperties(Map.of("min", -100, "max", 100)).getEntry();

        m_tuning_offset_yaw = RobotContainer.m_TuningTab.add("Tune Offset Yaw", -6.5)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 2).withProperties(Map.of("min", -10, "max", 10)).getEntry();

    }

    public void updatePIDF() {
        // Get PIDF values from the dashboard
        // m_yawGains.kF = m_yaw_pid_kFF.getDouble(0);
        // m_yawGains.kP = m_yaw_pid_kP.getDouble(0);
        // m_yawGains.kD = m_yaw_pid_kD.getDouble(0);
        // m_yawGains.kI = m_yaw_pid_kI.getDouble(0);

        /* Config the position closed loop gains in slot0 */
        m_yawMotor.config_kF(m_kPIDLoopIdx, m_yawGains.kF, m_kTimeoutMs);
        m_yawMotor.config_kP(m_kPIDLoopIdx, m_yawGains.kP, m_kTimeoutMs);
        m_yawMotor.config_kI(m_kPIDLoopIdx, m_yawGains.kI, m_kTimeoutMs);
        m_yawMotor.config_kD(m_kPIDLoopIdx, m_yawGains.kD, m_kTimeoutMs);

        // Get PIDF values from the dashboard
        // m_pitchGains.kF = m_pitch_pid_kFF.getDouble(0);
        // m_pitchGains.kP = m_pitch_pid_kP.getDouble(0);
        // m_pitchGains.kD = m_pitch_pid_kD.getDouble(0);
        // m_pitchGains.kI = m_pitch_pid_kI.getDouble(0);

        /* Config the position closed loop gains in slot0 */
        m_pitchMotor.config_kF(m_kPIDLoopIdx, m_pitchGains.kF, m_kTimeoutMs);
        m_pitchMotor.config_kP(m_kPIDLoopIdx, m_pitchGains.kP, m_kTimeoutMs);
        m_pitchMotor.config_kI(m_kPIDLoopIdx, m_pitchGains.kI, m_kTimeoutMs);
        m_pitchMotor.config_kD(m_kPIDLoopIdx, m_pitchGains.kD, m_kTimeoutMs);
    }

    public void EnableTestMode() {
        m_testMode.setBoolean(true);
        RobotContainer.m_Targeting.setTestMode(true);
    }

}
