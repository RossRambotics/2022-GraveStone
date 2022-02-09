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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.TalonFX_Gains;

public class Turret extends SubsystemBase {

    private final double kTICKS_PER_DEGREE = (20 * 2048.0) / 360; // TODO update with gear ratio
    private NetworkTableEntry m_currentYaw = null; // updated by calaculating it back from the turret motor encoder
    private NetworkTableEntry m_goalYaw = null;
    private NetworkTableEntry m_testTargetYaw = null;
    private int m_kTimeoutMs = 30;
    private int m_kPIDLoopIdx = 0;
    // private int m_kSlotIdx = 0;

    private TalonFX_Gains m_gainsVelocity = new TalonFX_Gains(0.05, 0, 0.5, 0, 300, 1.00);
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Turret");
    private NetworkTableEntry m_pid_kP = null;
    private NetworkTableEntry m_pid_kI = null;
    private NetworkTableEntry m_pid_kD = null;
    private NetworkTableEntry m_pid_kFF = null;
    private NetworkTableEntry m_testMode = null;

    WPI_TalonFX m_frontMotor = new WPI_TalonFX(Constants.TURRET_MOTOR);

    /** Creates a new Turret. */
    public Turret() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        m_frontMotor.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        m_frontMotor.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        m_frontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                m_kPIDLoopIdx,
                m_kTimeoutMs);

        // Reverse the back motor
        m_frontMotor.setInverted(TalonFXInvertType.CounterClockwise);

        /* Config the peak and nominal outputs */
        m_frontMotor.configNominalOutputForward(0, m_kTimeoutMs);
        m_frontMotor.configNominalOutputReverse(0, m_kTimeoutMs);
        m_frontMotor.configPeakOutputForward(1, m_kTimeoutMs);
        m_frontMotor.configPeakOutputReverse(-1, m_kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_frontMotor.config_kF(m_kPIDLoopIdx, m_gainsVelocity.kF, m_kTimeoutMs);
        m_frontMotor.config_kP(m_kPIDLoopIdx, m_gainsVelocity.kP, m_kTimeoutMs);
        m_frontMotor.config_kI(m_kPIDLoopIdx, m_gainsVelocity.kI, m_kTimeoutMs);
        m_frontMotor.config_kD(m_kPIDLoopIdx, m_gainsVelocity.kD, m_kTimeoutMs);

        // setup shuffleboard
        createShuffleBoardTab();
    }

    @Override
    public void periodic() {
        m_currentYaw.setDouble(m_frontMotor.getSelectedSensorPosition(0) / kTICKS_PER_DEGREE);

        if (m_testMode.getBoolean(false)) {
            RobotContainer.getTheRobot().m_Targeting.setTestTargetYaw(m_testTargetYaw.getDouble(0)
                    - m_currentYaw.getDouble(0));
        }
        // This method will be called once per scheduler run

        // check if a soft limit is triggered?
        if (m_goalYaw.getDouble(0) < -90.0) {
            m_goalYaw.setDouble(90.0);
        }
        if (m_goalYaw.getDouble(0) > 90.0) {
            m_goalYaw.setDouble(90.0);
        }

        // TODO check if hard limit

        // int com.ctre.phoenix.motorcontrol.can.BaseTalon.isFwdLimitSwitchClosed()

    }

    // sets the yaw of the turret
    // this is relative to the front of the robot
    public void setYawDegreesFront(double goal) {
        // check to make sure the goal is within bounds of the turrent

        // if the goal is outside the bounds, set the goal to the boundary

        m_goalYaw.setDouble(goal);
        m_frontMotor.set(TalonFXControlMode.Position, m_goalYaw.getDouble(0) *
                kTICKS_PER_DEGREE);
    }

    // set the yaw of the turret
    // this method is relative to the current yaw of the turret
    public void setYawDegreesRelative(double goal) {
        // convert to yaw that is relative to the front of the robot
        goal += m_currentYaw.getDouble(0);
        this.setYawDegreesFront(goal);
    }

    // this sets that yaw of the turret to a known position
    // intended to be used if a limit switch is hit
    // or to rezero the turret
    public void resetYawDegressAbs(double yaw) {
        m_currentYaw.setDouble(yaw);
        m_frontMotor.setSelectedSensorPosition(m_currentYaw.getDouble(0) * kTICKS_PER_DEGREE);
    }

    public void simulationInit() {
        RobotContainer.getTheRobot().m_PhysicsSim.getInstance().addTalonFX(m_frontMotor, 0.75, 20660);
        RobotContainer.getTheRobot().m_Targeting.setTestMode(true);
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c = new frc.robot.commands.Turret.UpdatePIDF(this);
        c.setName("Update PIDF");
        commands.add(c);

        c = new frc.robot.commands.Turret.EnableTestMode(this);
        c.setName("Test Mode");
        commands.add(c);

        m_testTargetYaw = m_shuffleboardTab.add("Test Target Yaw", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 0).withProperties(Map.of("min", -100.0, "max", 100.0)).getEntry();

        m_currentYaw = m_shuffleboardTab.add("Current Yaw", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 1).withProperties(Map.of("min", -100.0, "max", 100.0)).getEntry();

        m_goalYaw = m_shuffleboardTab.add("Goal Yaw", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(2, 2).withProperties(Map.of("min", -100.0, "max", 100.0)).getEntry();

        // m_actualBackRPM = m_shuffleboardTab.add("Shooter Back Actual RPM",
        // 4000).withWidget(BuiltInWidgets.kGraph)
        // .withSize(4, 3)
        // .withPosition(4, 2).getEntry();

        // m_diffBackRPM = m_shuffleboardTab.add("Shooter Back Diff RPM",
        // 4000).withWidget(BuiltInWidgets.kGraph)
        // .withSize(4, 3)
        // .withPosition(4, 2).getEntry();

        m_testMode = m_shuffleboardTab.add("Turret Test Mode",
                false).withSize(2, 1).withPosition(6, 0).getEntry();
        m_pid_kFF = m_shuffleboardTab.add("Turret PID kFF",
                m_gainsVelocity.kF).withSize(2, 1).withPosition(6, 1).getEntry();
        m_pid_kP = m_shuffleboardTab.add("Turret PID kP",
                m_gainsVelocity.kP).withSize(2, 1).withPosition(6, 2).getEntry();
        m_pid_kD = m_shuffleboardTab.add("Turret PID kD",
                m_gainsVelocity.kD).withSize(2, 1).withPosition(6, 3).getEntry();
        m_pid_kI = m_shuffleboardTab.add("Turret PID kI",
                m_gainsVelocity.kI).withSize(2, 1).withPosition(6, 4).getEntry();

    }

    public void updatePIDF() {
        // Get PIDF values from the dashboard
        m_gainsVelocity.kF = m_pid_kFF.getDouble(0);
        m_gainsVelocity.kP = m_pid_kP.getDouble(0);
        m_gainsVelocity.kD = m_pid_kD.getDouble(0);
        m_gainsVelocity.kI = m_pid_kI.getDouble(0);

        /* Config the Velocity closed loop gains in slot0 */
        m_frontMotor.config_kF(m_kPIDLoopIdx, m_gainsVelocity.kF, m_kTimeoutMs);
        m_frontMotor.config_kP(m_kPIDLoopIdx, m_gainsVelocity.kP, m_kTimeoutMs);
        m_frontMotor.config_kI(m_kPIDLoopIdx, m_gainsVelocity.kI, m_kTimeoutMs);
        m_frontMotor.config_kD(m_kPIDLoopIdx, m_gainsVelocity.kD, m_kTimeoutMs);
    }

    public void EnableTestMode() {
        m_testMode.setBoolean(true);
        RobotContainer.getTheRobot().m_Targeting.setTestMode(true);
    }

}
