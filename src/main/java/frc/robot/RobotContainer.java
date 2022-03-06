// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveWhileTracking;
import frc.robot.commands.Turret.TrackTarget;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Tracking;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDStrip.LEDStrip;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {

    private static RobotContainer m_theRobot = null;

    public static void setTheRobot(RobotContainer r) {
        m_theRobot = r;
    }

    public static RobotContainer getTheRobot() {
        return m_theRobot;
    }

    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Auto");

    static public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    static public final Shooter m_Shooter = new Shooter();
    static public final Tracking m_Tracking = new Tracking();
    static public final Turret m_Turret = new Turret();
    // public final Turret m_Turret = null;
    static public final Targeting m_Targeting = new Targeting();
    static public final Intake m_Intake = new Intake();
    static public final Indexer m_Indexer = new Indexer();
    static public final LEDStrip m_LEDStrip = new LEDStrip();

    private final XboxController m_controllerDriver = new XboxController(0);
    // private final XboxController m_controllerOperator = new XboxController(1);

    public PhysicsSim m_PhysicsSim;

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -modifyAxis(
                        getInputLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(
                        getInputLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(
                        getInputRightX())
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        // Configure the button bindings
        configureButtonBindings();

        // Configure auton shuffleboard panel
        // createShuffleBoardTab();

        // disable Live Window per recommendations by WPILIB team to reduce network
        // overhead
        // remove this line if stuff is missing from shuffleboard that we need.
        // LiveWindow.disableAllTelemetry();
    }

    private SlewRateLimiter m_slewLeftY = new SlewRateLimiter(2.0);

    private double getInputLeftY() {
        double driverLeftY = m_controllerDriver.getLeftY();
        // double operatorLeftY = m_controllerOperator.getLeftY() / m_weakPower;
        double operatorLeftY = 0;
        double leftY = operatorLeftY;
        if (Math.abs(leftY) < 0.01) {
            leftY = driverLeftY;
        }
        return m_slewLeftY.calculate(leftY);

    }

    private SlewRateLimiter m_slewLeftX = new SlewRateLimiter(2.0);

    private double getInputLeftX() {
        double driverLeftX = m_controllerDriver.getLeftX();
        // double operatorLeftX = m_controllerOperator.getLeftX() / m_weakPower;
        double operatorLeftX = 0;
        double leftX = operatorLeftX;
        if (Math.abs(leftX) < 0.01) {
            leftX = driverLeftX;
        }
        return m_slewLeftX.calculate(leftX);
    }

    private SlewRateLimiter m_slewRightX = new SlewRateLimiter(2.0);

    private double getInputRightX() {
        double driverRightX = m_controllerDriver.getRightX();
        // double operatorRightX = m_controllerOperator.getRightX() / m_weakPower;
        double operatorRightX = 0;
        double rightX = operatorRightX;
        if (Math.abs(rightX) < 0.01) {
            rightX = driverRightX;
        }
        // return m_slewRightX.calculate(rightX);
        return rightX;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new Button(m_controllerDriver::getBackButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

        // map button for tracking cargo
        // create tracking cargo drive command
        CommandBase cmd = new DriveWhileTracking(m_drivetrainSubsystem,
                () -> -modifyAxis(
                        getInputLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(
                        getInputLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(
                        getInputLeftX())
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        new Button(m_controllerDriver::getLeftBumper)
                .whenHeld(cmd, true);

        // extends the intake and turns on the intake wheels Driver
        // new Button(m_controllerDriver::getAButton)
        // .whenReleased(command);

        // // retracts the intake and turns off the intake wheels
        // new Button(m_controllerDriver::getBButton)
        // .whenPressed(command);

        // // shotes into the lower hub at low RPM Driver
        // new Button(m_controllerDriver::getYButton)
        // .whenPressed(command);

        // // targets to target
        // new Button(m_controllerDriver::getRightBumperPressed)
        // .whileHeld(command);

        // // climb goes up operator
        // new Button(m_controllerOperator::getAButtonPressed)
        // .whileHeld(command);

        // // climb goes down operator
        // new Button(m_controllerOperator::getYButton)
        // .whenPressed(command);

        // // aim at the target
        // new Button(m_controllerDriver::getRightTriggerAxis)
        // .whileHeld(command);

        // how long move turrent is
        var spinTurret = 1;
        var angleTurret = 1;

        // // turret go up
        // POVButton operatorTurretUp = new POVButton(m_controllerOperator, 0);
        // operatorTurretUp.whenPressed(new DefaultDriveCommand(

        // .withTimeout(angleTurret));

        // // turret go down
        // POVButton operatorTurretDown = new POVButton(m_controllerOperator, 180);
        // operatorTurretDown.whenPressed(new DefaultDriveCommand(

        // .withTimeout(angleTurret));

        // // turret go right
        // POVButton operatorTurretRight = new POVButton(m_controllerOperator, 90);
        // operatorTurretRight.whenPressed(new DefaultDriveCommand(

        // .withTimeout(spinTurret));

        // // turret go up
        // POVButton operatorTurretleft = new POVButton(m_controllerOperator, 0);
        // operatorTurretleft.whenPressed(new DefaultDriveCommand(

        // .withTimeout(spinTurret));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        CommandBase cmd = (CommandBase) m_autoChooser.getSelected();
        if (cmd == null) {
            return new InstantCommand();
        }
        return cmd;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    private SendableChooser m_autoChooser = new SendableChooser();

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 1)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("BBL", 3, 1);

        TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                Math.PI, Math.PI);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                4, 0, 0, kThetaControllerConstraints);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_drivetrainSubsystem.zeroGyroscope();

        // use this to automatically set
        // the robot position on the field to match the start of the trajectory
        PathPlannerState start = examplePath.getInitialState();
        m_drivetrainSubsystem.getOdometry().resetPosition(start.poseMeters,
                m_drivetrainSubsystem.getGyroscopeRotation());

        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
                examplePath,
                m_drivetrainSubsystem::getOdometryPose,
                m_drivetrainSubsystem.getKinematics(),
                // Position controllers
                new PIDController(0.2, 0, 0),
                new PIDController(0.2, 0, 0),
                thetaController,
                m_drivetrainSubsystem::setSwerveModulesStates,
                m_drivetrainSubsystem);
        command.setName("Example Path");
        commands.add(command);

        // Add auto routines
        CommandBase autoCmd = null;
        autoCmd = new InstantCommand();
        autoCmd.setName("Do Nothing");
        m_autoChooser.setDefaultOption("Do Nothing", autoCmd);

        autoCmd = new InstantCommand();
        autoCmd.setName("Drive");
        m_autoChooser.addOption("Drive", autoCmd);

        autoCmd = new InstantCommand();
        autoCmd.setName("Drive And Shoot");
        m_autoChooser.addOption("Drive And Shoot", autoCmd);
        tab.add("Autonomous", m_autoChooser).withSize(2, 1);

        this.m_Shooter.createShuffleBoardTab();
        this.m_Tracking.createShuffleBoardTab();
        this.m_Indexer.createShuffleBoardTab();
        this.m_Intake.createShuffleBoardTab();
        this.m_Turret.createShuffleBoardTab();

        // m_Turret.setDefaultCommand(new TrackTarget());

    }
}
