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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveWhileTracking;
import frc.robot.commands.Climb.DefaultClimb;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Turret.TrackTarget;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RioLEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Tracking;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDPanel.LEDPanel;
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

    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Match.Auto");
    static public final ShuffleboardTab m_TuningTab = Shuffleboard.getTab("Match.Tuning");

    static public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    static public final Shooter m_Shooter = new Shooter();
    static public final Tracking m_Tracking = new Tracking();
    static public final Turret m_Turret = new Turret();

    static public final Climb m_Climb = new Climb();

    static public final Targeting m_Targeting = new Targeting();
    static public final Intake m_Intake = new Intake();
    static public final Indexer m_Indexer = new Indexer();
    static public final RioLEDs m_RioLEDs = new RioLEDs();

    static public final LEDStrip m_LEDStrip = new LEDStrip();
    // static public final LEDPanel m_LEDPanel = new LEDPanel();

    private final XboxController m_controllerDriver = new XboxController(0);
    private final XboxController m_controllerOperator = new XboxController(1);

    public PhysicsSim m_PhysicsSim;

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> -modifyAxis(
                        getInputRightX())
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));


        //Climb Default Command
        m_Climb.setDefaultCommand(new DefaultClimb(m_Climb, () -> getOperatorRightY()));

        // Configure the button bindings
        configureButtonBindings();

        // Configure auton shuffleboard panel
        // createShuffleBoardTab();

        // disable Live Window per recommendations by WPILIB team to reduce network
        // overhead
        // remove this line if stuff is missing from shuffleboard that we need.
        // LiveWindow.disableAllTelemetry();
    }

    private SlewRateLimiter m_slewLeftY = new SlewRateLimiter(0.6);

    private double getInputLeftY() {
        double kDEAD_SLEW = 0.2;
        double driverLeftY = modifyAxis(m_controllerDriver.getLeftY()
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        // double operatorLeftY = m_controllerOperator.getLeftY() / m_weakPower;
        double operatorLeftY = 0.0;
        double leftY = operatorLeftY;
        if (Math.abs(leftY) < 0.01) {
            leftY = driverLeftY;
        }

        double slew = m_slewLeftY.calculate(leftY);
        if (Math.abs(slew) < kDEAD_SLEW) {
            if (driverLeftY == 0) {
                slew = 0.0;
            } else if (driverLeftY > 0) {
                slew = kDEAD_SLEW;
            } else {
                slew = -kDEAD_SLEW;
            }
            m_slewLeftY.reset(slew);
        }

        return slew;

    }

    private SlewRateLimiter m_slewLeftX = new SlewRateLimiter(0.6);

    private double getInputLeftX() {
        double kDEAD_SLEW = 0.2;
        double driverLeftX = modifyAxis(
                m_controllerDriver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        // double operatorLeftX = m_controllerOperator.getLeftX() / m_weakPower;
        double operatorLeftX = 0.0;
        double leftX = operatorLeftX;
        if (Math.abs(leftX) < 0.01) {
            leftX = driverLeftX;
        }

        double slew = m_slewLeftX.calculate(leftX);
        if (Math.abs(slew) < kDEAD_SLEW) {
            if (driverLeftX == 0) {
                slew = 0.0;
            } else if (driverLeftX > 0) {
                slew = kDEAD_SLEW;
            } else {
                slew = -kDEAD_SLEW;
            }
            m_slewLeftX.reset(slew);
        }
        return slew;
    }

    private SlewRateLimiter m_slewRightX = new SlewRateLimiter(0.6);

    private double getInputRightX() {
        double kDEAD_SLEW = 0.2;
        double driverRightX = modifyAxis(
                m_controllerDriver.getRightX()
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        // double operatorRightX = m_controllerOperator.getRightX() / m_weakPower;
        double operatorRightX = 0.0;
        double rightX = operatorRightX;
        if (Math.abs(rightX) < 0.01) {
            rightX = driverRightX;
        }
        double slew = m_slewRightX.calculate(rightX);
        if (Math.abs(slew) < kDEAD_SLEW) {
            if (driverRightX == 0) {
                slew = 0.0;
            } else if (driverRightX > 0) {
                slew = kDEAD_SLEW;
            } else {
                slew = -kDEAD_SLEW;
            }
            m_slewRightX.reset(slew);
        }
        return slew;
        // return rightX;
    }

    private double getOperatorRightY(){
        double operatorRightY = 0;

        //implement Joystick Deadzone
        if (Math.abs(m_controllerOperator.getRightY()) > 0.01){
            operatorRightY = m_controllerOperator.getRightY();

        }
        System.out.println("Running Climb Default:" + operatorRightY);
        return operatorRightY;
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
        cmd = new ParallelCommandGroup(
                new ExtendIntake(),
                new StartIntake());

        cmd.setName("A Button - Intake Cargo");

        new Button(m_controllerDriver::getAButton)
                .whenHeld(cmd);

        // // reverse the intake
        new Button(m_controllerDriver::getBButton)
                .whenHeld(new ReverseIntake());

        // // shootes into the lower hub at low RPM Driver
        new Button(m_controllerDriver::getYButton)
                .whenPressed(new frc.robot.commands.Shooter.ShootLow()
                        .withTimeout(20.0));

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
        this.m_Targeting.createShuffleBoardTab();

        // m_Turret.setDefaultCommand(new TrackTarget());
        CommandBase cmd = new frc.robot.commands.Intake.DefCommand();

        cmd.setName("Default RetractIntake Cmd");
        cmd.addRequirements(m_Intake);
        m_Intake.setDefaultCommand(cmd);

        cmd = new frc.robot.commands.Indexer.EmptyCheck();
        m_Indexer.setDefaultCommand(cmd);

        DataLogManager.start();
        DataLogManager.log("Log Started.");

    }
}
