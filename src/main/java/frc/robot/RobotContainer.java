// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Climb.DefaultClimb;
import frc.robot.commands.Drive.SnapDrive;
import frc.robot.commands.Drive.SnapDriveToCargo;
import frc.robot.commands.Intake.IntakeCargo;
import frc.robot.commands.Intake.ReverseIntake;
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
    static public final LEDPanel m_LEDPanel = new LEDPanel();
    private static double slewLimit = 0.6;

    private final XboxController m_controllerDriver = new XboxController(0);
    private final XboxController m_controllerOperator = new XboxController(1);

    public PhysicsSim m_PhysicsSim;

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        m_drivetrainSubsystem.setDefaultCommand(new SnapDrive(m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> snapAngle()));

        // Climb Default Command
        m_Climb.setDefaultCommand(new DefaultClimb(m_Climb, () -> -getOperatorRightY()));

        // Configure the button bindings
        configureButtonBindings();

        // disable Live Window per recommendations by WPILIB team to reduce network
        // overhead
        // remove this line if stuff is missing from shuffleboard that we need.
        // LiveWindow.disableAllTelemetry();
    }

    private SlewRateLimiter m_slewLeftY = new SlewRateLimiter(1.5);

    private double getInputLeftY() {
        double driverLeftY = modifyAxis(m_controllerDriver.getLeftY());

        double slew = m_slewLeftY.calculate(driverLeftY) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

        return slew * slewLimit;

    }

    private SlewRateLimiter m_slewLeftX = new SlewRateLimiter(1.5);

    private double getInputLeftX() {
        double driverLeftX = modifyAxis(m_controllerDriver.getLeftX());

        double slew = m_slewLeftX.calculate(driverLeftX) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        return slew * slewLimit;
    }

    private double getOperatorRightY() {
        double operatorRightY = 0;

        // implement Joystick Deadzone
        if (Math.abs(m_controllerOperator.getRightY()) > 0.08) {
            operatorRightY = m_controllerOperator.getRightY();

        }

        return operatorRightY;
    }

    private double m_lastSnapAngle = 720; // defaults to 720 because 720 tells snap drive to not adjust the angle

    private double snapAngle() {
        double x = m_controllerDriver.getRightX();
        double y = -m_controllerDriver.getRightY();

        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
            // return 720;
            // stop angle drift
            return m_lastSnapAngle;
        }

        double angle = Math.toDegrees(Math.atan2(x, y));

        if ((angle > 360) || (angle < -360)) {
            return 360;
        }

        m_lastSnapAngle = angle;
        return angle;
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
        CommandBase cmd = new ParallelCommandGroup(
                new SnapDriveToCargo(m_drivetrainSubsystem,
                        () -> -getInputLeftY(),
                        () -> -getInputLeftX(),
                        new Rotation2d()),
                new IntakeCargo());
        cmd.setName("SnapDriveToCargo");
        new Button(m_controllerDriver::getLeftBumper)
                .whenHeld(cmd, true);

        cmd = new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> {
                    return -0.2;
                });
        new POVButton(m_controllerDriver, 180)
                .whenHeld(cmd);

        cmd = new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> {
                    return 0.2;
                });
        new POVButton(m_controllerDriver, 90)
                .whenHeld(cmd);

        // extends the intake and turns on the intake wheels Driver
        cmd = new IntakeCargo();

        cmd.setName("A Button - Intake Cargo");

        new Button(m_controllerDriver::getAButton)
                .toggleWhenPressed(cmd);

        // // reverse the intake
        cmd = new ReverseIntake();
        new Button(m_controllerDriver::getBButton)
                .whenHeld(cmd);

        // // shootes into the lower hub at low RPM Driver
        new Button(m_controllerDriver::getYButton)
                .whenPressed(new frc.robot.commands.Shooter.ShootLow()
                        .withTimeout(2.6));

        // shoots into the high goal from hub
        new Button(m_controllerDriver::getXButton)
                .whenPressed(new frc.robot.commands.Shooter.ShootHighFromHub()
                        .withTimeout(5.0));

        // // shoot
        new Button(m_controllerDriver::getRightBumperPressed)
                .whenPressed(new frc.robot.commands.Shooter.ShootHigh()
                        .withTimeout(5));

        /**
         * Operator controls
         */

        // assign lock turret
        new Button(m_controllerOperator::getYButton)
                .whenPressed(new frc.robot.commands.Turret.LockTurret());

        // assign unlock turret
        new Button(m_controllerOperator::getXButton)
                .whenPressed(new frc.robot.commands.Turret.UnlockTurret());

        new Button(m_controllerOperator::getAButton)
                .whenPressed(new frc.robot.commands.Intake.ResetIntake());

        if (m_controllerDriver.getRightStickButton() == true) {
            slewLimit = 1;
        } else {
            slewLimit = 0.6;
        }

        /**
         * Implement Snap Drive
         */

        // North
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                0);

        new POVButton(m_controllerOperator, 0)
                .whenHeld(cmd);

        // South
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                180);

        new POVButton(m_controllerOperator, 180)
                .whenHeld(cmd);

        // East
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                90);

        new POVButton(m_controllerOperator, 90)
                .whenHeld(cmd);

        // West
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                270);

        new POVButton(m_controllerOperator, 270)
                .whenHeld(cmd);

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
        value = deadband(value, 0.1);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    private SendableChooser m_autoChooser = new SendableChooser();

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 1)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase cmd = new frc.robot.commands.auto.TestShort();
        commands.add(cmd);

        cmd = new frc.robot.commands.auto.HubShotBackShot();
        commands.add(cmd);

        cmd = new frc.robot.commands.auto2.Start3LowBackHigh();
        commands.add(cmd);

        cmd = new frc.robot.commands.auto2.Start3LowBackHighHub();
        commands.add(cmd);

        cmd = new frc.robot.commands.auto2.Start3_2LowBackHighHub();
        commands.add(cmd);

        cmd = new frc.robot.commands.auto3.S3_Low_C3_High();
        commands.add(cmd);

        cmd = new frc.robot.commands.Drive.SnapDriveToCargo(
                RobotContainer.m_drivetrainSubsystem,
                new Rotation2d(0));
        commands.add(cmd);

        // Add auto routines
        CommandBase autoCmd = null;
        autoCmd = new InstantCommand();
        autoCmd.setName("Do Nothing");
        m_autoChooser.addOption("Do Nothing", autoCmd);

        autoCmd = new frc.robot.commands.auto.HubShotBackShot();
        autoCmd.setName("Auto Hub Shot");
        m_autoChooser.setDefaultOption("Hub Shot", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S3_C3_High();
        autoCmd.setName("S3_C3_High");
        m_autoChooser.addOption("S3_C3_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S3_Low_C3_High();
        autoCmd.setName("S3_Low_C3_High");
        m_autoChooser.addOption("S3_Low_C3_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S1_C1_High();
        autoCmd.setName("S1_C1_High");
        m_autoChooser.addOption("S1_C1_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S2_C1_W1_High();
        autoCmd.setName("S2_C1_W1_High");
        m_autoChooser.addOption("S2_C1_W1_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S3_W4_High();
        autoCmd.setName("S3_W4_High");
        m_autoChooser.addOption("S3_W4_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S2_W2_High();
        autoCmd.setName("S2_W2_High");
        m_autoChooser.addOption("S2_W2_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S1_W2_High();
        autoCmd.setName("S1_W2_High");
        m_autoChooser.addOption("S1_W2_High", autoCmd);

        autoCmd = new frc.robot.commands.auto3.S3_C2_High();
        autoCmd.setName("S3_C2_High");
        m_autoChooser.addOption("S3_C2_High", autoCmd);

        tab.add("Autonomous", m_autoChooser).withSize(2, 1);

        this.m_Shooter.createShuffleBoardTab();
        this.m_Tracking.createShuffleBoardTab();
        this.m_Indexer.createShuffleBoardTab();
        this.m_Intake.createShuffleBoardTab();
        this.m_Turret.createShuffleBoardTab();
        this.m_Targeting.createShuffleBoardTab();

        // m_Turret.setDefaultCommand(new TrackTarget());
        cmd = new frc.robot.commands.Intake.DefCommand();

        cmd.setName("Default RetractIntake Cmd");
        cmd.addRequirements(m_Intake);
        m_Intake.setDefaultCommand(cmd);

        cmd = new frc.robot.commands.Indexer.DefaultIndexer();
        m_Indexer.setDefaultCommand(cmd);

        DataLogManager.start();
        DataLogManager.log("Log Started.");
    }
}
