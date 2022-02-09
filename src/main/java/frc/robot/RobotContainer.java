// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Tracking;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class RobotContainer {
  private static RobotContainer m_theRobot = null;

  public static void setTheRobot(RobotContainer r) {
    m_theRobot = r;
  }

  public static RobotContainer getTheRobot() {
    return m_theRobot;
  }

  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // public final Shooter m_Shooter = new Shooter();
  public final Tracking m_Tracking = new Tracking();
  // public final Turret m_Turret = new Turret();
  public final Turret m_Turret = null;
  public final Targeting m_Targeting = new Targeting();

  private final XboxController m_controller = new XboxController(0);

  public PhysicsSim m_PhysicsSim;

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // m_Turret.setDefaultCommand(new TrackTarget(m_Turret));

    // Configure the button bindings
    configureButtonBindings();

    // disable Live Window per recommendations by WPILIB team to reduce network
    // overhead
    // remove this line if stuff is missing from shuffleboard that we need.
    // LiveWindow.disableAllTelemetry();
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
    new Button(m_controller::getBackButton)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // map button for tracking cargo
    // create tracking cargo drive command
    CommandBase cmd = new DriveWhileTracking(m_drivetrainSubsystem,
        () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    new Button(m_controller::getLeftBumperPressed)
        .whenHeld(cmd, true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
}
