// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BottomLeftNoHumanPlayer extends CommandBase {
  /** Creates a new BottomLeftNoHumanPlayer. */
  private DrivetrainSubsystem m_driveSub = null;

  public BottomLeftNoHumanPlayer(DrivetrainSubsystem driveSubsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSub = driveSubsytem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("1.0 BBL (no human player)", 3, 1);

    TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        Math.PI, Math.PI);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        4, 0, 0, kThetaControllerConstraints);

    // let's the theta controller know that it is a circle (ie, 180 = -180)
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveSub.zeroGyroscope();

    // use this to automatically set
    // the robot position on the field to match the start of the trajectory
    PathPlannerState start = examplePath.getInitialState();
    m_driveSub.getOdometry().resetPosition(start.poseMeters,
        m_driveSub.getGyroscopeRotation());

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        examplePath,
        m_driveSub::getOdometryPose,
        m_driveSub.getKinematics(),
        // Position controllers
        new PIDController(0.2, 0, 0),
        new PIDController(0.2, 0, 0),
        thetaController,
        m_driveSub::setSwerveModulesStates,
        m_driveSub);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
