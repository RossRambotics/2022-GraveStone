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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BottomLeftNoHumanPlayer extends CommandBase {
    /** Creates a new BottomLeftNoHumanPlayer. */

    public BottomLeftNoHumanPlayer() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PathPlannerTrajectory path1 = PathPlanner.loadPath("Test1", 2, 1);
        PathPlannerTrajectory path2 = PathPlanner.loadPath("Test2", 2, 1);

        TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                Math.PI, Math.PI);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                4, 0, 0, kThetaControllerConstraints);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // RobotContainer.m_drivetrainSubsystem.resetOdometry();

        // use this to automatically set
        // the robot position on the field to match the start of the trajectory
        PathPlannerState start = path1.getInitialState();
        RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(start.poseMeters,
                RobotContainer.m_drivetrainSubsystem.getGyroscopeRotation());

        PPSwerveControllerCommand command1 = new PPSwerveControllerCommand(
                path1,
                RobotContainer.m_drivetrainSubsystem::getOdometryPose,
                RobotContainer.m_drivetrainSubsystem.getKinematics(),
                // Position controllers
                new PIDController(0.2, 0, 0),
                new PIDController(0.2, 0, 0),
                thetaController,
                RobotContainer.m_drivetrainSubsystem::setSwerveModulesStates,
                RobotContainer.m_drivetrainSubsystem);

        PPSwerveControllerCommand command2 = new PPSwerveControllerCommand(
                path2,
                RobotContainer.m_drivetrainSubsystem::getOdometryPose,
                RobotContainer.m_drivetrainSubsystem.getKinematics(),
                // Position controllers
                new PIDController(0.2, 0, 0),
                new PIDController(0.2, 0, 0),
                thetaController,
                RobotContainer.m_drivetrainSubsystem::setSwerveModulesStates,
                RobotContainer.m_drivetrainSubsystem);

        CommandBase cmd = new SequentialCommandGroup(command1, command2);

        cmd.schedule();

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
