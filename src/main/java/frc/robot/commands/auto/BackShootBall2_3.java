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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.ShootHigh;
import frc.robot.commands.Turret.TrackTarget;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.RobotContainer;

public class BackShootBall2_3 extends CommandBase {
    private DrivetrainSubsystem m_driveSub = null;

    /** Creates a new BottomRightHumanPlayer. */
    public BackShootBall2_3() {
        m_driveSub = RobotContainer.m_drivetrainSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("BackShootBall2_3", 2, 1);

        TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                Math.PI, Math.PI);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                -4, 0, 0, kThetaControllerConstraints);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // m_driveSub.zeroGyroscope();

        // use this to automatically set
        // the robot position on the field to match the start of the trajectory
        PathPlannerState start = examplePath.getInitialState();
        m_driveSub.getOdometry().resetPosition(start.poseMeters,
                m_driveSub.getGyroscopeRotation());

        PPSwerveControllerCommand runPath = new PPSwerveControllerCommand(
                examplePath,
                m_driveSub::getOdometryPose,
                m_driveSub.getKinematics(),
                // Position controllers
                new PIDController(0.2, 0, 0),
                new PIDController(0.2, 0, 0),
                thetaController,
                m_driveSub::setSwerveModulesStates,
                m_driveSub);

        SequentialCommandGroup command = new SequentialCommandGroup(new ExtendIntake().withTimeout(1),
                new StartIntake().withTimeout(1), runPath, new ShootHigh().withTimeout(5),
                new StopIntake().withTimeout(1), new RetractIntake().withTimeout(1));

        // ParallelCommandGroup parallelgroup = new ParallelCommandGroup(command, new
        // TrackTarget());

        command.schedule();

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
