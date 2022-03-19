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
import frc.robot.commands.Shooter.ShootLow;
import frc.robot.commands.Turret.TrackTarget;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.RobotContainer;

public class HubShotBackShot extends CommandBase {
    private DrivetrainSubsystem m_driveSub = null;

    /** Creates a new BottomRightHumanPlayer. */
    public HubShotBackShot() {
        m_driveSub = RobotContainer.m_drivetrainSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        m_driveSub.zeroGyroscope();
        CommandBase cmd = new frc.robot.commands.DefaultDriveCommand(
                RobotContainer.m_drivetrainSubsystem,
                () -> {
                    return -0.75;
                },
                () -> {
                    return 0;
                },
                () -> {
                    return 0;
                });

        cmd = cmd.withTimeout(2.5);

        SequentialCommandGroup command = new SequentialCommandGroup(
                new ShootLow().withTimeout(2.6),
                new ExtendIntake().withTimeout(0.1),
                new StartIntake().withTimeout(0.1), cmd, new ShootHigh().withTimeout(5),
                new StopIntake().withTimeout(0.1), new RetractIntake().withTimeout(1));

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
