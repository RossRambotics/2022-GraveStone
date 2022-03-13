// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class TwoBallCheck extends CommandBase {
  private boolean m_ballFound = false;

  /** Creates a new TwoBallCheck. */
  public TwoBallCheck() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.getTheRobot().m_Indexer.getSensorIndexerMiddle()
        && RobotContainer.getTheRobot().m_Indexer.getSensorIndexerEntry()) {
      m_ballFound = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CommandBase cmd = new frc.robot.commands.Intake.RetractIntake();
    CommandBase cmd2 = new frc.robot.commands.Intake.StopIntake();

    CommandBase all = new SequentialCommandGroup(cmd, cmd2);
    all.schedule();

    cmd.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_ballFound;
  }
}
