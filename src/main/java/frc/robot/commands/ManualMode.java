// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.ClimberMode;
import frc.robot.subsystems.Shooter.BallIndexerMode;
import frc.robot.subsystems.T_rex.TrexArmPosition;

public class ManualMode extends CommandBase {
  Boolean HasRan = false;
  /** Creates a new ManualModeToggle. */
  public ManualMode() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.ManualControl=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ManualControl=false;
    RobotContainer.m_Shooter.runFlywheel(false);
    RobotContainer.m_Shooter.runBallIndexer(BallIndexerMode.Stopped);
    //RobotContainer.m_Climber.runClimberElevator(ClimberMode.Stopped);
    //RobotContainer.m_tRex.setArmPosition(TrexArmPosition.ESTOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
