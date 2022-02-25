// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase {
  /** Creates a new IntakeBall. */
  private Intake l_IntakeSubsystem;
  public IntakeBall(Intake IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    l_IntakeSubsystem = IntakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    l_IntakeSubsystem.getBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    l_IntakeSubsystem.disableIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
