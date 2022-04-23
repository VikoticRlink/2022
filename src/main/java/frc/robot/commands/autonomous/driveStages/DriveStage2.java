// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.driveStages;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.driveBase.DriveBaseNew;

/** Implement Autonomous mode Drive stage 2 */
public class DriveStage2 extends CommandBase {
  DriveBaseNew m_driveBaseSubsystem;

  public DriveStage2(DriveBaseNew driveBaseSubsystem) {
    m_driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveBaseSubsystem.tankDriveVolts(0, 0);
    m_driveBaseSubsystem.resetEncoders();
    m_driveBaseSubsystem.CoastMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
