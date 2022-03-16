// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.driveStages;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveBaseNew;
import frc.robot.subsystems.Intake;

/** Implement Autonomous mode Drive stage 1 */
public class DriveStage0 extends CommandBase {
  DriveBaseNew m_driveBaseSubsystem;
  Intake m_intakeSubsystem;

  /** Create an instance of the command */
  public DriveStage0(DriveBaseNew driveBaseSubsystem, Intake intakeSubsystem) {
    m_driveBaseSubsystem = driveBaseSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    
    addRequirements(driveBaseSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveBaseSubsystem.resetEncoders();
    m_driveBaseSubsystem.BrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.getBalls();
    m_driveBaseSubsystem.tankDriveVolts(2, -2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBaseSubsystem.tankDriveVolts(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_driveBaseSubsystem.rightMaster.getSelectedSensorPosition() > -30000);
  }
}
