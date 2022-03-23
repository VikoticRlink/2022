// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.driveStages;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveBaseNew;
import frc.robot.subsystems.Intake;

// Make sure ball is intaked.
// Run getballs for 2 seconds
public class DriveStage1 extends CommandBase {
  DriveBaseNew m_driveBaseSubsystem;
  Intake m_intakeSubsystem;

  public DriveStage1(DriveBaseNew driveBaseSubsystem, Intake intakeSubsystem) {
    m_driveBaseSubsystem = driveBaseSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    
    addRequirements(driveBaseSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    m_intakeSubsystem.getBalls();
    m_driveBaseSubsystem.tankDriveVolts(4, 4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBaseSubsystem.tankDriveVolts(0,0);
    m_driveBaseSubsystem.leftMaster.setSafetyEnabled(false);
    m_driveBaseSubsystem.rightMaster.setSafetyEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_driveBaseSubsystem.rightMaster.getSelectedSensorPosition() <= -60000);
  }
}
