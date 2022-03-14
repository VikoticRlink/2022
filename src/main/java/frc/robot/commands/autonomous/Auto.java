// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto extends CommandBase {
  int driveStage=0;
  boolean IsDone;

  /** Creates a new Auto. */
  public Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_DriveBase.resetEncoders();
    RobotContainer.m_DriveBase.BrakeMode();
    IsDone=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Drive Stage", driveStage);
   switch (driveStage) {
      case 0: 
        if (RobotContainer.m_DriveBase.rightMaster.getSelectedSensorPosition() > -30000) {
          RobotContainer.m_Intake.getBalls();
          RobotContainer.m_DriveBase.tankDriveVolts(2, -2);
          }else{
            RobotContainer.m_DriveBase.tankDriveVolts(0,0);
            driveStage = 1;
          }
      break;
      case 1:
        if (RobotContainer.m_DriveBase.rightMaster.getSelectedSensorPosition() < 30000) {
          RobotContainer.m_DriveBase.tankDriveVolts(-3.6, 3.6);
        }else {
          driveStage = 2;
        }
      break;
    case 2:
      if (RobotContainer.m_DriveBase.rightMaster.getSelectedSensorPosition() < 120000) {
        RobotContainer.m_DriveBase.tankDriveVolts(-6.0, 6.0);
      //Shoot Ball Here
      }else{
        driveStage = 3;
        RobotContainer.m_DriveBase.tankDriveVolts(0, 0);
        RobotContainer.m_DriveBase.resetEncoders();
        RobotContainer.m_DriveBase.CoastMode();
        RobotContainer.m_Intake.disableIntake();
        this.isScheduled();
        IsDone=true;
      }
    break;
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IsDone;
  }
}