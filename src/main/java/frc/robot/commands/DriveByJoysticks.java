// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBaseNew;

public class DriveByJoysticks extends CommandBase {
  private static double DrivePowerModifer = 1;
  /** Creates a new DriveByJoysticks. */
  public DriveByJoysticks() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.m_DriveBase);
    requires(RobotContainer.m_DriveBase);
  }

  private void requires(DriveBaseNew m_DriveBase) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.driverController.bumpRight.get()) {
      DrivePowerModifer = Constants.MotorScaler.DriveMidLimit;
      SmartDashboard.putString("Speed", "Medium");
    }
    else {
      if (RobotContainer.driverController.bumpLeft.get()) {
        DrivePowerModifer = Constants.MotorScaler.DriveSlowLimit;
        SmartDashboard.putString("Speed", "Slow");
      }
      else {
        DrivePowerModifer = Constants.MotorScaler.DriveStandardLimit;
        SmartDashboard.putString("Speed", "Normal");
      }
    }


      if (RobotContainer.DriveDirection == RobotContainer.RobotDirection.Forward) {
        RobotContainer.m_DriveBase.tankDrive(DrivePowerModifer * RobotContainer.driverController.rightStickY(),
            DrivePowerModifer * RobotContainer.driverController.leftStickY());

      }
      else {
        RobotContainer.m_DriveBase.tankDrive(-1 * DrivePowerModifer * RobotContainer.driverController.leftStickY(),
            -1 * DrivePowerModifer * RobotContainer.driverController.rightStickY());
      }
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_DriveBase.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
