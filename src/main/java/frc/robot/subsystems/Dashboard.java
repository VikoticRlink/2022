// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;


public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  public Dashboard() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putBoolean("Drive Direction", (RobotContainer.DriveDirection == 1));
    SmartDashboard.putBoolean("Manual Mode", RobotContainer.ManualControl);
    SmartDashboard.putBoolean("Ball Indexed", RobotContainer.m_Shooter.Limitswitch_Is_Closed());
    
    SmartDashboard.putNumber("Bot Heading", RobotContainer.m_DriveBase.BotHeading());
  }
}
