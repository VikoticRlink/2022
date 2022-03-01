// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.shooter.FireBall;
import frc.robot.commands.shooter.BackOffFlywheel;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  public Dashboard() {
    SmartDashboard.putData("Fire Ball", new FireBall(RobotContainer.m_Shooter));
    SmartDashboard.putData("Intake Ball", new IntakeBall(RobotContainer.m_Intake));
    SmartDashboard.putData("BackOffFlywheel", new BackOffFlywheel(RobotContainer.m_Shooter));
//https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // We should look at Shuffleboard as our smartdashboard.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html

    
    SmartDashboard.putBoolean("Drive Direction", (RobotContainer.DriveDirection == 1));
    SmartDashboard.putBoolean("Manual Mode", RobotContainer.ManualControl);
    
    SmartDashboard.putBoolean("Ball Indexed", RobotContainer.m_Shooter.Limitswitch_Is_Closed());
    SmartDashboard.putNumber("Bot Heading", RobotContainer.m_DriveBase.BotHeading());
    SmartDashboard.putNumber("Intake Actuater", RobotContainer.m_Intake.readEncoder());
  }
}
