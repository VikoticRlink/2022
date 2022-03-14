// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  public Dashboard() {
    //SmartDashboard.putData("Fire Ball", new FireBall(RobotContainer.m_Shooter));
    //SmartDashboard.putData("Intake Ball", new IntakeBall(RobotContainer.m_Intake));
    //SmartDashboard.putData("BackOffFlywheel", new BackOffFlywheel(RobotContainer.m_Shooter));
    // SEE: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // We should look at Shuffleboard as our smartdashboard.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html

    //SmartDashboard.putString("Auto chooser", RobotContainer.m_chooser.getSelected().getName());
    SmartDashboard.putString("Drive Direction", RobotContainer.DriveDirection.toString());
    SmartDashboard.putBoolean("Manual Mode", RobotContainer.ManualControl);
    SmartDashboard.putBoolean("Alliance", RobotContainer.isRedAlliance);
    
    SmartDashboard.putBoolean("First Ball", RobotContainer.m_Shooter.upperBallIsPresent());
    SmartDashboard.putBoolean("Second Ball", RobotContainer.m_Shooter.lowerBallIsPresent());
    SmartDashboard.putBoolean("Ball Against Flywheel", RobotContainer.m_Shooter.getBallLimitSensor());
    //SmartDashboard.putNumber("Bot Heading", RobotContainer.m_DriveBase.getHeading());
    SmartDashboard.putNumber("Intake Arm Position", RobotContainer.m_Intake.getIntakeArmPositionRaw());
    SmartDashboard.putNumber("T-rex Arm position", RobotContainer.m_tRex.getArmPositionRaw());

    SmartDashboard.putNumber("leftDrive", RobotContainer.m_DriveBase.leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightDrive", RobotContainer.m_DriveBase.rightMaster.getSelectedSensorPosition());

    
//PushMotorTemps();
  }

  //Motor Temp records
  //private void PushMotorTemps(){
    //SmartDashboard.putNumber("Left Drive Temp", RobotContainer.m_DriveBase.leftMaster.getTemperature());
    //SmartDashboard.putNumber("Left Drive Slave Temp", RobotContainer.m_DriveBase.leftSlave.getTemperature());
    //SmartDashboard.putNumber("Right Drive Temp", RobotContainer.m_DriveBase.rightMaster.getTemperature());
    //SmartDashboard.putNumber("Right Drive Slave Temp", RobotContainer.m_DriveBase.rightSlave.getTemperature());
  //  SmartDashboard.putNumber("Climb Master Temp", RobotContainer.m_Climber.ClimbMaster.getTemperature());
  //  SmartDashboard.putNumber("Climb Slave Temp", RobotContainer.m_Climber.ClimbSlave.getTemperature());
  //  SmartDashboard.putNumber("Intake Actuator Temp", RobotContainer.m_Intake.m_armMotor.getTemperature());
  //  SmartDashboard.putNumber("tRex Master Temp", RobotContainer.m_tRex.m_tRexMaster.getTemperature());
  //  SmartDashboard.putNumber("tRex Slave Temp", RobotContainer.m_tRex.m_tRexSlave.getTemperature());
   
  //}
}

