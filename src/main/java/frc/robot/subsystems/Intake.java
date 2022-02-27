// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_TalonFX IntakeMotor = new WPI_TalonFX(20);
  WPI_TalonFX IntakeActuator = new WPI_TalonFX(31);
  WPI_TalonFX IntakeActuatorSlave = new WPI_TalonFX(32); // Will be only one motor when it goes to production
/*Soft Limits
Soft limits can be used to disable motor drive when the “Sensor Position” is outside of a specified range. Forward throttle will be disabled if the “Sensor Position” is greater than the Forward Soft Limit. Reverse throttle will be disabled if the “Sensor Position” is less than the Reverse Soft Limit. The respective Soft Limit Enable must be enabled for this feature to take effect.

 Talon configured to have soft limits 10000 native units in either direction and enabled 
rightMaster.configForwardSoftLimitThreshold(10000, 0);
rightMaster.configReverseSoftLimitThreshold(-10000, 0);
rightMaster.configForwardSoftLimitEnable(true, 0);
rightMaster.configReverseSoftLimitEnable(true, 0);
*/
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotState.isEnabled() && RobotState.isTeleop()){
      IntakeMotor.set(TalonFXControlMode.PercentOutput, -1 * Tools.featherJoystick(RobotContainer.OperatorController.getRightY(), Constants.JoystickSensitivity));
  }
}

  public void getBalls(){
    IntakeMotor.set(TalonFXControlMode.PercentOutput, -1);
    RobotContainer.m_Shooter.Run_Index_Motor(1);
  }

  public void disableIntake(){
    IntakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    RobotContainer.m_Shooter.Run_Index_Motor(0);
  }
}
