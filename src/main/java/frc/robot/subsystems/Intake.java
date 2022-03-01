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
  WPI_TalonFX IntakeMotor = new WPI_TalonFX(Constants.MotorID.intakeMotor);
  WPI_TalonFX IntakeActuator = new WPI_TalonFX(Constants.MotorID.intakeActuator);
/*Soft Limits
Soft limits can be used to disable motor drive when the “Sensor Position” is outside of a specified range. Forward throttle will be disabled if the “Sensor Position” is greater than the Forward Soft Limit. Reverse throttle will be disabled if the “Sensor Position” is less than the Reverse Soft Limit. The respective Soft Limit Enable must be enabled for this feature to take effect.

 Talon configured to have soft limits 10000 native units in either direction and enabled 
rightMaster.configForwardSoftLimitThreshold(10000, 0);
rightMaster.configReverseSoftLimitThreshold(-10000, 0);
rightMaster.configForwardSoftLimitEnable(true, 0);
rightMaster.configReverseSoftLimitEnable(true, 0);
*/
  public Intake() {
    // *** ToDo: replace this code once IntakeActuator works.  Right now, we are lying to
    // ***       the Falcon controller so that we can use the Intake motor in place of
    // ***       the actuator.

    IntakeMotor.setNeutralMode(NeutralMode.Brake);
    IntakeActuator.configForwardSoftLimitThreshold(10000, 0);
    IntakeActuator.configReverseSoftLimitThreshold(-10000, 0);
    IntakeActuator.configForwardSoftLimitEnable(true, 0);
    IntakeActuator.configReverseSoftLimitEnable(true, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Motor is currently running at
    if(RobotState.isEnabled() && RobotState.isTeleop() && RobotContainer.ManualControl){
      double driveAmount = Tools.featherJoystick(RobotContainer.OperatorController.getRightTriggerAxis(), Constants.JoystickSensitivity);
      double intakeAmount = Tools.featherJoystick(RobotContainer.OperatorController.getRightY(), Constants.JoystickSensitivity);
      driveAmount *= Constants.MotorScaler.kIntakeSpeed;  // Scale the motor speed
      intakeAmount *= Constants.MotorScaler.kIntakeActuator;
      IntakeActuator.set(TalonFXControlMode.PercentOutput, -1 * intakeAmount);
      IntakeMotor.set(TalonFXControlMode.PercentOutput, -1 * driveAmount);
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
  public double readEncoder(){      
    return IntakeActuator.getSelectedSensorPosition(0);
  }
}
