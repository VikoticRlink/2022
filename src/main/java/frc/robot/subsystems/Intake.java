// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Tools;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  WPI_TalonFX IntakeMotor = new WPI_TalonFX(Constants.MotorID.intakeMotor);
  WPI_TalonFX IntakeActuator = new WPI_TalonFX(Constants.MotorID.intakeActuator);

  public Intake() {
    // *** ToDo: replace this code once IntakeActuator works.  Right now, we are lying to
    // ***       the Falcon controller so that we can use the Intake motor in place of
    // ***       the actuator.

    IntakeActuator.setNeutralMode(NeutralMode.Brake);
    IntakeActuator.configForwardSoftLimitThreshold(0, 0);
    IntakeActuator.configReverseSoftLimitThreshold(23000, 0);
    IntakeActuator.configForwardSoftLimitEnable(false, 0);
    IntakeActuator.configReverseSoftLimitEnable(false, 0);
    configIntake();
    IntakeActuator.setSelectedSensorPosition(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Motor is currently running at
    if(RobotState.isEnabled() && RobotState.isTeleop() && RobotContainer.ManualControl){
      double driveAmount = Tools.featherJoystick(RobotContainer.OperatorController.getRightTriggerAxis(), Constants.JoystickSensitivity);
      //double intakeAmount = Tools.featherJoystick(RobotContainer.OperatorController.getRightY(), Constants.JoystickSensitivity);
      driveAmount *= Constants.MotorScaler.kIntakeSpeed;  // Scale the motor speed
     // intakeAmount *= Constants.MotorScaler.kIntakeActuator;
     // IntakeActuator.set(TalonFXControlMode.PercentOutput, -1 * intakeAmount);
      IntakeMotor.set(TalonFXControlMode.PercentOutput, -1 * driveAmount);
  }
}

  public void getBalls(){
    //setIntake(1);
    IntakeMotor.set(TalonFXControlMode.PercentOutput, Constants.MotorScaler.kIntakeSpeed);
    RobotContainer.m_Shooter.Run_Index_Motor(1);
    setIntake(1);
    RobotContainer.m_tRex.setArms(4);
  }

  public void disableIntake(){
    IntakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    RobotContainer.m_Shooter.Run_Index_Motor(0);
    setIntake(0);
    RobotContainer.m_tRex.setArms(1);
  }
  public double readEncoder(){      
    return IntakeActuator.getSelectedSensorPosition(0);
  }

  public void setIntake(int GoToPosition){
    IntakeActuator.set(TalonFXControlMode.Position, Constants.ArmPosition[GoToPosition]);
  }
  private void configIntake(){
    IntakeActuator.setNeutralMode(NeutralMode.Brake);
    IntakeActuator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    IntakeActuator.setSensorPhase(Constants.kSensorPhase);
    IntakeActuator.setInverted(Constants.kMotorInvert);
    IntakeActuator.configNominalOutputForward(0, Constants.kTimeoutMs);
    IntakeActuator.configNominalOutputReverse(0, Constants.kTimeoutMs);
    IntakeActuator.configPeakOutputForward(1, Constants.kTimeoutMs);
    IntakeActuator.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    IntakeActuator.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      IntakeActuator.config_kF(Constants.kPIDLoopIdx, Constants.kIntakeGains.kF, Constants.kTimeoutMs);
      IntakeActuator.config_kP(Constants.kPIDLoopIdx, Constants.kIntakeGains.kP, Constants.kTimeoutMs);
      IntakeActuator.config_kI(Constants.kPIDLoopIdx, Constants.kIntakeGains.kI, Constants.kTimeoutMs);
      IntakeActuator.config_kD(Constants.kPIDLoopIdx, Constants.kIntakeGains.kD, Constants.kTimeoutMs);
  }
}
