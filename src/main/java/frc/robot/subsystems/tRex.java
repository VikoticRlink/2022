// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools;

public class tRex extends SubsystemBase {
  /** Creates a new tRex. */
  WPI_TalonFX tRexMaster = new WPI_TalonFX(Constants.MotorID.tRexMaster);
  WPI_TalonFX tRexSlave = new WPI_TalonFX(Constants.MotorID.tRexSlave);
  public tRex() {
    tRexSlave.setInverted(true);
    tRexSlave.follow(tRexMaster);
    tRexSlave.setNeutralMode(NeutralMode.Brake);
    tRexMaster.setNeutralMode(NeutralMode.Brake);
    tRexMaster.setSelectedSensorPosition(0, 0, 0);
    tRexMaster.setNeutralMode(NeutralMode.Brake);
    tRexMaster.configForwardSoftLimitThreshold(0, 0);
    tRexMaster.configReverseSoftLimitThreshold(68187, 0);
    tRexMaster.configForwardSoftLimitEnable(false, 0);
    tRexMaster.configReverseSoftLimitEnable(false, 0);
    configArms();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotState.isEnabled() && RobotState.isTeleop() && RobotContainer.ManualControl){
      double armSwing = Tools.featherJoystick(RobotContainer.OperatorController.getRightY(), Constants.JoystickSensitivity);
      armSwing *= Constants.MotorScaler.ktRexArms;
      tRexMaster.set(TalonFXControlMode.PercentOutput, -1 * armSwing);
  }
  }

  public double readEncoder(){
    
    return tRexMaster.getSelectedSensorPosition(0);
  }
  public void setArms(int GoToPosition){
    tRexMaster.set(TalonFXControlMode.Position, Constants.tRexPosition[GoToPosition]);
  }
  private void configArms(){
    tRexMaster.setNeutralMode(NeutralMode.Brake);
    tRexMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    tRexMaster.setSensorPhase(Constants.kSensorPhase);
    tRexMaster.setInverted(Constants.kMotorInvert);
    tRexMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    tRexMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    tRexMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    tRexMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    tRexMaster.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      tRexMaster.config_kF(Constants.kPIDLoopIdx, Constants.ktRexGains.kF, Constants.kTimeoutMs);
      tRexMaster.config_kP(Constants.kPIDLoopIdx, Constants.ktRexGains.kP, Constants.kTimeoutMs);
      tRexMaster.config_kI(Constants.kPIDLoopIdx, Constants.ktRexGains.kI, Constants.kTimeoutMs);
      tRexMaster.config_kD(Constants.kPIDLoopIdx, Constants.ktRexGains.kD, Constants.kTimeoutMs);
  }
}
