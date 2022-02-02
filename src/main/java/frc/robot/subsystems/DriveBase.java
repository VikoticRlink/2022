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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools;

public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  TalonFX leftMaster = new TalonFX(1);
  TalonFX rightMaster = new TalonFX(2);

  TalonFXConfiguration configs = new TalonFXConfiguration();

  public DriveBase() {
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    leftMaster.configAllSettings(configs);
    rightMaster.configAllSettings(configs);

    rightMaster.setSelectedSensorPosition(0, 0, 0);
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    configDrivebase();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    if(RobotState.isEnabled() && RobotState.isTeleop()){
      if(RobotContainer.DriverController.getRightBumper()){
        leftMaster.set(TalonFXControlMode.PercentOutput, Tools.featherJoystick(RobotContainer.DriverController.getLeftY(), Constants.JoystickSensitivity));
        rightMaster.set(TalonFXControlMode.PercentOutput, -1 * Tools.featherJoystick(RobotContainer.DriverController.getRightY(), Constants.JoystickSensitivity));
        SmartDashboard.putString("Speed", "Turbo");
      }else{
        if(RobotContainer.DriverController.getLeftBumper()){
          leftMaster.set(TalonFXControlMode.PercentOutput, (Constants.SlowSpeedLimit * Tools.featherJoystick(RobotContainer.DriverController.getLeftY(), Constants.JoystickSensitivity)));
          rightMaster.set(TalonFXControlMode.PercentOutput, (-1 * Constants.SlowSpeedLimit * Tools.featherJoystick(RobotContainer.DriverController.getRightY(), Constants.JoystickSensitivity)));
          SmartDashboard.putString("Speed", "Slow");
        }else{
          leftMaster.set(TalonFXControlMode.PercentOutput, (Constants.StandardSpeedLimit * Tools.featherJoystick(RobotContainer.DriverController.getLeftY(), Constants.JoystickSensitivity)));
          rightMaster.set(TalonFXControlMode.PercentOutput, (-1 * Constants.StandardSpeedLimit * Tools.featherJoystick(RobotContainer.DriverController.getRightY(), Constants.JoystickSensitivity)));
          SmartDashboard.putString("Speed", "Normal");
        }
      }
    }

  } 

  
  
  private void configDrivebase(){
    //rightMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    rightMaster.setSensorPhase(Constants.kSensorPhase);
    rightMaster.setInverted(Constants.kMotorInvert);
    rightMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    rightMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    rightMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    rightMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    rightMaster.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      rightMaster.config_kF(Constants.kPIDLoopIdx, Constants.kDriveGains.kF, Constants.kTimeoutMs);
      rightMaster.config_kP(Constants.kPIDLoopIdx, Constants.kDriveGains.kP, Constants.kTimeoutMs);
      rightMaster.config_kI(Constants.kPIDLoopIdx, Constants.kDriveGains.kI, Constants.kTimeoutMs);
      rightMaster.config_kD(Constants.kPIDLoopIdx, Constants.kDriveGains.kD, Constants.kTimeoutMs);
  
    //leftMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    leftMaster.setSensorPhase(Constants.kSensorPhase);
    leftMaster.setInverted(Constants.kMotorInvert);
    leftMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    leftMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    leftMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    leftMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    leftMaster.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      leftMaster.config_kF(Constants.kPIDLoopIdx, Constants.kDriveGains.kF, Constants.kTimeoutMs);
      leftMaster.config_kP(Constants.kPIDLoopIdx, Constants.kDriveGains.kP, Constants.kTimeoutMs);
      leftMaster.config_kI(Constants.kPIDLoopIdx, Constants.kDriveGains.kI, Constants.kTimeoutMs);
      leftMaster.config_kD(Constants.kPIDLoopIdx, Constants.kDriveGains.kD, Constants.kTimeoutMs);
  }
  public void resetEncoders(){
    rightMaster.setSelectedSensorPosition(0, 0, 0);
    leftMaster.setSelectedSensorPosition(0, 0, 0);
  }
  public void CoastMode(){
    rightMaster.setNeutralMode(NeutralMode.Coast);
   // rightSlave.setNeutralMode(NeutralMode.Coast);
    leftMaster.setNeutralMode(NeutralMode.Coast);
   // leftSlave.setNeutralMode(NeutralMode.Coast);
  }
  public void BrakeMode(){
    rightMaster.setNeutralMode(NeutralMode.Brake);
   // rightSlave.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
   // leftSlave.setNeutralMode(NeutralMode.Brake);
  
  }
}
// Falcon 500 examples
//https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)