// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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
}
