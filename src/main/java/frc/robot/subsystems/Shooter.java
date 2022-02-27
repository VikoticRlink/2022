// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools;


public class Shooter extends SubsystemBase {
  TalonFX ShooterMotor = new TalonFX(22);
  TalonFX ShooterMotorSlave = new TalonFX(18);
  TalonFX IndexMotor = new TalonFX(23);
  private DigitalInput BallIndexer = new DigitalInput(0);
  
  
  
  /** Creates a new Shooter. */
  public Shooter()
  {
    ShooterMotorSlave.follow(ShooterMotor); 
    ShooterMotorSlave.setInverted(Constants.kMotorInvert);
  }

  public boolean Limitswitch_Is_Closed() {
    return RobotContainer.OperatorController.getBButton();
    //return BallIndexer.get();
  }


  public void Run_Index_Motor(double direction) {
    // Values -1, 0 , 1 accepted | -1 -> Backwards, 0 -> Stop, 1 -> Forwards
    IndexMotor.set(TalonFXControlMode.PercentOutput, .50 * direction);
  }

  public void Run_Flywheel_Motor(double run) {
    // Values 0 , 1 accepted | 0 -> Stop, 1 -> Forwards
    ShooterMotor.set(TalonFXControlMode.PercentOutput, -1 * run);
  }

  @Override
  public void periodic() {

    if(RobotState.isEnabled() && RobotState.isTeleop() && RobotContainer.ManualControl){
      ShooterMotor.set(TalonFXControlMode.PercentOutput, -1 * Tools.featherJoystick(RobotContainer.OperatorController.getRightTriggerAxis(), Constants.JoystickSensitivity));
      IndexMotor.set(TalonFXControlMode.PercentOutput, -1 * Tools.featherJoystick(RobotContainer.OperatorController.getLeftTriggerAxis(), Constants.JoystickSensitivity));

  }
    // This method will be called once per scheduler run
  }
}
