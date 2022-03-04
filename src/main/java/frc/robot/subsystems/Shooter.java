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
import com.ctre.phoenix.motorcontrol.WPI_MotorSafetyImplem;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools;


public class Shooter extends SubsystemBase {
  WPI_TalonFX ShooterMotor = new WPI_TalonFX(Constants.MotorID.shooterMaster);
  WPI_TalonFX ShooterMotorSlave = new WPI_TalonFX(Constants.MotorID.shooterSlave);
  WPI_TalonFX IndexMotor = new WPI_TalonFX(Constants.MotorID.indexMotor);
  private DigitalInput BallIndexer = new DigitalInput(0);
  private DigitalInput BallIndexer2 = new DigitalInput(1);
  
  
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
  public boolean BallPrimed() {
    return BallIndexer.get();
  } public boolean SecondBall() {
    return BallIndexer2.get();
  }

  public void Run_Index_Motor(double direction) {
    // Values -1, 0 , 1 accepted | -1 -> Backwards, 0 -> Stop, 1 -> Forwards
    IndexMotor.set(TalonFXControlMode.PercentOutput, 0.50 * direction);
  }

  public void Run_Flywheel_Motor(double run) {
    // Values 0 , 1 accepted | 0 -> Stop, 1 -> Forwards
    ShooterMotor.set(TalonFXControlMode.PercentOutput, -1 * run);
  }

  @Override
  public void periodic() {

    if(RobotState.isEnabled() && RobotState.isTeleop() && RobotContainer.ManualControl){
      double shooterAmount = Tools.featherJoystick(RobotContainer.OperatorController.getLeftTriggerAxis(), Constants.JoystickSensitivity);
      double indexAmount = Tools.featherJoystick(RobotContainer.OperatorController.getRightTriggerAxis(), Constants.JoystickSensitivity);
      shooterAmount *= Constants.MotorScaler.kShooterSpeed;  // Scale the motor speed
      indexAmount *= Constants.MotorScaler.kIndexSpeed;  // Scale the motor speed

      ShooterMotor.set(TalonFXControlMode.PercentOutput, -1 * shooterAmount);
      IndexMotor.set(TalonFXControlMode.PercentOutput, indexAmount);

  }
    // This method will be called once per scheduler run
  }
}
