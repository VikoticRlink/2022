// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro; 
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

public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  private final Gyro m_gyro = new ADXRS450_Gyro();
  WPI_TalonFX leftMaster = new WPI_TalonFX(1);
  WPI_TalonFX rightMaster = new WPI_TalonFX(2);
  WPI_TalonFX leftSlave = new WPI_TalonFX(11);
  WPI_TalonFX rightSlave = new WPI_TalonFX(12);
//https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-drive-subsystem.html
//https://github-wiki-see.page/m/frc6421/WarriorBotsJAVABootcamp2021/wiki/4.-Autonomous--with-RamseteCommand
// alt drivebase config https://github.com/frc6421/WarriorBotsJAVABootcamp2021/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java

  TalonFXConfiguration configs = new TalonFXConfiguration();
  private static double DrivePowerModifer=1;

  public DriveBase() {
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    leftMaster.configAllSettings(configs);
    rightMaster.configAllSettings(configs);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    configDrivebase();
    m_gyro.calibrate();

    //pathweaver code
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run.
    SmartDashboard.putNumber("Left Drive", leftMaster.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Drive", rightMaster.getSelectedSensorPosition(0));
    
    
    if(RobotContainer.DriverController.getRightBumper()){
      DrivePowerModifer = 1;
      SmartDashboard.putString("Speed", "Turbo");
    }else{
      if(RobotContainer.DriverController.getLeftBumper()){
        DrivePowerModifer = Constants.SlowSpeedLimit ;
        SmartDashboard.putString("Speed", "Slow");
      }else{
        DrivePowerModifer = Constants.StandardSpeedLimit;
        SmartDashboard.putString("Speed", "Normal");
      }
    }

    if(RobotState.isEnabled() && RobotState.isTeleop()){
      if(RobotContainer.DriveDirection==1){
        leftMaster.set(TalonFXControlMode.PercentOutput, RobotContainer.DriveDirection * Tools.featherJoystick(RobotContainer.DriverController.getRightY(), Constants.JoystickSensitivity));
        rightMaster.set(TalonFXControlMode.PercentOutput, -1 * RobotContainer.DriveDirection * Tools.featherJoystick(RobotContainer.DriverController.getLeftY(), Constants.JoystickSensitivity));
      }else{
        leftMaster.set(TalonFXControlMode.PercentOutput, (DrivePowerModifer * Tools.featherJoystick(RobotContainer.DriverController.getRightY(), Constants.JoystickSensitivity)));
        rightMaster.set(TalonFXControlMode.PercentOutput, (-1 * DrivePowerModifer * Tools.featherJoystick(RobotContainer.DriverController.getLeftY(), Constants.JoystickSensitivity)));
      }
    }

  } 

  
  
  private void configDrivebase(){
    rightMaster.setNeutralMode(NeutralMode.Coast);
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
  
    leftMaster.setNeutralMode(NeutralMode.Coast);
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
  public double readEncoder(boolean readRight){
    if (readRight){
      return rightMaster.getSelectedSensorPosition(0);
    }else{
      return leftMaster.getSelectedSensorPosition(0);
    }
  }
  public void driveTo(int Distance){
    rightMaster.set(TalonFXControlMode.Position, 1);
    leftMaster.set(TalonFXControlMode.Position, -1 * Distance);
  
  }
  public void drivePosition(double leftInput, double rightInput){
    rightMaster.set(TalonFXControlMode.Position, rightInput);
    leftMaster.set(TalonFXControlMode.Position, leftInput);
  }
  public void drivePercent(double leftInput, double rightInput){
    rightMaster.set(TalonFXControlMode.PercentOutput, rightInput);
    leftMaster.set(TalonFXControlMode.PercentOutput, leftInput);
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
  public double BotHeading(){
    return m_gyro.getRotation2d().getDegrees();
  }
}
// Falcon 500 examples
//https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)
//Gyro info
//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
//Pathweaver
//https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/introduction.html
//Tragectory Tutorial Overview
//https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/trajectory-tutorial-overview.html
