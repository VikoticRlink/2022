// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX ClimbMaster = new WPI_TalonFX(Constants.MotorID.climberMaster);
  WPI_TalonFX ClimbSlave = new WPI_TalonFX(Constants.MotorID.climberSlave);
  
  	/**
	 * Talon FX motors support multiple (cascaded) PID loops that are identified
   * by index.  This subsystem requires just one PID loop whose index is given
   * by this constant.
	 */
  public static final int kPIDIndex = 0;

  /** Modes that the Climb Elevator can be run in
   */
  public enum ClimberMode {
    Stopped (0.0),    /** Climb Elevator is stopped */
    Manual (2),      /** Climb Elevator is in Manual Mode */
    Extend (1.0),   /** Extend Climb Elevator */
    Retract (-1.0);  /** Retract Climb Elevator */

    private final double motorSpeed;
    private ClimberMode(double speed) {motorSpeed = speed;}
    public double getMotorSpeed() { return motorSpeed; }
  };

  public Climber() {
    ClimbSlave.setInverted(true);
    ClimbSlave.follow(ClimbMaster);
    ClimbSlave.setNeutralMode(NeutralMode.Brake);
    ClimbMaster.setNeutralMode(NeutralMode.Brake);
    ClimbMaster.setSelectedSensorPosition(0, 0, 0);
    
    ClimbMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    ClimbMaster.configReverseSoftLimitThreshold(-443796, 0);
    ClimbMaster.configForwardSoftLimitThreshold(-700, 0);
    ClimbMaster.configForwardSoftLimitEnable(true, 0);
    ClimbMaster.configReverseSoftLimitEnable(true, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.robotIsInManualTeleOpMode()) {
     // runClimberElevator(ClimberMode.Manual);
        double climbAmount = RobotContainer.operatorController.leftStickY();
        climbAmount *= Constants.MotorScaler.kClimberSpeed;
        ClimbMaster.set(TalonFXControlMode.PercentOutput, climbAmount);
  }
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Runs the Climber Arms
   * 
   * @param mode  Mode to run the ball indexer in
   */
  public void runClimberElevator(ClimberMode mode) {
    double percentOutput = mode.getMotorSpeed();
    
    if (ClimberMode.Stopped != mode) {
      percentOutput *= Constants.MotorScaler.kClimberSpeed;
    }
    switch(mode){
      case Stopped:
        percentOutput = 0;
      break;
      case Manual:
        RobotContainer.operatorController.leftStickY();
      break;
      default:
        percentOutput *= Constants.MotorScaler.kClimberSpeed;
      break;
    }
    ClimbMaster.set(TalonFXControlMode.PercentOutput, percentOutput);
  }
  /////////////////////////////////////////////////////////////////////////////
  /** Returns the raw encoder count of the T-rex arm master motor */
  public double getClimberPosition()
  {
    return ClimbMaster.getSelectedSensorPosition(kPIDIndex);
  }
  
  /** Resets the encoders to currently read a position of 0. */
  public void resetEncoders() {
    ClimbMaster.setSelectedSensorPosition(0, 0, 0);
  }

}
