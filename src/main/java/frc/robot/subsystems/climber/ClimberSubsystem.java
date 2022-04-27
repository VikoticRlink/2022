///////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5290 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;

public class ClimberSubsystem extends SubsystemBase {
  /** Handle to present bot state values */
  private BotStateSubsystem m_botState;
  /** Handle to the joystick subsystem */
  JoystickSubsystem m_joystickSubsystem;

  /** Talon motor driving the climber arms */
  WPI_TalonFX m_masterMotor = new WPI_TalonFX(Constants.MotorID.climberMaster);
  /** Talon motor driving the climber arms */
  WPI_TalonFX m_slaveMotor = new WPI_TalonFX(Constants.MotorID.climberSlave);

  /**
   * Talon FX motors support multiple (cascaded) PID loops that are identified by index. This
   * subsystem requires just one PID loop whose index is given by this constant.
   */
  public static final int kPIDIndex = 0;

  /**
   * Modes that the Climb Elevator can be run in
   */
  public enum ClimberMode {
    /** Climb Elevator is stopped */
    Stopped(0.0),
    /** Climb Elevator is in Manual Mode */
    Manual(2),
    /** Extend Climb Elevator */
    Extend(1.0),
    /** Retract Climb Elevator */
    Retract(-1.0);

    private final double motorSpeed;

    private ClimberMode(double speed) {
      motorSpeed = speed;
    }

    public double getMotorSpeed() {
      return motorSpeed;
    }
  };

  /**
   * Creates an instance of the subsystem
   * 
   * @param botState          Handle to information about the present state of the bot
   * @param joystickSubsystem Subsystem used to process joystick commands
   */
  public ClimberSubsystem(BotStateSubsystem botState, JoystickSubsystem joystickSubsystem) {
    m_botState = botState;
    m_joystickSubsystem = joystickSubsystem;

    m_slaveMotor.setInverted(true);
    m_slaveMotor.follow(m_masterMotor);
    m_slaveMotor.setNeutralMode(NeutralMode.Brake);
    m_masterMotor.setNeutralMode(NeutralMode.Brake);
    m_masterMotor.setSelectedSensorPosition(0, 0, 0);

    m_masterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_masterMotor.configReverseSoftLimitThreshold(-450000, 0);
    m_masterMotor.configForwardSoftLimitThreshold(0, 0);
    m_masterMotor.configForwardSoftLimitEnable(false, 0);
    m_masterMotor.configReverseSoftLimitEnable(true, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_botState.robotIsInManualTeleOpMode()) {
      // runClimberElevator(ClimberMode.Manual);
      double climbAmount = m_joystickSubsystem.operatorController.leftStickY();
      climbAmount *= Constants.MotorScaler.kClimberSpeed;
      m_masterMotor.set(TalonFXControlMode.PercentOutput, climbAmount);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Runs the Climber Arms
   * 
   * @param mode Mode to run the ball indexer in
   */
  public void runClimberElevator(ClimberMode mode) {
    double percentOutput = mode.getMotorSpeed();

    if (ClimberMode.Stopped != mode) {
      percentOutput *= Constants.MotorScaler.kClimberSpeed;
    }
    switch (mode) {
    case Stopped:
      percentOutput = 0;
      break;
    case Manual:
      m_joystickSubsystem.operatorController.leftStickY();
      break;
    default:
      percentOutput *= Constants.MotorScaler.kClimberSpeed;
      break;
    }
    m_masterMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the raw encoder count of the T-rex arm master motor */
  public double getClimberPosition() {
    return m_masterMotor.getSelectedSensorPosition(kPIDIndex);
  }

  /** Resets the encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_masterMotor.setSelectedSensorPosition(0, 0, 0);
  }

}
