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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

///////////////////////////////////////////////////////////////////////////////
/**
 * Subsystem providing a pair of short T-rex arms used by the robot for
 * climbing. The arms are driven by two motors designated as 'master' and
 * 'slave'. The 'slave' motor controller is configured to track the 'master'.
 *
 * WPI_TalonFX class reference:
 * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_f_x.html
 */
public class T_rexSubsystem extends SubsystemBase {

  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////
  /* Choose true so that Talon does not report sensor out of phase */
  private static final boolean kSensorPhase = true;

  /** Set to control which direction the motors move for positive values */
  private static final boolean kMotorInvert = false;

  /**
   * Talon FX motors support multiple (cascaded) PID loops that are identified
   * by index. This subsystem requires just one PID loop whose index is given
   * by this constant.
   */
  public static final int kPIDIndex = 0;

  //////////////////////////////////
  /// *** ATTRIBUTES ***
  //////////////////////////////////
  /** Handle to present bot state values */
  private BotStateSubsystem m_botState;
  /** Handle to the joystick subsystem */
  JoystickSubsystem m_joystickSubsystem;
  /** Master motor used to position the T-rex arms */
  WPI_TalonFX m_tRexMaster;
  /** Slave motor used to position the T-rex arms */
  WPI_TalonFX m_tRexSlave; 

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the T_rex subsystem. */
  public T_rexSubsystem(BotStateSubsystem botState, JoystickSubsystem joystickSubsystem) {
    m_botState = botState;
    m_joystickSubsystem = joystickSubsystem;
    m_tRexMaster = new WPI_TalonFX(Constants.MotorID.tRexMaster);
    m_tRexSlave = new WPI_TalonFX(Constants.MotorID.tRexSlave);
    configure();
  }

  /** Positions the T-rex arms can be set to using the setArmPosition() method */
  public enum TrexArmPosition {
    // public static final int[] tRexPosition = new int []{0, 13629, 31880, 43589,
    // 54841, 68187};

    /** < T-rex arms are stowed away in their top position */
    StowedAway(0),
    /** < T-Rex arms up - not gathering balls */
    ArmsUp(13629),
    /** < T-Rex arms at a 45 (climbing) */
    FourtyFive(31880),
    /** < T-Rex arms straight out */
    StraightOut(43589),
    /** < T-Rex arms bouncing balls in */
    BallCollect(59000),
    /** < T-Rex arms fully down */
    FullDown(68187),
    /** < Emergency Stop and hold */
    ESTOP(-1000);

    private final int encoderCount;

    private TrexArmPosition(int count) {
      this.encoderCount = count;
    }

    public int getEncoderCount() {
      return encoderCount;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Set the position of the T-rex arms as a value between bottom and top
   * position (0.0 to 1.0 respectively)
   * 
   * @param position Position to move the T-rex arms to
   */
  public void setArmPosition(TrexArmPosition position) {

    if (position != TrexArmPosition.ESTOP) {
      m_tRexMaster.set(TalonFXControlMode.Position, position.getEncoderCount());
    } else {
      m_tRexMaster.set(TalonFXControlMode.Position, getArmPositionRaw());
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Called each scheduler cycle to run the subsystem
   * 
   * @see SubsystemBase.periodic()
   */
  @Override
  public void periodic() {
    if (m_botState.robotIsInManualTeleOpMode()) {
      double armSwing = m_joystickSubsystem.operatorController.rightStickY()
          * Constants.MotorScaler.kTRexArmManualSpeed;
      m_tRexMaster.set(TalonFXControlMode.PercentOutput, -1 * armSwing);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Configures motors used for the T-rex arms
  private void configure() {

    // Timeout value (in milliseconds) used for commands used to configure the
    // motor. If nonzero, config functions will block while waiting for motor
    // configuration to succeed, and report an error if configuration times out.
    // If zero, no blocking or error checking is performed.
    final int kConfigTimeoutMs = 30;

    // Talon FX motors support multiple (cascaded) PID loops, each of which has
    // a configuration that is identified by an index number. This subsystem
    // requires only one PID controller. Its index is given by this constant.
    final int kPIDIndex = 0;

    // Configure the T-rex slave motor
    // inverted direction.
    m_tRexSlave.setInverted(true); // Follow the motion of the master
    m_tRexSlave.follow(m_tRexMaster); // Slave direction is opposite of master
    m_tRexSlave.setNeutralMode(NeutralMode.Brake);

    // Configure the T-rex master motor
    m_tRexMaster.setNeutralMode(NeutralMode.Brake);

    // m_tRexMaster.configReverseSoftLimitThreshold(52000, 0);
    // m_tRexMaster.configForwardSoftLimitThreshold(0, 0);
    m_tRexMaster.configForwardSoftLimitEnable(false, 0);
    m_tRexMaster.configReverseSoftLimitEnable(false, 0);

    // Configure the motor to use its built-in encoder for position control
    m_tRexMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        kPIDIndex, kConfigTimeoutMs);

    m_tRexMaster.setSensorPhase(kSensorPhase);
    m_tRexMaster.setInverted(kMotorInvert);
    m_tRexMaster.configNominalOutputForward(0, kConfigTimeoutMs);
    m_tRexMaster.configNominalOutputReverse(0, kConfigTimeoutMs);
    m_tRexMaster.configPeakOutputForward(1, kConfigTimeoutMs);
    m_tRexMaster.configPeakOutputReverse(-1, kConfigTimeoutMs);
    m_tRexMaster.configAllowableClosedloopError(0, kPIDIndex, kConfigTimeoutMs);

    // Config Position Closed Loop gains
    // NOTE: typically, kF stays zero.
    m_tRexMaster.config_kF(kPIDIndex, Constants.ktRexGains.kF, kConfigTimeoutMs);
    m_tRexMaster.config_kP(kPIDIndex, Constants.ktRexGains.kP, kConfigTimeoutMs);
    m_tRexMaster.config_kI(kPIDIndex, Constants.ktRexGains.kI, kConfigTimeoutMs);
    m_tRexMaster.config_kD(kPIDIndex, Constants.ktRexGains.kD, kConfigTimeoutMs);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the raw encoder count of the T-rex arm master motor */
  public double getArmPositionRaw() {
    return m_tRexMaster.getSelectedSensorPosition(kPIDIndex);
  }

  /** Resets the encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_tRexMaster.setSelectedSensorPosition(0, 0, 0);
  }

}