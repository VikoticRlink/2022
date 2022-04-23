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

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.subsystems.climber.T_rexSubsystem;
import frc.robot.subsystems.climber.T_rexSubsystem.TrexArmPosition;

///////////////////////////////////////////////////////////////////////////////
/**
 * Subsystem providing an intake system used by the robot to gather balls
 * into the Shooter
 * 
 * WPI_TalonFX class reference:
 * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_f_x.html
 */
public class IntakeSubsystem extends SubsystemBase {

  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////

  /* Choose true so that Talon does not report sensor out of phase */
  private static final boolean kSensorPhase = true;

  /** Set to control which direction the intake motor moves for positive values */
  private static final boolean kInvertIntakeMotor = false;

  /**
   * Set to control which direction the intake arm motor moves for positive values
   */
  private static final boolean kInvertIntakeArmMotor = false;

  /**
   * Talon FX motors support multiple (cascaded) PID loops that are identified
   * by index. This subsystem requires just one PID loop whose index is given
   * by this constant.
   */
  public static final int kPIDIndex = 0;

  //////////////////////////////////
  /// *** ATTRIBUTES ***
  //////////////////////////////////
  /** Handle to the robot runtime state */
  BotStateSubsystem m_botState;
  /** Handle to the climber T-rex arm subsystem */
  T_rexSubsystem m_tRex;
  /** Shooter subsystem providing the index motor */
  ShooterSubsystem m_shooterSubsystem;
  /** Motor used to pull balls into the shooter */
  WPI_TalonFX m_takeUpMotor;
  /** Motor used to position the intake arm */
  WPI_TalonFX m_armMotor;

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the Intake subsystem
   * 
   * @param shooterSubsystem Reference to a Shooter subsystem to use
   */
  public IntakeSubsystem(BotStateSubsystem botState, T_rexSubsystem tRexSubsystem,
      ShooterSubsystem shooterSubsystem) {
    m_botState = botState;
    m_tRex = tRexSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_takeUpMotor = new WPI_TalonFX(Constants.MotorID.intakeMotor);
    m_armMotor = new WPI_TalonFX(Constants.MotorID.intakeActuator);

    // Configure the intake motors
    configure();
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Called each scheduler cycle to run the subsystem
   * 
   * @see SubsystemBase.periodic()
   */
  @Override
  public void periodic() {

    // Service the subsystem when the robot is in Manual mode
    if (m_botState.robotIsInManualTeleOpMode()) {
      // Left trigger pull amount drives the intake speed
      // double driveAmount = tainer.operatorController.leftTriggerPull()
      // * Constants.MotorScaler.kIntakeMotorSpeed;
      // m_takeUpMotor.set(TalonFXControlMode.PercentOutput, -1 * driveAmount);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Configures the Intake subsystem for gathering balls
   */
  public void getBalls() {
    // Move T-Rex arms to gather balls
    m_tRex.setArmPosition(TrexArmPosition.BallCollect);

    // Move the intake arm into position to gather balls
    setIntakeArmPosition(IntakeArmPosition.DownToGatherBalls);

    // Run the intake motor to start pulling in balls
    m_takeUpMotor.set(TalonFXControlMode.PercentOutput,
        -1 * Constants.MotorScaler.kIntakeMotorSpeed);

    // Run the ball indexer to pull balls from the intake into the shooter
    m_shooterSubsystem.runBallIndexer(ShooterSubsystem.BallIndexerMode.FeedBall);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Disables the intake subsystem
   */
  public void disableIntake() {
    m_takeUpMotor.set(TalonFXControlMode.PercentOutput, 0);
    m_shooterSubsystem.runBallIndexer(ShooterSubsystem.BallIndexerMode.Stopped);
    setIntakeArmPosition(IntakeArmPosition.UpAndStowedAway);
    m_tRex.setArmPosition(TrexArmPosition.ArmsUp);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Positions of the intake arms for use with the setIntakeArms() method */
  public enum IntakeArmPosition {
    /** Intake arms down for gathering balls */
    DownToGatherBalls(19307),
    /** Intake arms at frame limit */
    FrameLimit(23000),
    /** Intake arms are stowed away in their top position */
    UpAndStowedAway(0);

    private final int encoderCount;

    private IntakeArmPosition(int count) {
      this.encoderCount = count;
    }

    public int getEncoderCount() {
      return encoderCount;
    }
  };

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Set the position of the intake arms
   * 
   * @param position Position to move the intake arms to
   */
  public void setIntakeArmPosition(IntakeArmPosition position) {
    m_armMotor.set(TalonFXControlMode.Position, position.getEncoderCount());
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the number of balls that are presently detected */
  public int numBallsDetected() {
    return m_shooterSubsystem.numBallsDetected();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Configures the intake and actuator motors
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

    // Configure the actuator motor used to move the intake arm
    m_armMotor.setInverted(kInvertIntakeArmMotor);
    m_armMotor.setNeutralMode(NeutralMode.Brake);
    // m_intakeActuator.configReverseSoftLimitThreshold(23000, 0);
    // m_intakeActuator.configForwardSoftLimitThreshold(0, 0);
    m_armMotor.configForwardSoftLimitEnable(false, 0);
    m_armMotor.configReverseSoftLimitEnable(false, 0);

    // Configure the motor for the intake arm to use its built-in encoder for
    // position control
    m_armMotor.setSelectedSensorPosition(0, 0, 0);
    m_armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        kPIDIndex, kConfigTimeoutMs);

    m_armMotor.setSensorPhase(kSensorPhase);
    m_armMotor.setInverted(kInvertIntakeMotor);
    m_armMotor.configNominalOutputForward(0, kConfigTimeoutMs);
    m_armMotor.configNominalOutputReverse(0, kConfigTimeoutMs);
    m_armMotor.configPeakOutputForward(1, kConfigTimeoutMs);
    m_armMotor.configPeakOutputReverse(-1, kConfigTimeoutMs);
    m_armMotor.configAllowableClosedloopError(0, kPIDIndex, kConfigTimeoutMs);

    // Config Position Closed Loop gains in slot0.
    // NOTE: typically, kF stays zero.
    m_armMotor.config_kF(kPIDIndex, Constants.kIntakeGains.kF, kConfigTimeoutMs);
    m_armMotor.config_kP(kPIDIndex, Constants.kIntakeGains.kP, kConfigTimeoutMs);
    m_armMotor.config_kI(kPIDIndex, Constants.kIntakeGains.kI, kConfigTimeoutMs);
    m_armMotor.config_kD(kPIDIndex, Constants.kIntakeGains.kD, kConfigTimeoutMs);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the raw encoder count of the intake arm master motor */
  public double getIntakeArmPositionRaw() {
    return m_armMotor.getSelectedSensorPosition(kPIDIndex);
  }

  /** Resets the encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_armMotor.setSelectedSensorPosition(0, 0, 0);
  }
}
