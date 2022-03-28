// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utility.TalonPIDConfig;

///////////////////////////////////////////////////////////////////////////////
/**
 * The Shooter subsystem manages a set of flywheels used to launch balls
 * from the robot's shooter and a ball indexer that feeds balls into the
 * flywheels
 */
public class Shooter extends SubsystemBase {

  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////

  /** Set to true to reverse the direction of the flywheel motor */
  private static final boolean kInvertFlywheelMotor = true;

  /** Set to true to reverse the direction of the ball indexer motor */
  private static final boolean kInvertBallIndexerMotor = true;

  private static final double kDefault_kP = 0.04;
  private static final double kDefault_kI = 0.0;
  private static final double kDefault_kD = 0.0;
  private static final double kDefault_kF = 0.02;

  // FalconFX reports velocity in counts per 100ms
  // 1 revolution = 2048 counts
  // 1 minutes = 60 * 10 * 100ms
  // conversion is  600  / 2048
  private static final double ticks2RPm = 600.0 / 2048.0;

  
  //////////////////////////////////
  /// *** ATTRIBUTES ***
  //////////////////////////////////
  WPI_TalonFX m_flywheelMotorMaster;
  /** < Master motor used to drive the flywheels */
  WPI_TalonFX m_flywheelMotorSlave;
  /** < Slave motor used to drive the flywheels */
  WPI_TalonFX m_indexMotor;
  /** < Motor used to drive the ball indexer */

  /**
   * A digital sensor that reads true when a ball reaches the end of the
   * shooter mechanism and is positioned against the flywheel
   */
  private DigitalInput m_ballLimitSensor;

  /**
   * Sensor that reads true when a ball is present in the 'ready' position at the
   * upper end of the shooter
   */
  private DigitalInput m_ballSensorMiddle;

  /**
   * Sensor that reads true when a ball is present in the lower end of the shooter
   */
  private DigitalInput m_ballSensorLower;

  /** Flywheel motor PID controller parameters */
  public TalonPIDConfig m_PIDConfig;

    // PID coefficients (starting point)
    // Small initial kFF and kP values, probably just big enough to do *something* 
    // and *probably* too small to overdrive an untuned system.
    private double kFF = 0.02;
    private double kP = 0.04;
    private double kI = 0;
    private double kD = 0;
    private double kIz = 0;
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;
    private double kMaxOutput = 1.0;
    private double kMinOutput = -1.0;
    private double maxRPM = 6300;     // free speed of Falcon 500 is listed as 6380
    private double m_rate_RPMpersecond = 1e10;    // 10 million effectively disables rate limiting

  /////////////////////////////////////////////////////////////////////////////
  /** Creates a new Shooter. */
  public Shooter() {
    // Set up ball sensors
    m_ballLimitSensor = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputUpper);
    m_ballSensorMiddle = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputMiddle);
    m_ballSensorLower = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputLower);

    m_PIDConfig = new TalonPIDConfig(kDefault_kP, kDefault_kI, kDefault_kD, kDefault_kF,
                                     kPIDLoopIdx, kTimeoutMs);

    // Configure shooter motors
    m_flywheelMotorMaster = new WPI_TalonFX(Constants.MotorID.flywheelMaster);
    //m_flywheelMotorSlave = new WPI_TalonFX(Constants.MotorID.flywheelSlave);
    m_indexMotor = new WPI_TalonFX(Constants.MotorID.indexMotor);
    m_flywheelMotorMaster.setNeutralMode(NeutralMode.Coast);
    //m_flywheelMotorSlave.setNeutralMode(NeutralMode.Coast);

    // Configure the flywheel master motor
    m_flywheelMotorMaster.setInverted(kInvertFlywheelMotor);

    // set PID coefficients
		m_flywheelMotorMaster.config_kP(kPIDLoopIdx, m_PIDConfig.kP, kTimeoutMs);
		m_flywheelMotorMaster.config_kI(kPIDLoopIdx, m_PIDConfig.kI, kTimeoutMs);
		m_flywheelMotorMaster.config_kD(kPIDLoopIdx, m_PIDConfig.kD, kTimeoutMs);
    m_flywheelMotorMaster.config_kF(kPIDLoopIdx, m_PIDConfig.kF, kTimeoutMs);
    m_flywheelMotorMaster.config_IntegralZone(kPIDLoopIdx, kIz, kTimeoutMs);
    m_flywheelMotorMaster.configNominalOutputForward(0, kTimeoutMs);
    m_flywheelMotorMaster.configNominalOutputReverse(0, kTimeoutMs);
		m_flywheelMotorMaster.configPeakOutputForward(1, kTimeoutMs);
		m_flywheelMotorMaster.configPeakOutputReverse(-1, kTimeoutMs);

    // Configure the flywheel slave motor to follow the master and
    // invert its direction
    // We MAY break these out so they aren't following in the future to put spin on
    // the ball.
    // Currently there is no encoders on these, so it's pretty raw settings anyhow.
    //m_flywheelMotorSlave.follow(m_flywheelMotorMaster);
    //m_flywheelMotorSlave.setInverted(kInvertFlywheelMotor);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Methods used to detect the presence and position of balls in the shooter

  /** Returns true when a ball is up against the flywheels in the shooter */
  public boolean getBallLimitSensor() {
    return !m_ballLimitSensor.get();
  }

  /** Returns true when a ball is present in the upper end of the shooter */
  public boolean upperBallIsPresent() {
    return !m_ballSensorMiddle.get();
  }

  /** Returns true when a ball is present in the lower end of the shooter */
  public boolean lowerBallIsPresent() {
    return !m_ballSensorLower.get();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the number of balls presently detected in the shooter: 1 or 2 */
  public int numBallsDetected() {
    return (1);
    /*
     * int count = 0;
     * if (!m_ballSensorMiddle.get())
     * {
     * count += 1;
     * }
     * 
     * if (!m_ballSensorLower.get())
     * {
     * count += 1;
     * }
     * 
     * return count;
     */
  }

  /**
   * Modes that the ball indexer can be run in*/
  public enum BallIndexerMode {
    Stopped(0.0),
    /** < Ball indexer is stopped */
    FeedBall(0.4),
    /** < Run forward to feed balls into the shooter */
    ShootBall(0.6),
    /** < Run forward to move a ball into the flywheel */
    Reverse(-0.4);

    /** < Run indexer in reverse */

    private final double motorSpeed;

    private BallIndexerMode(double speed) {
      motorSpeed = speed;
    }

    public double getMotorSpeed() {
      return motorSpeed;
    }
  };

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Runs the ball indexer to move balls through the shooter
   * 
   * @param mode Mode to run the ball indexer in
   */
  public void runBallIndexer(BallIndexerMode mode) {
    double percentOutput = mode.getMotorSpeed();

    if (BallIndexerMode.Stopped != mode) {
      double invert = (kInvertBallIndexerMotor ? -1.0 : 1.0);
      percentOutput *= invert;
    }

    m_indexMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
  }


  /////////////////////////////////////////////////////////////////////////////
  /** Speeds the flywheel can be run at */
  public enum FlywheelSpeed {
    Stopped (0.0),         /** Turn off the flywheel */
    Low (3000),              /** Slowest muzzle velocity */
    Medium (4000),           /** Medium muzzle velocity */
    GreasedLightning (6300), /** Back away... not today */
    Autonomous (3000);  /** Muzzle velocity used in autonomous mode */
    //To-Do
    //How to setup variable sliders on shuffleboard to configure this
    //https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/configuring-widgets.html
    private final double value;
    private FlywheelSpeed(double val) {this.value = val;}
    public double value() { return value; }
  };


  /////////////////////////////////////////////////////////////////////////////
  /**
   * Runs or stops the flywheel in the shooter
   * 
   * @param speed Speed to run the flywheel at
   */
  public void runFlywheel(FlywheelSpeed speed) {
   // double invert = (kInvertFlywheelMotor ? -1.0 : 1.0);
    double motorSpeed = speed.value();// * invert;
    //m_flywheelMotorMaster.set(TalonFXControlMode.PercentOutput, motorSpeed);
    /*if(speed == FlywheelSpeed.Low){
      m_flywheelMotorMaster.set(TalonFXControlMode.PercentOutput, 0.7 * motorSpeed);
    }else{
      m_flywheelMotorMaster.set(TalonFXControlMode.PercentOutput, motorSpeed);
    }*/

    m_flywheelMotorMaster.set(TalonFXControlMode.Velocity, motorSpeed / ticks2RPm);
  }

public double getIndexPosition(){
  return m_indexMotor.getSelectedSensorPosition(0);
}
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Called each scheduler cycle to run the subsystem
   * 
   * @see SubsystemBase.periodic()
   */
  @Override
  public void periodic() {

    // In manual tele-operated mode:
    // * Left trigger drives flywheel
    // * Right trigger drives ball indexer
    if (RobotContainer.robotIsInManualTeleOpMode()) {
      double shooterAmount = RobotContainer.operatorController.leftTriggerPull();
      double indexAmount = RobotContainer.operatorController.rightTriggerPull() * (kInvertBallIndexerMotor ? -1.0 : 1.0)
          * BallIndexerMode.ShootBall.getMotorSpeed();

     // m_flywheelMotorMaster.set(TalonFXControlMode.PercentOutput, shooterAmount); // * 0.65);
      
      m_flywheelMotorMaster.set(TalonFXControlMode.Velocity, shooterAmount * maxRPM / ticks2RPm);
      m_indexMotor.set(TalonFXControlMode.PercentOutput, indexAmount);
      
    }
  }
  //Sonic Squirels flywheel tuner
  //https://github.com/FRC-Sonic-Squirrels/Flywheel-Tuner/blob/main/src/main/java/frc/robot/Robot.java
}
