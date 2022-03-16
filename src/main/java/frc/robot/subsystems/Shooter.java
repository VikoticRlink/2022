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
  private static final boolean kInvertFlywheelMotor = false;

  /** Set to true to reverse the direction of the ball indexer motor */
  private static final boolean kInvertBallIndexerMotor = false;

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

  /////////////////////////////////////////////////////////////////////////////
  /** Creates a new Shooter. */
  public Shooter() {
    m_flywheelMotorMaster = new WPI_TalonFX(Constants.MotorID.flywheelMaster);
    m_flywheelMotorSlave = new WPI_TalonFX(Constants.MotorID.flywheelSlave);
    m_indexMotor = new WPI_TalonFX(Constants.MotorID.indexMotor);
    m_ballLimitSensor = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputUpper);
    m_ballSensorMiddle = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputMiddle);
    m_ballSensorLower = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputLower);
    m_flywheelMotorMaster.setNeutralMode(NeutralMode.Coast);
    m_flywheelMotorSlave.setNeutralMode(NeutralMode.Coast);
    // Configure the flywheel master motor
    m_flywheelMotorMaster.setInverted(kInvertFlywheelMotor);

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
    FeedBall(0.7),
    /** < Run forward to feed balls into the shooter */
    ShootBall(1.0),
    /** < Run forward to move a ball into the flywheel */
    Reverse(-0.7);

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
    Low (1.0),              /** Slowest muzzle velocity */
    Medium (0.65),           /** Medium muzzle velocity */
    GreasedLightning (1.0), /** Back away... not today */
    Autonomous (0.65);  /** Muzzle velocity used in autonomous mode */
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
    double invert = (kInvertFlywheelMotor ? -1.0 : 1.0);
    double motorSpeed = speed.value() * invert;
    m_flywheelMotorMaster.set(TalonFXControlMode.PercentOutput, motorSpeed);
    if(speed == FlywheelSpeed.Low){
      m_flywheelMotorSlave.set(TalonFXControlMode.PercentOutput, - 0.5 * motorSpeed);
    }else{
      m_flywheelMotorSlave.set(TalonFXControlMode.PercentOutput, motorSpeed);
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

    // In manual tele-operated mode:
    // * Left trigger drives flywheel
    // * Right trigger drives ball indexer
    if (RobotContainer.robotIsInManualTeleOpMode()) {
      double shooterAmount = RobotContainer.operatorController.leftTriggerPull();
      double indexAmount = RobotContainer.operatorController.rightTriggerPull()
          * BallIndexerMode.ShootBall.getMotorSpeed();

      m_flywheelMotorMaster.set(TalonFXControlMode.PercentOutput, shooterAmount * 0.65);
      m_indexMotor.set(TalonFXControlMode.PercentOutput, indexAmount);
    }
  }
}
