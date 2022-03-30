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
  private static final boolean kInvertFlywheelMotor = true;

  /** Set to true to reverse the direction of the ball indexer motor */
  private static final boolean kInvertBallIndexerMotor = true;

  // FalconFX reports velocity in counts per 100ms
  // 1 revolution = 2048 counts
  // 1 minutes = 60 * 10 * 100ms
  // conversion is  600  / 2048
  private double ticks2RPm = 600.0 / 2048.0;

  private boolean IndexerDone = false;
  //////////////////////////////////
  /// *** ATTRIBUTES ***
  //////////////////////////////////
  WPI_TalonFX m_flywheelMotor;
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

    // PID coefficients (starting point)
    // Small initial kFF and kP values, probably just big enough to do *something* 
    // and *probably* too small to overdrive an untuned system.
    private double kFF = 0.055;
    private double kP = 0.22;
    private double kI = 0.002;
    private double kD = 0;
    private double kIz = 100; 
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    private double IndexkFF = 0.02;
    private double IndexkP = 0.04;
    private double IndexkI = 0.0;
    private double IndexkD = 0;
    private double IndexkIz = 0; 
    final int IndexkPIDLoopIdx = 0;
    final int IndexkTimeoutMs = 30;

    private double maxRPM = 6300;     // free speed of Falcon 500 is listed as 6380
    private boolean ShooterAtSpeed=false;

  /////////////////////////////////////////////////////////////////////////////
  /** Creates a new Shooter. */
  public Shooter() {
    m_flywheelMotor = new WPI_TalonFX(Constants.MotorID.flywheel);
    m_indexMotor = new WPI_TalonFX(Constants.MotorID.indexMotor);
    m_ballLimitSensor = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputUpper);
    m_ballSensorMiddle = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputMiddle);
    m_ballSensorLower = new DigitalInput(Constants.ShooterConstants.BallSensorDigitalInputLower);
    m_flywheelMotor.setNeutralMode(NeutralMode.Coast);
    // Configure the flywheel master motor
    m_flywheelMotor.setInverted(kInvertFlywheelMotor);

    // set Flywheel PID coefficients
    m_flywheelMotor.config_kF(kPIDLoopIdx, kFF, kTimeoutMs);
		m_flywheelMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		m_flywheelMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		m_flywheelMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    m_flywheelMotor.config_IntegralZone(kPIDLoopIdx, kIz, kTimeoutMs);
    m_flywheelMotor.configNominalOutputForward(0, kTimeoutMs);
    m_flywheelMotor.configNominalOutputReverse(0, kTimeoutMs);
		m_flywheelMotor.configPeakOutputForward(1, kTimeoutMs);
		m_flywheelMotor.configPeakOutputReverse(-1, kTimeoutMs);

        // set Indexer PID coefficients
        m_indexMotor.config_kF(kPIDLoopIdx, IndexkFF, IndexkTimeoutMs);
        m_indexMotor.config_kP(kPIDLoopIdx, IndexkP, IndexkTimeoutMs);
        m_indexMotor.config_kI(kPIDLoopIdx, IndexkI, IndexkTimeoutMs);
        m_indexMotor.config_kD(kPIDLoopIdx, IndexkD, IndexkTimeoutMs);
        m_indexMotor.config_IntegralZone(kPIDLoopIdx, IndexkIz, IndexkTimeoutMs);
        m_indexMotor.configNominalOutputForward(0, IndexkTimeoutMs);
        m_indexMotor.configNominalOutputReverse(0, IndexkTimeoutMs);
        m_indexMotor.configPeakOutputForward(1, IndexkTimeoutMs);
        m_indexMotor.configPeakOutputReverse(-1, IndexkTimeoutMs);
    
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
    Reverse(-0.4),
    ShootOneBall(-20000),
    Backoff(8600);

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

    if (mode == BallIndexerMode.ShootOneBall || mode == BallIndexerMode.Backoff){
      m_indexMotor.set(TalonFXControlMode.Position, mode.getMotorSpeed()); //clean this up to full position
      if (mode == BallIndexerMode.ShootOneBall ){
      if(m_indexMotor.getSelectedSensorPosition()<=mode.getMotorSpeed()){
        IndexerDone = true;
      }else{
        IndexerDone = false;
      }
    }else{
      if(m_indexMotor.getSelectedSensorPosition()>=mode.getMotorSpeed()){
        IndexerDone = true;
      }else{
        IndexerDone = false;
      }
      }
    }else{
      if (BallIndexerMode.Stopped != mode) {
        double invert = (kInvertBallIndexerMotor ? -1.0 : 1.0);
        percentOutput *= invert;
      }
      m_indexMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
    }
  }


  /////////////////////////////////////////////////////////////////////////////
  /** Speeds the flywheel can be run at */
  public enum FlywheelSpeed {
    Stopped (0.0),         /** Turn off the flywheel */
    Low (2000),              /** Slowest muzzle velocity 2000 or so*/
    Medium (8000),           /** Medium muzzle velocity */
    GreasedLightning (4200), /** Back away... not today 4200*/
    Autonomous (2000);  /** Muzzle velocity used in autonomous mode */
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
    //m_flywheelMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);
    /*if(speed == FlywheelSpeed.Low){
      m_flywheelMotor.set(TalonFXControlMode.PercentOutput, 0.7 * motorSpeed);
    }else{
      m_flywheelMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);
    }*/
    if((motorSpeed/ticks2RPm*.9)<=getShooterRPM() && getShooterRPM()<=(motorSpeed/ticks2RPm*1.1)){
      ShooterAtSpeed=true;
    }else{
      ShooterAtSpeed=false;
    }
    m_flywheelMotor.set(TalonFXControlMode.Velocity, motorSpeed / ticks2RPm);
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

     // m_flywheelMotor.set(TalonFXControlMode.PercentOutput, shooterAmount); // * 0.65);
      
      m_flywheelMotor.set(TalonFXControlMode.Velocity, shooterAmount * maxRPM / ticks2RPm);
      m_indexMotor.set(TalonFXControlMode.PercentOutput, indexAmount);
      
    } 
  }
  public double getShooterRPM(){
    return m_flywheelMotor.getSelectedSensorVelocity(0) * ticks2RPm;
  }
  public boolean isShooterAtSpeed(){
    return ShooterAtSpeed;
  }
  public void resetIndexEncoder(){
    m_indexMotor.setSelectedSensorPosition(0,0,0);
  }
  public boolean IndexAtLocation(){
    return IndexerDone; //ToDo make this look at encoder values.
  }
}
