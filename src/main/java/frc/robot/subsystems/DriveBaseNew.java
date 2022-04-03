package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.Constants;
import frc.robot.RobotContainer;


///////////////////////////////////////////////////////////////////////////////
/**
 * This subsystem manages motors and odometry used to move the robot around the arena.
 */
public class DriveBaseNew extends SubsystemBase {

  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////

  /** The number of motors used in the drive base subsystem */
  public static final int kNumDriveMotors = 4;

  /** System current budget allocated to the drive subsystem */
  public static final double kCurrentBudgetAmps = Constants.BatteryBudget.kDriveMotorCurrentAmps;

  /** Current limit (Amps) applied to each drive motor */
  public static final double kMotorCurrentLimitAmps = kCurrentBudgetAmps / kNumDriveMotors;

  /** Current limiting threshold (Amps) applied to each drive motor */
  private static final double kMotorCurrentLimitThresholdAmps = kMotorCurrentLimitAmps * 0.95;

  /** Time (Seconds) that motor current must exceed maximum before current limiting occurs */
  private static final double kMotorCurrentLimitHoldoffSec = 0.05;

  /** Time (Seconds) it takes to ramp motors from neutral to full output */
  private static final double kMotorRampTimeSec = 0.1;

  
  //////////////////////////////////
  /// *** ATTRIBUTES ***
  //////////////////////////////////

  // The motors on the left side of the drive base.
  public WPI_TalonFX m_leftMaster = new WPI_TalonFX(Constants.MotorID.leftDriveMaster);
  WPI_TalonFX m_leftSlave = new WPI_TalonFX(Constants.MotorID.leftDriveSlave);
  private final MotorControllerGroup m_leftMotors = 
      new MotorControllerGroup(m_leftMaster, m_leftSlave);

  // The motors on the right side of the drive base.
  public WPI_TalonFX m_rightMaster = new WPI_TalonFX(Constants.MotorID.rightDriveMaster);
  WPI_TalonFX m_rightSlave = new WPI_TalonFX(Constants.MotorID.rightDriveSlave);
  private final MotorControllerGroup m_rightMotors = 
      new MotorControllerGroup(m_rightMaster, m_rightSlave);

  // Differential drive controller
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Gyro sensor referenced for odometry
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private static double DrivePowerModifer = 1;

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the subsystem */
  public DriveBaseNew() {
    ConfigureMotors();

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

    // Reset the encoder count on drive motors
    resetEncoders();

    // Initialize odometry
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }


  /////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse,
        m_rightMaster.getSelectedSensorPosition()
            * Constants.DriveConstants.kEncoderDistancePerPulse);

    if (RobotContainer.driverController.bumpRight.get()) {
      DrivePowerModifer = Constants.MotorScaler.DriveMidLimit;
      SmartDashboard.putString("Speed", "Medium");
    }
    else {
      if (RobotContainer.driverController.bumpLeft.get()) {
        DrivePowerModifer = Constants.MotorScaler.DriveSlowLimit;
        SmartDashboard.putString("Speed", "Slow");
      }
      else {
        DrivePowerModifer = Constants.MotorScaler.DriveStandardLimit;
        SmartDashboard.putString("Speed", "Normal");
      }
    }

    if (RobotState.isEnabled() && RobotState.isTeleop()) {
      if (RobotContainer.DriveDirection == RobotContainer.RobotDirection.Forward) {
        m_drive.tankDrive(DrivePowerModifer * RobotContainer.driverController.rightStickY(),
            DrivePowerModifer * RobotContainer.driverController.leftStickY());

      }
      else {
        m_drive.tankDrive(-1 * DrivePowerModifer * RobotContainer.driverController.leftStickY(),
            -1 * DrivePowerModifer * RobotContainer.driverController.rightStickY());
      }
      m_drive.feed();
    }

  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the current estimated pose of the robot */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the current wheel speeds of the robot */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftMaster.getSelectedSensorVelocity() * 10,
        m_rightMaster.getSelectedSensorVelocity() * 10);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Resets odometry to a specified pose
   *
   * @param pose The pose to apply to odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd Forward movement being commanded
   * @param rot Rotation being commanded
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  Voltage to apply to the left side of the drive base
   * @param rightVolts Voltage to apply to the right side of the drive base
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Reset drive base encoders to zero on the current position */
  public void resetEncoders() {
    m_rightMaster.setSelectedSensorPosition(0, 0, 0);
    m_leftMaster.setSelectedSensorPosition(0, 0, 0);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the average distance indicated by left and right side encoders */
  public double getAverageEncoderDistance() {
    return (m_leftMaster.getSelectedSensorPosition()
        * Constants.DriveConstants.kEncoderDistancePerPulse
        + m_rightMaster.getSelectedSensorPosition()
            * Constants.DriveConstants.kEncoderDistancePerPulse)
        / 2.0;
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public Encoder getLeftEncoder() {
  // return m_leftEncoder;
  // }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
  // return m_rightEncoder;
  // }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the current heading of the robot in degrees (-180.0 to 180) */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the turn rate of the robot in degrees per second */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Sets drive motor neutral mode to 'Coast' */
  public void CoastMode() {
    m_rightMaster.setNeutralMode(NeutralMode.Coast);
    m_leftMaster.setNeutralMode(NeutralMode.Coast);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Sets drive motor neutral mode to 'Brake' */
  public void BrakeMode() {
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Configures drive subsystem motors */
  private void ConfigureMotors() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
    
    // Configure current limiting
    SupplyCurrentLimitConfiguration limitConfig = 
      new SupplyCurrentLimitConfiguration(
          true,                     // Enable current limiting
          kMotorCurrentLimitAmps,   // Current limit to apply (Amps)
          kMotorCurrentLimitThresholdAmps,  // Threshold for current limiting
          kMotorCurrentLimitHoldoffSec      // Time to wait before applying current limiting
          );

    m_leftMaster.configSupplyCurrentLimit(limitConfig);
    m_rightMaster.configSupplyCurrentLimit(limitConfig);
    m_leftSlave.configSupplyCurrentLimit(limitConfig);
    m_rightSlave.configSupplyCurrentLimit(limitConfig);

    // Configure current ramping (seconds required to ramp from neutral to full output)
   // m_leftMaster.configOpenloopRamp(kMotorRampTimeSec);
   // m_leftSlave.configOpenloopRamp(kMotorRampTimeSec);
   // m_rightMaster.configOpenloopRamp(kMotorRampTimeSec);
   // m_rightSlave.configOpenloopRamp(kMotorRampTimeSec);
  }
}