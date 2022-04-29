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

package frc.robot.subsystems.driveBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.Constants;

///////////////////////////////////////////////////////////////////////////////
/**
 * This subsystem manages motors and odometry used to move the robot around the arena.
 */
public class WestCoastDriveSubsystem extends SubsystemBase {

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
  
  /**
   * Time (Seconds) that motor current must exceed maximum before current limiting occurs
   */
  private static final double kMotorCurrentLimitHoldoffSec = 0.05;


  /////////////////////////////////////
  // Kinematics/Odometry Constants
  /////////////////////////////////////

  /** Maximum speed the robot should move at (meters per second) */
  public static final double kMaxSpeedMetersPerSec = 3.0;
  /** Maximum angular speed the robot should have (rotations per second) */
  public static final double kMaxAngularSpeedRotPerSec = 1 * Math.PI;  // Half a rotation per second

  /** Width of the west coast drive tracks in inches */
  public static final double kGearBox = 8.45; // Defined 2/27/2022
  private static final double kTrackWidthInches = 27.16535; // Measured 2/27/2022
  private static final double kWheelDiameterInches = 2.004; // Measured 2/27/2022
  private static final int kFalconTicksPerRevolution = 2048; // Falcon encoder tick count
  
  private static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);
  private static final double kWheelRadiusMeters = Units.inchesToMeters(2.004);
  private static final int kEncoderTicksPerRevolution = 2048; // Falcon encoder tick count

  public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double)kFalconTicksPerRevolution
  * kGearBox;
  0.69

  /** Time (Seconds) it takes to ramp motors from neutral to full output */
  // private static final double kMotorRampTimeSec = 0.1;

  //////////////////////////////////
  /// *** ATTRIBUTES ***
  //////////////////////////////////

  // The motors on the left side of the drive base.
  public WPI_TalonFX m_leftMaster = new WPI_TalonFX(Constants.MotorID.leftDriveMaster);
  WPI_TalonFX m_leftSlave = new WPI_TalonFX(Constants.MotorID.leftDriveSlave);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMaster,
      m_leftSlave);

  // The motors on the right side of the drive base.
  public WPI_TalonFX m_rightMaster = new WPI_TalonFX(Constants.MotorID.rightDriveMaster);
  WPI_TalonFX m_rightSlave = new WPI_TalonFX(Constants.MotorID.rightDriveSlave);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMaster,
      m_rightSlave);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  //////////////////////////////////////////
  // DriveBase Kinematics and Odometry
  //////////////////////////////////////////

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      );

  // Differential drive controller
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Gyro sensor referenced for odometry
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  // private static double DrivePowerModifer = 1;

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the subsystem */
  public WestCoastDriveSubsystem() {
    ConfigureMotors();

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

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
        m_leftMaster.getSelectedSensorPosition()
            * Constants.DriveConstants.kEncoderDistancePerPulse,
        m_rightMaster.getSelectedSensorPosition()
            * Constants.DriveConstants.kEncoderDistancePerPulse);
    /*
     * if (RobotContainer.driverController.bumpRight.get()) { DrivePowerModifer =
     * Constants.MotorScaler.DriveMidLimit; SmartDashboard.putString("Speed", "Medium"); } else { if
     * (RobotContainer.driverController.bumpLeft.get()) { DrivePowerModifer =
     * Constants.MotorScaler.DriveSlowLimit; SmartDashboard.putString("Speed", "Slow"); } else {
     * DrivePowerModifer = Constants.MotorScaler.DriveStandardLimit;
     * SmartDashboard.putString("Speed", "Normal"); } }
     * 
     * if (RobotState.isEnabled() && RobotState.isTeleop()) { if (RobotContainer.DriveDirection ==
     * RobotContainer.RobotDirection.Forward) { m_drive.tankDrive(DrivePowerModifer *
     * RobotContainer.driverController.rightStickY(), DrivePowerModifer *
     * RobotContainer.driverController.leftStickY());
     * 
     * } else { m_drive.tankDrive(-1 * DrivePowerModifer *
     * RobotContainer.driverController.leftStickY(), -1 * DrivePowerModifer *
     * RobotContainer.driverController.rightStickY()); } m_drive.feed(); }
     */

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
  /**
   * Resets odometry to a specified pose
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
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  Voltage to apply to the left side of the drive base
   * @param rightVolts Voltage to apply to the right side of the drive base
   */
  public void tankDrive(double Right, double Left) {
    m_drive.tankDrive(Right, Left);
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

    // m_rightSlave.setStatusFramePeriod(1, 255);
    // m_rightSlave.setStatusFramePeriod(2, 255);
    // m_leftSlave.setStatusFramePeriod(1, 255);
    // m_leftSlave.setStatusFramePeriod(2, 255);

  }

  public void LimitMotors(boolean shouldLimit) {
    // Configure current limiting
    SupplyCurrentLimitConfiguration limitConfig = new SupplyCurrentLimitConfiguration(shouldLimit, // Enable/disable
                                                                                                   // current
                                                                                                   // limiting
        kMotorCurrentLimitAmps, // Current limit to apply (Amps)
        kMotorCurrentLimitThresholdAmps, // Threshold for current limiting
        kMotorCurrentLimitHoldoffSec // Time to wait before applying current limiting
    );

    m_leftMaster.configSupplyCurrentLimit(limitConfig);
    m_rightMaster.configSupplyCurrentLimit(limitConfig);
    m_leftSlave.configSupplyCurrentLimit(limitConfig);
    m_rightSlave.configSupplyCurrentLimit(limitConfig);
    // Configure current ramping (seconds required to ramp from neutral to full
    // output)
    // m_leftMaster.configOpenloopRamp(kMotorRampTimeSec);
    // m_leftSlave.configOpenloopRamp(kMotorRampTimeSec);
    // m_rightMaster.configOpenloopRamp(kMotorRampTimeSec);
    // m_rightSlave.configOpenloopRamp(kMotorRampTimeSec);
  }
}