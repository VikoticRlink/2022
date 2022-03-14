package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBaseNew extends SubsystemBase {
  // The motors on the left side of the drive.
  WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.MotorID.leftDriveMaster);
  WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.MotorID.rightDriveMaster);
  WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.MotorID.leftDriveSlave);
  WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.MotorID.rightDriveSlave);

  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(leftMaster, leftSlave);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(rightMaster, rightSlave);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  /*private final Encoder m_leftEncoder = 
      new Encoder(
        Constants.DriveConstants.kLeftEncoderPorts[0],
        Constants.DriveConstants.kLeftEncoderPorts[1],
        Constants.DriveConstants.kLeftEncoderReversed);
*/
  // The right-side drive encoder
  /*private final Encoder m_rightEncoder =
      new Encoder(
      Constants.DriveConstants.kRightEncoderPorts[0],
      Constants.DriveConstants.kRightEncoderPorts[1],
      Constants.DriveConstants.kRightEncoderReversed);
*/
  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private static double DrivePowerModifer=1;

  /** Creates a new DriveSubsystem. */
  public DriveBaseNew() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse, rightMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse);

    if(RobotContainer.driverController.bumpRight.get()){
      DrivePowerModifer = Constants.MotorScaler.DriveMidLimit;
      SmartDashboard.putString("Speed", "Medium");
    }else{
      if(RobotContainer.driverController.bumpLeft.get()){
        DrivePowerModifer = Constants.MotorScaler.DriveSlowLimit ;
        SmartDashboard.putString("Speed", "Slow");
      }else{
        DrivePowerModifer = Constants.MotorScaler.DriveStandardLimit;
        SmartDashboard.putString("Speed", "Normal");
      }
    }

    if(RobotState.isEnabled() && RobotState.isTeleop()){
      if(RobotContainer.DriveDirection == RobotContainer.RobotDirection.Forward){
        m_drive.tankDrive(DrivePowerModifer * RobotContainer.driverController.rightStickY(), DrivePowerModifer * RobotContainer.driverController.leftStickY());
    
      }else{
         m_drive.tankDrive(-1 * DrivePowerModifer * RobotContainer.driverController.leftStickY(), -1 * DrivePowerModifer * RobotContainer.driverController.rightStickY());
      }
    }


  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity()*10, rightMaster.getSelectedSensorVelocity()*10);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0, 0, 0);
    leftMaster.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse + rightMaster.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  //public Encoder getLeftEncoder() {
  //  return m_leftEncoder;
  //}

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  //public Encoder getRightEncoder() {
  //  return m_rightEncoder;
  //}

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}