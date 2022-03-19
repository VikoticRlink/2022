// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.fireBallSubcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.FlywheelSpeed;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

///////////////////////////////////////////////////////////////////////////////
/** A command used to move a ball in the Shooter into the flywheel to shoot it
 * 
 *  NOTE: If no balls are present in the Shooter, this command does nothing.
 */
public class FeedFlywheel extends CommandBase {
  Shooter m_shooterSubsystem;   /** Shooter subsystem */
  public JoystickButton m_fireButton;  /** Joystick button used to shoot in manual mode */
  private PulseDetector m_pulseDetector;  /** Used to detect when a ball exits the Shooter */
  private boolean m_isDone;     /** Set to true when the command has finished */

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * 
   * @param shooterSubsystem  Shooter subsystem to use
   * @param fireButton  Joystick button used to shoot balls in manual mode
   */
  public FeedFlywheel(Shooter shooterSubsystem, JoystickButton fireButton) {
    m_shooterSubsystem = shooterSubsystem;
    m_pulseDetector = new PulseDetector();
    m_fireButton = fireButton;

    addRequirements(shooterSubsystem);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pulseDetector.reset();

    // Flag the command as finished if there are no balls detected in the Shooter
    m_isDone = (m_shooterSubsystem.numBallsDetected() < 1);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If there are no balls detected in the Shooter, do nothing
    if (m_shooterSubsystem.numBallsDetected() < 1) {
      return;
    }

    m_shooterSubsystem.runBallIndexer(Shooter.BallIndexerMode.ShootBall);

    // When this command is executed from a button clicked in the Dashboard, no
    // button is present to read.  This is handled here by detecting when the
    // button is null
    boolean fireButtonIsPressed = false;
    if (m_fireButton != null) {
      fireButtonIsPressed = m_fireButton.get();
    }

    // In Tele-operated mode, the behavior of this command should be to fire
    // balls from the hopper as long as balls are present and the joystick
    // button is held down
    if (RobotState.isTeleop()) {
      if (fireButtonIsPressed) {
        // Check if all the sensors are false
        m_isDone = !(m_shooterSubsystem.lowerBallIsPresent() ||
            m_shooterSubsystem.upperBallIsPresent() ||
            m_shooterSubsystem.getBallLimitSensor());
      }
      else {
        boolean upperIndexSensor = m_shooterSubsystem.getBallLimitSensor();
        m_isDone = m_pulseDetector.process(upperIndexSensor);
      }
    }

    // In autonomous mode, this command will rely on a timeout

  }

  /////////////////////////////////////////////////////////////////////////////
  // When the command ends or is interrupted, stop the flywheel and ball indexer
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.runBallIndexer(Shooter.BallIndexerMode.Stopped);
    m_shooterSubsystem.runFlywheel(FlywheelSpeed.Stopped);
    m_isDone = true;
  }

  ///////////////////////////////////////////////////////////////////////////////
  /* Called after each time the scheduler runs the execute() method to determine
   * whether the command has finished.
   * 
   * Returns true after a ball was fired or if no balls are detected in the shooter
   */
  @Override
  public boolean isFinished() {
    return (m_shooterSubsystem.numBallsDetected() < 1) || m_isDone;
  }
}