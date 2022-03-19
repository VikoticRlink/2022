// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.fireBallSubcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.FlywheelSpeed;


///////////////////////////////////////////////////////////////////////////////
/** A command used to run the flywheel until it has attained the desired speed
 * 
 *  NOTE: If one or more balls are present in the Shooter, this command will
 *  execute forever.  It must be scheduled using a timeout decorator.
 */
public class SpinUpFlywheel extends CommandBase {
  Shooter m_shooterSubsystem;
  private FlywheelSpeed m_flywheelSpeed; /** Speed the flywheel will be run at */
  private int cycleCount = 0;

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * @param flywheelSpeed     Speed the flywheel will be run at
   * @param shooterSubsystem  Shooter subsystem used by the command
   */
  public SpinUpFlywheel(FlywheelSpeed flywheelSpeed, Shooter shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_flywheelSpeed = flywheelSpeed;
    addRequirements(shooterSubsystem);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cycleCount = 0;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.runFlywheel(m_flywheelSpeed);
    cycleCount += 1;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /////////////////////////////////////////////////////////////////////////////
  // Only return true if no balls are detected in the Shooter
  @Override
  public boolean isFinished() 
  { 
    return (cycleCount == 75);
    //return (m_shooterSubsystem.numBallsDetected() < 1);
  }
}