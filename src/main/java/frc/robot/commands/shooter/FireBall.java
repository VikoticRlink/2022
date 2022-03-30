// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.FlywheelSpeed;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.shooter.fireBallSubcommands.*;


///////////////////////////////////////////////////////////////////////////////
public class FireBall extends SequentialCommandGroup {

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * @param muzzleVelocity    Velocity to fire the ball at
   * @param shooterSubsystem  Shooter subsystem used by the command
   * @param fireButton  Button used to fire balls in tele-operated mode
  */
  public FireBall(FlywheelSpeed muzzleVelocity, Shooter shooterSubsystem, 
                  JoystickButton fireButton) {
    // Add commands in the order in which they will be carried out
    addCommands(
        new SpinUpFlywheel(muzzleVelocity, shooterSubsystem)
          //.perpetually()
          .withTimeout(1),
          
        new FeedFlywheel(shooterSubsystem, fireButton)
          .withTimeout(Constants.ShooterConstants.kFeedFlywheelTimeoutSeconds),
        new WaitCommand(0.25),
        new FeedFlywheel(shooterSubsystem, fireButton)
          .withTimeout(Constants.ShooterConstants.kFeedFlywheelTimeoutSeconds),
        new SpinUpFlywheel(FlywheelSpeed.Stopped, shooterSubsystem));

  }
}