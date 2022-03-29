
   
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.LoadBall;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.FireBall;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.FlywheelSpeed;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

///////////////////////////////////////////////////////////////////////////////
/** A command that laods and fires a ball using the Shooter subsystem
 * 
 *  This command sequentially executes the LoadBall and FireBall commands
 */
public class LoadAndFire extends SequentialCommandGroup {

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * @param muzzleVelocity    Speed to fire the ball
   * @param shooterSubsystem  Shooter subsystem the command will use
   * @param fireButton  Joystick button used to fire in manual tele-operated mode
  */
  public LoadAndFire(FlywheelSpeed muzzleVelocity, Shooter shooterSubsystem, 
                     JoystickButton fireButton) {

    addCommands(
        new LoadBall(shooterSubsystem),
        new FireBall(muzzleVelocity, shooterSubsystem, fireButton));
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Called when all sub-commands executed by this command have been executed
   *  or the command is interrupted */
  @Override
  public void end(boolean interrupted) {
    RobotContainer.RobotShooting = false;  // Clear shooting flag
    super.end(interrupted);
  }
}