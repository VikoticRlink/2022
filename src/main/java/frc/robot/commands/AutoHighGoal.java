// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.driveStages.*;
import frc.robot.subsystems.DriveBaseNew;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.FlywheelSpeed;

public class AutoHighGoal extends SequentialCommandGroup {
  
  public AutoHighGoal(DriveBaseNew driveBaseSubsystem, Intake intakeSubsystem,
                   Shooter shooterSubsystem) {
    addCommands(
      new DriveStage0(driveBaseSubsystem, intakeSubsystem),
      new WaitCommand(2.25),
      new InstantCommand(intakeSubsystem::disableIntake, intakeSubsystem),
      new LoadAndFire(FlywheelSpeed.GreasedLightning, shooterSubsystem, null),
      new InstantCommand(driveBaseSubsystem::CoastMode, driveBaseSubsystem)
    );
  }
}
