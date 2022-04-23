///////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------\
|                                                                                                  |
|                                 ================================                                 |
|                                 **    TEAM 5290 - Vikotics    **                                 |
|                                 ================================                                 |
|                                                                                                  |
|                                  .             o°                                                |
|                                   °o          @@                                                 |
|                                    @@         o@@o                                               |
|                                   *@@ **@@@@** o@@*                                              |
|                                  *@@@@@@@@@@@@@@@@@*                                             |
|                                  @@@@@@@@@@@@@@@@@@@                                             |
|                                  @@@@@@@@@@@@@@@@@@*                                             |
|                                  @@@@@@@@@@@@@@@@@@                                              |
|                                   **@@@@@@@@@@@@@@@@**                                           |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@***@                                 |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@@@@o                                 |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@oo                          **       |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@@@@**                     *@@     *  |
|                                    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                 *@@@O     @@ |
|                                    @@@@@@@@@@#°@@@@@@@@@@@@@@@@@@@@@               *@@@@     *@@@|
|                                   *@@@o°°°°°   @@@@@@@@@@@@@@@@@@@@@@*             @@@@o*    @@@@|
|                                 *@@@@@@    .  .@@@@@@@@@@@@@@@@@@@@@@@@@*          @@@@@@@@@@@@@@|
|                               *@@@@@@@@.o#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@**        @@@@@@@@@@@@@|
|                              *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*      o@@@@@@@@@@* |
|                              @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@     @@@@@@@**    |
|                              @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*  @@@@@@@       |
|          ***@@@@@****        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ *@@@@@@*       |
|      *@@@@@@@@@@@@@@@@@*    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*@@@@@@@        |
|**** @@@@@@@@@@@@@@@@**@@@    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@         |
|@@@@@@@@@@@@@@@@@@@@     o@    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@         |
|@@@@@@@@@@@@@@@@@@@@      *    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@.        |
|@@@@@@@@@@@@@@@@@@@*            @@@@@@@@@@@@@@@@@@@@@@@@@@@@@*    *@@@@@@@@@@@@@@@@@@@@@@#        |
|@@@@@@@@@@@@@@@@@@              O@@@@@@@@@@@@@@@@@@@@@@@@@@@*        **@@@@@@@@@@@@@@@@@@o        |
|@@@@@  ******@@@@@               @@@@@@@@@@@@@@@@@@@@@@@@@@*            *@@@@@@@@@@@@@@@@         |
|****         @@@@@@              @@@@@@@@@@@@@@@@@@@@@@@@@                **@@@@@@@@@@@@          |
|             @@@@@@             *@@@@@@@@@@@@@@@@@@@@@@@@                    @@@@@@@@@@           |
|             *@@@@@@           @@@@@@@@@@@@@@@@@@@@@@@@@@                    @@@@@@@@*            |
|              @@@@@@****     *@@@@@@@@@@@@@@@@@@@@@@@@@@@                   @@@@@@@*              |
|               @@@@@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                  *@@@@@@@               |
|              @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                  @@@@@@@*               |
|             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                 *@@@@@@@                |
|             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                @@@@@@@                 |
|              @@@@@@@@@@@@@@@@@@**#@@@@@@@@@@@@@@@@@@@@@@@@              o@@@@@@*                 |
|              @@@@@@@@@@@@@**     @@@@@@@@@@@@@@@@@@@@@@@@@@*            oO@@@@*                  |
|               *@@@@@@@@@         @@@@@@@@@@@@@@@@@@@@@@@@@@@@*                                   |
|                  @@@@@@@         @@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                                  |
|                  *@@@@@@@        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                                 |
|                   @@@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                                |
|                    @@@@@@@*      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                               |
|                    @@@@@@@@@    #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                               |
|                     @@@@@@@@*   O@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                              |
|                      @@@@@@@@   °@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                             |
|                      *@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o                            |
|                                @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                            |
|                                oooooooooooooooooooooooooooooooooooooo                            |
\-------------------------------------------------------------------------------------------------*/


package frc.robot;

import frc.robot.commands.DriveByJoysticks;
import frc.robot.subsystems.driveBase.DriveBaseSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.lighting.LightingSubsystem;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.dashboard.*;
import frc.robot.subsystems.runtimeState.*;
import frc.robot.subsystems.*;



/////////////////////////////////////////////////////////////////////////////
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //----------------------------------
  // ROBOT SUBSYSTEMS
  //----------------------------------
  public JoystickSubsystem m_joystickSubsystem;
  public BotStateSubsystem m_botState;
  public ClimberSubsystem m_Climber;
  public T_rexSubsystem m_tRex;
  public DriveBaseSubsystem m_DriveBase;
  public ShooterSubsystem m_Shooter;
  public IntakeSubsystem m_Intake;
  public LightingSubsystem m_Lighting;
  public DashboardSubsystem m_Dashboard;

  //----------------------------------
  // COMMANDS
  //----------------------------------


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_joystickSubsystem = new JoystickSubsystem();
    m_botState = new BotStateSubsystem();
    m_DriveBase = new DriveBaseSubsystem();
    m_Lighting = new LightingSubsystem(m_botState);
    m_Climber = new ClimberSubsystem(m_botState, m_joystickSubsystem);
    m_tRex = new T_rexSubsystem(m_botState, m_joystickSubsystem);
    m_Shooter = new ShooterSubsystem(m_botState, m_joystickSubsystem);
    m_Intake = new IntakeSubsystem(m_botState, m_tRex, m_Shooter);
    m_Dashboard = new DashboardSubsystem(m_botState, m_DriveBase, m_Intake, m_Shooter);

    // Configure the button bindings
    m_joystickSubsystem.configureButtonBindings(m_botState, m_Shooter, m_Intake);

    // Make the drive base be driven by Joystick commands when not processing
    // another command
    m_DriveBase.setDefaultCommand(
        new DriveByJoysticks(m_botState, m_DriveBase, m_joystickSubsystem));
  }

}
