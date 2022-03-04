// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.*;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //UI systems are defined here...
  public static XboxController DriverController, OperatorController;

  private JoystickButton OperatorA, OperatorB, OperatorX, OperatorY, OperatorlBump, OperatorrBump, OperatorBack, OperatorStart;
  private JoystickButton DriverA, DriverB, DriverX, DriverY, DriverlBump, DriverrBump, DriverBack, DriverStart;
  public static boolean ManualControl = false;
  public static Integer DriveDirection = 1;
  public static boolean isRedAlliance = false;

  // The robot's subsystems are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static Climber m_Climber = new Climber();
  public static DriveBase m_DriveBase = new DriveBase();
  public static Intake m_Intake = new Intake();
  public static Lighting m_Lighting = new Lighting();
  public static Shooter m_Shooter = new Shooter();
  public static Dashboard m_Dashboard = new Dashboard();
  public static tRex m_tRex = new tRex();

  //The robot's commands are defined here...
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Auto m_autonomous = new Auto();
  private final AutoRedOne Red1 = new AutoRedOne();
  private final AutoRedTwo Red2 = new AutoRedTwo();
  private final AutoRedThree Red3 = new AutoRedThree();
  private final AutoBlueOne Blue1 = new AutoBlueOne();
  private final AutoBlueTwo Blue2 = new AutoBlueTwo();
  private final AutoBlueThree Blue3 = new AutoBlueThree();
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_chooser.setDefaultOption("Simple Auto", m_autonomous);
    m_chooser.addOption("Red 1", Red1);
    m_chooser.addOption("Red 2", Red2);
    m_chooser.addOption("Red 3", Red3);
    m_chooser.addOption("Blue 1", Blue1);
    m_chooser.addOption("Blue 2", Blue2);
    m_chooser.addOption("Blue 3", Blue3);   
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DriverController = new XboxController(0);
    OperatorController = new XboxController(1);

    OperatorA = new JoystickButton(OperatorController, 1);
    OperatorB = new JoystickButton(OperatorController, 2);
    OperatorX = new JoystickButton(OperatorController, 3);
    OperatorY = new JoystickButton(OperatorController, 4);
    OperatorlBump = new JoystickButton(OperatorController, 5);
    OperatorrBump = new JoystickButton(OperatorController, 6);
    OperatorBack = new JoystickButton(OperatorController, 7);
    OperatorStart = new JoystickButton(OperatorController, 8);

    OperatorStart.whenPressed(new ManualModeToggle());
   // OperatorX.whenPressed(new LoadAndFire(m_Shooter));
    OperatorA.whenHeld(new IntakeBall(RobotContainer.m_Intake));
    
    DriverStart = new JoystickButton(DriverController, 8);
    DriverStart.whenPressed(new ToggleFrontOfBot());
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}



//private final XboxController m_joystick = new XboxController(0);
//  final JoystickButton l2 = new JoystickButton(m_joystick, 9);
//  final JoystickButton r2 = new JoystickButton(m_joystick, 10);
//  final JoystickButton l1 = new JoystickButton(m_joystick, 11);
//  final JoystickButton r1 = new JoystickButton(m_joystick, 12);
//  r1.whenPressed(new PrepareToPickup(m_claw, m_wrist, m_elevator));
//  r2.whenPressed(new Pickup(m_claw, m_wrist, m_elevator));
//  l1.whenPressed(new Place(m_claw, m_wrist, m_elevator));
//  l2.whenPressed(new Autonomous(m_drivetrain, m_claw, m_wrist, m_elevator));
// The XboxController class provides named indicies for each of the buttons that you can access with XboxController.Button.kX.value. The rumble feature of the controller can be controlled by using XboxController.setRumble(GenericHID.RumbleType.kRightRumble, value). Many users do a split stick arcade drive that uses the left stick for just forwards / backwards and the right stick for left / right turning.