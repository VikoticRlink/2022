package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/////////////////////////////////////////////////////////////////////////////
/** This utility class wraps the functionality of a joystick controller for
 *  driving the robot
 */
public class JoystickController extends Object {

  /** An object used to feather and deadband joystick values */
  public class JoystickProcessor extends Object {
    public static final double kDefaultSensitivity = 1.0;
    public static final double kDefaultDeadbandLower = 0.1;
    public static final double kDefaultDeadbandUpper = 0.95;

    /**< Sensitivity 0.0(off) to 1.0(linear) */
    public double sensitivity;  
    public double deadbandLower; /**< Deadband under which values are set to 0.0 */
    public double deadbandUpper; /**< Deadband above which values are set to 1.0 */

    /** Creates a JoystickConfig with default sensitivity and deadband */
    JoystickProcessor() {
      sensitivity = kDefaultSensitivity;
      deadbandLower = kDefaultDeadbandLower;
      deadbandUpper = kDefaultDeadbandUpper;
    }

    /////////////////////////////////////////////////////////////////////////////
    /** Method used to process a joystick value by 'feathering' and deadbanding it
     * @param joystickValue   Joystick value to feather in the range -1.0 to 1.0
     */
    public double process(double joystickvalue) {
      // Sensitivity is in values of 0 to 1 and it is cubing the input.
      double joystickOutput = joystickvalue;
      joystickOutput = ((1.0 - sensitivity) * joystickOutput)
                       + (sensitivity * Math.pow(joystickOutput, 3));

      double result = joystickvalue;
      double magnitude = Math.abs(joystickvalue);
      if (magnitude < deadbandLower) {
        result = 0.0;
      } else if (magnitude > deadbandUpper) {
        result = (joystickvalue > 0.0) ? 1.0 : -1.0;
      }
      return result;
    }
  }

  /** Object used to communicate with an Xbox controller */
  private XboxController m_controller;

  public JoystickButton A;           /** Xbox controller 'A' button */
  public JoystickButton B;           /** Xbox controller 'B' button */
  public JoystickButton X;           /** Xbox controller 'X' button */
  public JoystickButton Y;           /** Xbox controller 'Y' button */
  public JoystickButton bumpLeft;    /** Xbox controller left bumper button */
  public JoystickButton bumpRight;   /** Xbox controller right bumper button */
  public JoystickButton Back;        /** Xbox controller 'Back' button */
  public JoystickButton Start;       /** Xbox controller 'Start' button */
  public JoystickButton leftStickTrigger;   /** Trigger (push) on left joystick */
  public JoystickButton rightStickTrigger;  /** Trigger (push) on right joystick */

  // JoystickProcessor objects used to feather and deadband joystick controls
  public JoystickProcessor leftStickXProc;  /** Processor for left joystick X */
  public JoystickProcessor leftStickYProc;  /** Processor for left joystick Y */
  public JoystickProcessor rightStickXProc; /** Processor for right joystick X */
  public JoystickProcessor rightStickYProc; /** Processor for right joystick Y */
  public JoystickProcessor leftTriggerProc;    /** Processor for left trigger */
  public JoystickProcessor rightTriggerProc;   /** Processor for left trigger */


  /////////////////////////////////////////////////////////////////////////////
  /** Creates a JoysticController instance that communicates with an Xbox 
   * controller on a specified port
   * 
   * @param port    Port number of the Xbox controller the object will use
   */
  public JoystickController(int port) {
    m_controller = new XboxController(port);

    A = new JoystickButton(m_controller, XboxController.Button.kA.value);
    B = new JoystickButton(m_controller, XboxController.Button.kB.value);
    X = new JoystickButton(m_controller, XboxController.Button.kX.value);
    Y = new JoystickButton(m_controller, XboxController.Button.kY.value);
    bumpLeft = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    bumpRight = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    Back = new JoystickButton(m_controller, XboxController.Button.kBack.value);
    Start = new JoystickButton(m_controller, XboxController.Button.kStart.value);
    leftStickTrigger = new JoystickButton(m_controller, XboxController.Button.kLeftStick.value);
    rightStickTrigger = new JoystickButton(m_controller, XboxController.Button.kRightStick.value);

    leftStickXProc = new JoystickProcessor();
    leftStickYProc = new JoystickProcessor();
    rightStickXProc = new JoystickProcessor();
    rightStickYProc = new JoystickProcessor();
    leftTriggerProc = new JoystickProcessor();
    rightTriggerProc = new JoystickProcessor();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the feathered position of the left stick Y-axis as a value in the
   *  range -1.0 (bottom) to 1.0 (top) with a center position at 0.0 
   */
  public double leftStickX() {
    return leftStickXProc.process(m_controller.getLeftX());
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the feathered position of the left stick Y-axis as a value in the
   *  range -1.0 (bottom) to 1.0 (top) with a center position at 0.0 
   */
  public double leftStickY() {
    return leftStickYProc.process(m_controller.getLeftY());
  }


  /////////////////////////////////////////////////////////////////////////////
  /** Returns the feathered position of the left stick Y-axis as a value in the
   *  range -1.0 (bottom) to 1.0 (top) with a center position at 0.0 
   */
  public double rightStickX() {
    return rightStickXProc.process(m_controller.getLeftX());
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the feathered position of the left stick Y-axis as a value in the
   *  range -1.0 (bottom) to 1.0 (top) with a center position at 0.0 
   */
  public double rightStickY() {
    return rightStickYProc.process(m_controller.getLeftY());
  }


  /////////////////////////////////////////////////////////////////////////////
  /** Returns the amount of pull on the left trigger as a value from 0 to 1 */
  public double leftTriggerPull() {
      return leftTriggerProc.process(m_controller.getLeftTriggerAxis());
  }

  /////////////////////////////////////////////////////////////////////////////
  public double rightTriggerPull() {
      return rightTriggerProc.process(m_controller.getRightTriggerAxis());
  }
}