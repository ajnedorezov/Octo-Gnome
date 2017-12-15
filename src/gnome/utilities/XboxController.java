package gnome.utilities;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Creates an XboxController with predefined methods for finding the state/value of Xbox controller
 * buttons/axes
 *
 * @author James Decker
 */
public class XboxController extends Joystick {

  // Xbox Controller Button mapping
  public final static int A_Button = 1, B_Button = 2, X_Button = 3, Y_Button = 4, Left_Bumper = 5,
      Right_Bumper = 6, Back_Button = 7, Start_Button = 8, Left_Stick = 9, Right_Stick = 10,

  // Xbox Axes mapping::
  Axis_LeftX = 0, Axis_LeftY = 1, Axis_LeftTrigger = 2, Axis_RightTrigger = 3, Axis_RightX = 4,
      Axis_RightY = 5,
  // Not reccomended: buggy and unreliable
  Axis_DPad = 6;
  
  private Joystick controller;
  public Button ButtonA, ButtonB, ButtonX, ButtonY, BumperLeft, BumperRight, ButtonStart, ButtonBack; 

  /**
   * Constructs a new XboxController with a specified port
   */
  public XboxController(final int port) {
    super(port);
    
    controller = new Joystick(port);
    ButtonA = new JoystickButton(controller, A_Button);
    ButtonB = new JoystickButton(controller, B_Button);
    ButtonX = new JoystickButton(controller, X_Button);
    ButtonY = new JoystickButton(controller, Y_Button);
    ButtonStart = new JoystickButton(controller, Start_Button);
    ButtonBack = new JoystickButton(controller, Back_Button);
    BumperRight = new JoystickButton(controller, Right_Bumper);
    BumperLeft = new JoystickButton(controller, Left_Bumper);
  }

  public boolean getStartButton() {
    return getRawButton(Start_Button);
  }

  public boolean getBackButton() {
    return getRawButton(Back_Button);
  }

  // Control Sticks::

  /**
   * returns the current state of the A Button
   */
  public boolean getAButton() {
    return getRawButton(A_Button);
  }

  /**
   * returns the current state of the B Button
   */
  public boolean getBButton() {
    return getRawButton(B_Button);
  }

  /**
   * returns the current state of the X_Button
   */
  public boolean getXButton() {
    return getRawButton(X_Button);
  }

  /**
   * returns the current state of the Y_Button
   */
  public boolean getYButton() {
    return getRawButton(Y_Button);
  }

  /**
   * returns the current state of the Left_Bumper
   */
  public boolean getLB() {
    return getRawButton(Left_Bumper);
  }

  /**
   * returns the current state of the Right_Bumper
   */
  public boolean getRB() {
    return getRawButton(Right_Bumper);
  }

  public boolean getLeftTrigger() {
    return getRawAxis(Axis_LeftTrigger) != 0;
  }

  public boolean getRightTrigger() {
    return getRawAxis(Axis_RightTrigger) != 0;
  }

  /**
   * returns the current state of the Left_Stick
   */
  public boolean getLeftStickClick() {
    return getRawButton(Left_Stick);
  }

  /**
   * returns the current state of the Right_Stick
   */
  public boolean getRightStickClick() {
    return getRawButton(Right_Stick);
  }

  /**
   * returns the current value of the Axis_LeftX
   */
  public double getLeftXAxis() {
    return getRawAxis(Axis_LeftX);
  }

  /**
   * returns the current value of the Axis_LeftY
   */
  public double getLeftYAxis() {
    return getRawAxis(Axis_LeftY);
  }

  /**
   * returns the current value of the Axis_RightX
   */
  public double getRightXAxis() {
    return getRawAxis(Axis_RightX);
  }

  /**
   * returns the current value of the Axis_RightY
   */
  public double getRightYAxis() {
    return getRawAxis(Axis_RightY);
  }
  
  /**
   * returns the current value of the Axis_LeftX
   */
  public double getDeadbandedLeftXAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_LeftX), deadband);
  }

  /**
   * returns the current value of the Axis_LeftY
   */
  public double getDeadbandedLeftYAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_LeftY), deadband);
  }

  /**
   * returns the current value of the Axis_RightX
   */
  public double getDeadbandedRightXAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_RightX), deadband);
  }

  /**
   * returns the current value of the Axis_RightY
   */
  public double getDeadbandedRightYAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_RightY), deadband);
  }


  /**
   * returns the current value of the Axis_DPad (NOTE:: Very buggy, not recommended)
   */
  public double getDPadAxis() {
    return getRawAxis(Axis_LeftX);
  }
  
  public static double applyDeadband(double value, double deadband) {
    if (value > -deadband && value < deadband) {
      return 0.0;
    } else {
      return value;
    }
  }
}
