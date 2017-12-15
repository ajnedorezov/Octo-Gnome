package gnome;

import gnome.commands.DriveMecanumMode;
import gnome.commands.DriveTankMode;
import gnome.loops.Navigation;
import gnome.OI;
import gnome.utilities.XboxController;
import gnome.subsystems.Drive;
import gnome.Constants;



/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  public static OI instance = null;
  protected XboxController mDriverController;
  protected XboxController mOperatorController;
  
  public double mDriveLeftStickX, mDriveLeftStickY;
  public double mDriveRightStickX, mDriveRightStickY;
  
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
  
  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  OI() {
    // Instantiate the gamepads
    mDriverController = new XboxController(Constants.XBOXController.DRIVER_PORT);
    mOperatorController = new XboxController(Constants.XBOXController.OPERATOR_PORT);
    
    // Map Driver Buttons to relevant commands
    mDriverController.BumperLeft.whenPressed(new DriveTankMode());
    mDriverController.BumperRight.whenPressed(new DriveMecanumMode());
    
  }
  
  public void processInputs() {
    
    mDriveLeftStickX = mDriverController.getDeadbandedLeftXAxis(Constants.XBOXController.DEAD_BAND);
    mDriveLeftStickY = mDriverController.getDeadbandedLeftYAxis(Constants.XBOXController.DEAD_BAND);
    mDriveRightStickX = mDriverController.getDeadbandedRightXAxis(Constants.XBOXController.DEAD_BAND);
    mDriveRightStickY = mDriverController.getDeadbandedRightYAxis(Constants.XBOXController.DEAD_BAND);

  }
  
}
