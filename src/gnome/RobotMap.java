package gnome;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
  public static class CAN_IDs {
	public static int LEFT_FRONT_MOTOR = 2;
	public static int LEFT_REAR_MOTOR = 3;
	public static int RIGHT_FRONT_MOTOR = 4;
	public static int RIGHT_REAR_MOTOR = 5;
  }
  
  public static class PWMs {
	public static int DRIVE_LEFT_MOTOR = 0;
	public static int DRIVE_RIGHT_MOTOR = 1;
  }

  public static class Solenoids {
    public static int LEFT_POD_SHIFTER_A = 0;
	public static int LEFT_POD_SHIFTER_B = 1;
	public static int RIGHT_POD_SHIFTER_A = 2;
    public static int RIGHT_POD_SHIFTER_B = 3;
  }
	
  public static class DigitalInputs {
    public static final int DRIVE_LEFT_ENCODER_1 = 0;
    public static final int DRIVE_LEFT_ENCODER_2 = 1;
    public static final int DRIVE_RIGHT_ENCODER_1 = 2;
    public static final int DRIVE_RIGHT_ENCODER_2 = 3;
  }
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
