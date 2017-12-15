package gnome;

public class Constants {
  
  public static class XBOXController {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double DEAD_BAND = 0.3;
  }
  
  public static class Drive {
    public static class MotorInds {
      public static int NUM_MOTORS = 4;
      public static int LEFT_FRONT_MOTOR = 0;
      public static int LEFT_REAR_MOTOR = 1;
      public static int RIGHT_FRONT_MOTOR = 2;
      public static int RIGHT_REAR_MOTOR = 3;
    }
    
    public static double MECANUM_WHEEL_DIAM = 6.0;
    public static double TANK_WHEEL_DIAM = 4.0;
    public static double MECANUM_DRIVE_DISTANCE_PER_PULSE = Math.PI * MECANUM_WHEEL_DIAM / 4096.0;  // Conversion for Mecanum wheels on Octo-Gnome
    public static double TANK_DRIVE_DISTANCE_PER_PULSE = Math.PI * TANK_WHEEL_DIAM / 4096.0;  // Conversion for Tank wheels on Octo-Gnome
    public static double DRIVE_DISTANCE_PER_PULSE = (Math.PI * 4) / 255; // Pow Bot Encoders
  }

}
