package gnome.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gnome.Constants;
import gnome.RobotMap;
import gnome.commands.DriveTankMode;
import gnome.loops.Navigation;

/**
 *
 */
public class Drive extends Subsystem {

    private static Drive instance = new Drive();
    private static Navigation mNavigation;
  
    private boolean inTankMode = true;
    
    RobotDrive mDrive;
    Talon mLeftDriveMotor, mRightDriveMotor;
    CANTalon mFrontLeftMotor, mFrontRightMotor, mRearLeftMotor, mRearRightMotor;
    DoubleSolenoid mLeftShifter, mRightShifter;
      
    double mMaxOutput = 1.0;
    
  
    public static Drive getInstance() {
      return instance;
    }

    private Drive() {
      mNavigation = Navigation.getInstance();
      
      mLeftDriveMotor = new Talon(RobotMap.PWMs.DRIVE_LEFT_MOTOR);
      mRightDriveMotor = new Talon(RobotMap.PWMs.DRIVE_RIGHT_MOTOR);
      mRightDriveMotor.setInverted(true);
      
      mFrontLeftMotor = new CANTalon(RobotMap.CAN_IDs.LEFT_FRONT_MOTOR);
      mRearLeftMotor = new CANTalon(RobotMap.CAN_IDs.LEFT_REAR_MOTOR);
      mRearLeftMotor.setInverted(true);
      mFrontRightMotor = new CANTalon(RobotMap.CAN_IDs.RIGHT_FRONT_MOTOR);
      mRearRightMotor = new CANTalon(RobotMap.CAN_IDs.RIGHT_REAR_MOTOR);
      mRearLeftMotor.setInverted(true);
      
      mLeftShifter = new DoubleSolenoid(RobotMap.Solenoids.LEFT_POD_SHIFTER_A, RobotMap.Solenoids.LEFT_POD_SHIFTER_B);
      mRightShifter = new DoubleSolenoid(RobotMap.Solenoids.RIGHT_POD_SHIFTER_A, RobotMap.Solenoids.RIGHT_POD_SHIFTER_B);
      
      
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    protected static void normalize(double[] wheelSpeeds) {
      double maxMagnitude = Math.abs(wheelSpeeds[0]);
      for (int i = 1; i < Constants.Drive.MotorInds.NUM_MOTORS; i++) {
        double temp = Math.abs(wheelSpeeds[i]);
        if (maxMagnitude < temp) {
          maxMagnitude = temp;
        }
      }
      if (maxMagnitude > 1.0) {
        for (int i = 0; i < Constants.Drive.MotorInds.NUM_MOTORS; i++) {
          wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
        }
      }
    }
    
    /**
     * Rotate a vector in Cartesian space.
     */
    protected static double[] rotateVector(double x, double y, double angle) {
      double cosA = Math.cos(angle * (3.14159 / 180.0));
      double sinA = Math.sin(angle * (3.14159 / 180.0));
      double[] out = new double[2];
      out[0] = x * cosA - y * sinA;
      out[1] = x * sinA + y * cosA;
      return out;
    }
    
    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) {
      if (num > 1.0) {
        return 1.0;
      }
      if (num < -1.0) {
        return -1.0;
      }
      return num;
    }
    
    /**
     * Drive method for Mecanum wheeled robots.
     *
     * <p>A method for driving with Mecanum wheeled robots. There are 4 wheels on the robot, arranged
     * so that the front and back wheels are toed in 45 degrees. When looking at the wheels from the
     * top, the roller axles should form an X across the robot.
     *
     * <p>This is designed to be directly driven by joystick axes.
     *
     * @param x         The speed that the robot should drive in the X direction. [-1.0..1.0]
     * @param y         The speed that the robot should drive in the Y direction. This input is
     *                  inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
     * @param rotation  The rate of rotation for the robot that is completely independent of the
     *                  translation. [-1.0..1.0]
     * @param gyroAngle The current angle reading from the gyro. Use this to implement field-oriented
     *                  controls.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
      double xIn = x;
      double yIn = y;

      // Negate y for the joystick.
      yIn = -yIn;
      
      // Compenstate for gyro angle.
      double[] rotated = rotateVector(xIn, yIn, gyroAngle);
      xIn = rotated[0];
      yIn = rotated[1];

      double[] wheelSpeeds = new double[Constants.Drive.MotorInds.NUM_MOTORS];
      wheelSpeeds[Constants.Drive.MotorInds.LEFT_FRONT_MOTOR] = xIn + yIn + rotation;
      wheelSpeeds[Constants.Drive.MotorInds.RIGHT_FRONT_MOTOR] = -xIn + yIn - rotation;
      wheelSpeeds[Constants.Drive.MotorInds.LEFT_REAR_MOTOR] = -xIn + yIn + rotation;
      wheelSpeeds[Constants.Drive.MotorInds.RIGHT_REAR_MOTOR] = xIn + yIn - rotation;

      normalize(wheelSpeeds);
      set(wheelSpeeds);
    }

    /**
     * Drive method for Mecanum wheeled robots.
     *
     * <p>A method for driving with Mecanum wheeled robots. There are 4 wheels on the robot, arranged
     * so that the front and back wheels are toed in 45 degrees. When looking at the wheels from the
     * top, the roller axles should form an X across the robot.
     *
     * @param magnitude The speed that the robot should drive in a given direction.
     * @param direction The direction the robot should drive in degrees. The direction and maginitute
     *                  are independent of the rotation rate.
     * @param rotation  The rate of rotation for the robot that is completely independent of the
     *                  magnitute or direction. [-1.0..1.0]
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {

      // Normalized for full power along the Cartesian axes.
      magnitude = limit(magnitude) * Math.sqrt(2.0);
      // The rollers are at 45 degree angles.
      double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
      double cosD = Math.cos(dirInRad);
      double sinD = Math.sin(dirInRad);

      double[] wheelSpeeds = new double[Constants.Drive.MotorInds.NUM_MOTORS];
      wheelSpeeds[Constants.Drive.MotorInds.LEFT_FRONT_MOTOR] = (sinD * magnitude + rotation);
      wheelSpeeds[Constants.Drive.MotorInds.RIGHT_FRONT_MOTOR] = (cosD * magnitude - rotation);
      wheelSpeeds[Constants.Drive.MotorInds.LEFT_REAR_MOTOR] = (cosD * magnitude + rotation);
      wheelSpeeds[Constants.Drive.MotorInds.RIGHT_REAR_MOTOR] = (sinD * magnitude - rotation);

      normalize(wheelSpeeds);

      set(wheelSpeeds);
    }
    
    public void setTankSpeed(double leftMotorSpeed, double rightMotorSpeed) {
      set(leftMotorSpeed, rightMotorSpeed); // This is for the pow bot speed controllers
      set(leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed); // This is for octo-gnome speed controllers
    }

    /**
     * Set the speed and the turn of the robot, rather than each motor individually. Useful for arcade
     * drive.
     *
     * @param speed a double between -1.0 and 1.0, representing the full forward or full reverse.
     * @param turn a double between -1.0 and 1.0, representing turn anti-clockwise or clockwise.
     */
    public void setTankSpeedTurn(double speed, double turn) {
      double leftMotorSpeed = speed + turn;
      double rightMotorSpeed = speed - turn;
      
      set(leftMotorSpeed, rightMotorSpeed); // This is for the pow bot speed controllers
      set(leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed); // This is for octo-gnome speed controllers
    }
    
    
    /**
     * Set the drive to tank mode
     */
    public void setTank() {
      mLeftShifter.set(DoubleSolenoid.Value.kReverse);
      mRightShifter.set(DoubleSolenoid.Value.kReverse);
      
      resetEncoderPositions();

      if (!inTankMode) {
        inTankMode = true;
      }
    }
    
    public void setMecanum() {
      mLeftShifter.set(DoubleSolenoid.Value.kForward);
      mRightShifter.set(DoubleSolenoid.Value.kForward);

      resetEncoderPositions();
      
      if (inTankMode) {
        inTankMode = false;
      }
    }
    
    public boolean isTankMode(){
      return inTankMode;
    }
   
    
    private void set(double leftMotorSpeed, double rightMotorSpeed) {
      mLeftDriveMotor.set(leftMotorSpeed);
      mRightDriveMotor.set(rightMotorSpeed);
    }
    
    private void set(double leftFrontMotorSpeed, double leftRearMotorSpeed, double rightFrontMotorSpeed, double rightRearMotorSpeed) {
      mFrontLeftMotor.set(leftFrontMotorSpeed);
      mRearLeftMotor.set(leftRearMotorSpeed);
      mFrontRightMotor.set(rightFrontMotorSpeed);
      mRearRightMotor.set(rightRearMotorSpeed);
    }
    
    private void set(double[] wheelSpeeds){
      mFrontLeftMotor.set(wheelSpeeds[Constants.Drive.MotorInds.LEFT_FRONT_MOTOR] * mMaxOutput);
      mFrontRightMotor.set(wheelSpeeds[Constants.Drive.MotorInds.RIGHT_FRONT_MOTOR] * mMaxOutput);
      mRearLeftMotor.set(wheelSpeeds[Constants.Drive.MotorInds.LEFT_REAR_MOTOR] * mMaxOutput);
      mRearRightMotor.set(wheelSpeeds[Constants.Drive.MotorInds.RIGHT_REAR_MOTOR] * mMaxOutput);
    }
    
    private void resetEncoderPositions(){
      mFrontLeftMotor.setEncPosition(0);
      mRearLeftMotor.setEncPosition(0);
      mFrontRightMotor.setEncPosition(0);
      mFrontRightMotor.setEncPosition(0);
    }
    
    private double[] getEncoderPositions(double scaling){
      double[] wheelReadings = new double[Constants.Drive.MotorInds.NUM_MOTORS];
      wheelReadings[Constants.Drive.MotorInds.LEFT_FRONT_MOTOR] = mFrontLeftMotor.getEncPosition() * scaling;
      wheelReadings[Constants.Drive.MotorInds.RIGHT_FRONT_MOTOR] = mFrontRightMotor.getEncPosition() * scaling;
      wheelReadings[Constants.Drive.MotorInds.LEFT_REAR_MOTOR] = mRearLeftMotor.getEncPosition() * scaling;
      wheelReadings[Constants.Drive.MotorInds.RIGHT_REAR_MOTOR] = mRearRightMotor.getEncPosition() * scaling;
      return wheelReadings;
    }
    
    private double[] getTankPositions(){
      return getEncoderPositions(Constants.Drive.TANK_DRIVE_DISTANCE_PER_PULSE);
    }
    
    private double[] getMecanumPositions(){
      return getEncoderPositions(Constants.Drive.MECANUM_DRIVE_DISTANCE_PER_PULSE);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveTankMode());
    }
    
    /**
     * Logs this subsystem's specific properties to smartdashboard.
     */
    public void logToDashBoard() {
      double[] encoderVal = new double[Constants.Drive.MotorInds.NUM_MOTORS];
      if (inTankMode){
        encoderVal = getTankPositions();
      } else {
        encoderVal = getMecanumPositions();
      }
      SmartDashboard.putNumber("LeftFrontPositions", encoderVal[Constants.Drive.MotorInds.LEFT_FRONT_MOTOR]);
      SmartDashboard.putNumber("LeftRearPositions", encoderVal[Constants.Drive.MotorInds.LEFT_REAR_MOTOR]);
      SmartDashboard.putNumber("RightFrontPositions", encoderVal[Constants.Drive.MotorInds.RIGHT_FRONT_MOTOR]);
      SmartDashboard.putNumber("RightRearPositions", encoderVal[Constants.Drive.MotorInds.RIGHT_REAR_MOTOR]);
      
      SmartDashboard.putBoolean("InTankMode", inTankMode);
    }
}

