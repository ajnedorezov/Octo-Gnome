package gnome.commands;

import edu.wpi.first.wpilibj.command.Command;
import gnome.OI;
import gnome.loops.Navigation;
import gnome.subsystems.Drive;

/**
 *
 */
public class DriveMecanumMode extends Command {
  
    private OI mOI;
    private Drive mDrive;
    private Navigation mNavigation;
    

    public DriveMecanumMode() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      super("DriveMecanumMode");
      
      mOI = OI.getInstance();
      mDrive = Drive.getInstance();
      mNavigation = Navigation.getInstance();
      
      requires(mDrive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      mDrive.setMecanum();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      mDrive.mecanumDrive_Cartesian(mOI.mDriveLeftStickX, mOI.mDriveLeftStickY, mOI.mDriveRightStickX, mNavigation.getHeadingInDegrees());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
