package gnome.commands;

import edu.wpi.first.wpilibj.command.Command;
import gnome.OI;
import gnome.subsystems.Drive;

/**
 *
 */
public class DriveTankMode extends Command {
  
    private OI mOI;
    private Drive mDrive;
   

    public DriveTankMode() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      super("DriveTankMode");
      
      mOI = OI.getInstance();
      mDrive = Drive.getInstance();
      
      requires(mDrive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      mDrive.setTank();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      mDrive.setTankSpeedTurn(mOI.mDriveLeftStickY, mOI.mDriveRightStickX);
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
