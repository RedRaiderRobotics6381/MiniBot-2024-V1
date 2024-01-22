package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;


/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java
 */
public class LLDriveToAprilTagPosCmd extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   xController;
  private final PIDController   yController;
  private final PIDController   zController;
  private double visionObject;
  private double aprilTagID;
  
  public LLDriveToAprilTagPosCmd(SwerveSubsystem swerveSubsystem, double visionObject, double aprilTagID )
  {
    
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(this.swerveSubsystem);
    this.visionObject = visionObject;    
    
    xController = new PIDController(.25, 0.01, 0.0001);
    yController = new PIDController(0.0625, 0.00375, 0.0001);
    zController = new PIDController(0.0575,0.0, 0.000);

    xController.setIZone(0.1); //0.1 meters
    yController.setIZone(0.1); //0.1 meters
    zController.setIZone(0.5); //0.5 degrees

    xController.setTolerance(.01); //0.01 meters
    yController.setTolerance(.01); //0.01 meters
    zController.setTolerance(.5); //0.5 degrees
    
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
  
    LimelightHelpers.setPipelineIndex("", visionObject);
  
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    boolean tv = LimelightHelpers.getTV("");
    double fid = LimelightHelpers.getFiducialID("");
    
    if (tv == true){ //if the limelight sees a target
      if (fid == aprilTagID){ //if the limelight sees the target we want
        RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0.25); //rumble the controller

        //tx is the angle that the limelight sees the target at
        double tz = LimelightHelpers.getTX("");

        //target[0] is x, target[1] is y, target[2] is z in meters
        double target[] = LimelightHelpers.getTargetPose_CameraSpace("");

        // This is the value in meters per second that is used to drive the robot
        double translationValx = MathUtil.clamp(-xController.calculate(target[0], 0.0), -.5 , .5);
        double translationValy = MathUtil.clamp(yController.calculate(target[2], 1.0), -.5 , .5); 
        double translationValz = MathUtil.clamp(zController.calculate(tz, 0.0), -2.0 , 2.0); 

        SmartDashboard.putNumber("Y Translation Value", target[0]);
        SmartDashboard.putNumber("X Translation Value", target[2]);
        SmartDashboard.putNumber("Z Translation Value", translationValz);

        //swerveSubsystem.drive(new Translation2d(translationValx, translationValy), translationValz, false);        
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(translationValx, translationValy, translationValz)); //drive the robot
      }
    }
    else{
      
      end(true);

    }
  
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return xController.atSetpoint() && yController.atSetpoint() && zController.atSetpoint(); //if the robot is at the setpoint
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {

    //swerveSubsystem.lock();
    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);

  }
}
