package frc.robot.commands.Vision;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;


/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java
 */
public class LLDriveToObjectCmd extends Command
{

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   yController;
  private final PIDController   xController;
  private double visionObject;
  

  public LLDriveToObjectCmd(SwerveSubsystem swerveSubsystem, double visionObject)
  {
    this.swerveSubsystem = swerveSubsystem;
    yController = new PIDController(0.0625, 0.00375, 0.0001);
    xController = new PIDController(.25, 0.01, 0.0001);
    yController.setTolerance(.5);
    xController.setTolerance(.5);
    yController.setSetpoint(0.0);
    xController.setSetpoint(0.0);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
    this.visionObject = visionObject;
    
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(visionObject);


  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    

    NetworkTableEntry targetValid = table.getEntry("tv"); 
    //SmartDashboard.putBoolean("At Tolerance", yController.atSetpoint());
    double tv = targetValid.getDouble(0.0);

    // SmartDashboard.putNumber("Pipeline",tv);
    // SmartDashboard.putBoolean("TV", tv);
    if (tv == 1){
      RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0.25);
      NetworkTableEntry targetX = table.getEntry("tx");
      NetworkTableEntry targetY = table.getEntry("ty");
      double tx = targetX.getDouble(0.0);
      double ty = targetY.getDouble(0.0);

      //double throttle = RobotContainer.driverXbox.getLeftTriggerAxis();

      // This is the value in meters per second that is used to drive the robot

      double translationValy = MathUtil.clamp(yController.calculate(tx, 0.0), -.5 , .5); //* throttle, 2.5 * throttle);
      double translationValx = MathUtil.clamp(xController.calculate(ty, 0.0), -.5 , .5); //* throttle, 2.5 * throttle);
      SmartDashboard.putNumber("Y Translation Value", translationValy);
      SmartDashboard.putNumber("X Translation Value", translationValx);
      
      //swerveSubsystem.drive(new Translation2d(-translationValx, translationValy), 0.0, false, false);
      swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(-translationValx, translationValy,0.0));
      //swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, translationValy,-translationValx));

    }
    else{
      //swerveSubsystem.drive(new Translation2d(0, 0), 0.0, false, false);
      end(true);
      }
    
      // double translationVal = MathUtil.clamp(controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), -0.5,
    //                                        0.5);
    // swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true, false);
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
    return yController.atSetpoint() && xController.atSetpoint();
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
    swerveSubsystem.lock();
    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);
  }
}
