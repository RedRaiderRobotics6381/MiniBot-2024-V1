// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Vision.LLDriveToAprilTagPosCmd;
//import frc.robot.commands.Vision.LLDriveToAprilTagVelCmd;
import frc.robot.commands.Vision.LLDriveToObjectCmd;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDriveAng;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDriveAng;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
//import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  //XboxController driverXbox = new XboxController(0);
  public final static XboxController driverXbox = new XboxController(0);
  public final static XboxController engineerXbox = new XboxController(1);
  private final SendableChooser<Command> autoChooser;
  
  ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register Named Commands
    NamedCommands.registerCommand("autoBalance", new AutoBalanceCommand(drivebase));
    NamedCommands.registerCommand("armDown", armRotateSubsystem.rotatePosCommand(ArmConstants.posIntake));
    NamedCommands.registerCommand("armUp", armRotateSubsystem.rotatePosCommand(ArmConstants.posDrive));
    NamedCommands.registerCommand("armIntake", armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedIn));
    NamedCommands.registerCommand("armHold", armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedHold));
    NamedCommands.registerCommand("armOut", armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedOut));
    NamedCommands.registerCommand("alignCone", new LLDriveToObjectCmd(drivebase, 0));
    NamedCommands.registerCommand("alignCube", new LLDriveToObjectCmd(drivebase, 1));
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4));


    
    AbsoluteFieldDriveAng closedFieldAbsoluteDriveAng = new AbsoluteFieldDriveAng(drivebase,
                                                                         () ->
                                                                             MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                                                                    OperatorConstants.LEFT_Y_DEADBAND),
                                                                         () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
                                                                                                      OperatorConstants.LEFT_X_DEADBAND),
                                                                         () -> -driverXbox.getRightX());


    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedFieldAbsoluteDriveAng : closedFieldAbsoluteDriveAng);
    //drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right joystick x side to side
    //Axis 4 is right joystick y forward and back
    //Axis 5 is right trigger

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    //new JoystickButton(driverXbox, 2).whileTrue(new AutoBalanceCommand(drivebase)); 

    new JoystickButton(engineerXbox, 1).onTrue(armRotateSubsystem.rotatePosCommand(ArmConstants.posDrive)); // 180 is vertical 
    new JoystickButton(engineerXbox, 4).onTrue(armRotateSubsystem.rotatePosCommand(ArmConstants.posIntake)); //90 is horizontal 

    //new JoystickButton(engineerXbox,3 ).whileTrue(new ArmIntakeInCmd(armIntakeSubsystem));
    new JoystickButton(engineerXbox,3 ).whileTrue(armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedIn));
    new JoystickButton(engineerXbox, 3).onFalse(armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedHold));
    new JoystickButton(engineerXbox,2 ).whileTrue(armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedOut));
    new JoystickButton(engineerXbox, 2).onFalse(armIntakeSubsystem.ArmIntakeCmd(0));

    //new JoystickButton(engineerXbox,7 ).whileTrue(new DriveGyro180Cmd(swerveSubsystem));

    // new JoystickButton(driverXbox, 5).whileTrue(new LLDriveToObjectCmd(drivebase, 0));
    new JoystickButton(driverXbox, 5).whileTrue(new LLDriveToAprilTagPosCmd(drivebase, 0, 7));
    new JoystickButton(driverXbox, 6).whileTrue(new LLDriveToAprilTagPosCmd(drivebase, 0, 7));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    //return drivebase.getAutonomousCommand("New Path", true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
