// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
// import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Vision.DriveToAprilTagPosCmd;
import frc.robot.commands.Vision.DriveToObjectCmd;
//import frc.robot.commands.Vision.LLDriveToAprilTagPosCmd;
//import frc.robot.commands.Vision.LLDriveToObjectCmd;
// import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDriveAng;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
// import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
// import frc.robot.subsystems.Secondary.ArmRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
//import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

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

  private final PhotonCamera photonCamera = new PhotonCamera("photonvision");

  public static XboxController driverXbox = new XboxController(0);
  public static XboxController engineerXbox = new XboxController(1);
  //private final Supplier<Pose2d> poseProvider = drivebase::getPose;

  private final SendableChooser<Command> autoChooser;

  // ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  // ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    
    // Register Named Commands
    // NamedCommands.registerCommand("autoBalance", new AutoBalanceCommand(drivebase));
    // NamedCommands.registerCommand("armDown", armRotateSubsystem.rotatePosCommand(ArmConstants.posIntake));
    // NamedCommands.registerCommand("armUp", armRotateSubsystem.rotatePosCommand(ArmConstants.posDrive));
    // NamedCommands.registerCommand("armIntake", armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedIn));
    // NamedCommands.registerCommand("armHold", armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedHold));
    // NamedCommands.registerCommand("armOut", armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedOut));
    NamedCommands.registerCommand("alignSpeaker", new DriveToAprilTagPosCmd(photonCamera, drivebase, 0, AprilTagConstants.speakerID));
    NamedCommands.registerCommand("alignAmp", new DriveToAprilTagPosCmd(photonCamera, drivebase, 0, AprilTagConstants.ampID));
    NamedCommands.registerCommand("alignNote", new DriveToObjectCmd(drivebase, 1));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                driverXbox.getPOV());

    // // Applies deadbands and inverts controls because joysticks
    // // are back-right positive while robot
    // // controls are front-left positive
    // // left stick controls translation
    // // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRightX(),
    //     () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND));

    // AbsoluteFieldDriveAng closedFieldAbsoluteDriveAng = new AbsoluteFieldDriveAng(drivebase,
    //                                                                   () ->
    //                                                                       MathUtil.applyDeadband(-driverXbox.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                   () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
    //                                                                                               OperatorConstants.LEFT_X_DEADBAND),
    //                                                                   () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4),
    //                                                                                               OperatorConstants.RIGHT_X_DEADBAND));


    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
        //!RobotBase.isSimulation() ? closedAbsoluteDriveAdv : closedAbsoluteDriveAdv);
        //!RobotBase.isSimulation() ? closedFieldAbsoluteDriveAng : closedFieldAbsoluteDriveAng);
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
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back


    new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));


    // new JoystickButton(engineerXbox, 1).onTrue(armRotateSubsystem.rotatePosCommand(ArmConstants.posDrive)); // 180 is vertical 
    // new JoystickButton(engineerXbox, 4).onTrue(armRotateSubsystem.rotatePosCommand(ArmConstants.posIntake)); //90 is horizontal 
    
    // new JoystickButton(engineerXbox,3 ).whileTrue(new ArmIntakeInCmd(armIntakeSubsystem));
    // new JoystickButton(engineerXbox,3 ).whileTrue(armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedIn));
    // new JoystickButton(engineerXbox, 3).onFalse(armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedHold));
    // new JoystickButton(engineerXbox,2 ).whileTrue(armIntakeSubsystem.ArmIntakeCmd(ArmConstants.intakeSpeedOut));
    // new JoystickButton(engineerXbox, 2).onFalse(armIntakeSubsystem.ArmIntakeCmd(0));

    //new JoystickButton(engineerXbox,7 ).whileTrue(new DriveGyro180Cmd(swerveSubsystem));

    // new JoystickButton(driverXbox, 5).whileTrue(new LLDriveToObjectCmd(drivebase, 0));
    // new JoystickButton(driverXbox, 5).whileTrue(new LLDriveToAprilTagPosCmd(drivebase, 0, 7));
    // new JoystickButton(driverXbox, 6).whileTrue(new LLDriveToAprilTagPosCmd(drivebase, 0, 7));
    // new JoystickButton(driverXbox, 5).whileTrue(new DriveToAprilTagPosCmd(photonCamera,
    //                                                                                    drivebase,
    //                                                                                    poseProvider,
    //                                                                                    0,
    //                                                                                    11,
    //                                                                                    60.0,
    //                                                                                    0.0,
    //                                                                                    0.0));
    new JoystickButton(driverXbox, 5).whileTrue(new DriveToAprilTagPosCmd(photonCamera,
                                                                                       drivebase,
                                                                                       0,
                                                                                       1));
    
    //new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox,
    //                    2).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
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
