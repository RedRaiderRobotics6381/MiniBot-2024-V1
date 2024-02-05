// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagConstants;
//import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

//import com.revrobotics.CANSparkMax;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  PhotonCamera camera = new PhotonCamera("photonvision");
  DigitalInput aSensor = new DigitalInput(0);

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    //LimelightHelpers.setLEDMode_ForceOff("");
    camera.setLED(VisionLEDMode.kOff);
    DriverStation.silenceJoystickConnectionWarning(true); // Disable joystick connection warning
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
        if (allianceColor.get() == Alliance.Red) {
          AprilTagConstants.ampID     = 5;
          AprilTagConstants.speakerID = 4;
          AprilTagConstants.stageIDA  = 13;
          AprilTagConstants.stageIDB  = 12;
          AprilTagConstants.stageIDC  = 11;
        }
        if (allianceColor.get() == Alliance.Blue) {
          AprilTagConstants.ampID     = 6;
          AprilTagConstants.speakerID = 7;
          AprilTagConstants.stageIDA  = 14;
          AprilTagConstants.stageIDB  = 15;
          AprilTagConstants.stageIDC  = 16;
        }
      }
    boolean aSensorState = aSensor.get();
    //System.out.println(aSensorState);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    camera.setLED(VisionLEDMode.kOff);
    //LimelightHelpers.setLEDMode_ForceOff("");
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    camera.setLED(VisionLEDMode.kDefault);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    //m_robotContainer.setMotorBrake(true);
    //ArmRotateSubsystem.ArmRotateSetpoint = 90;

    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kDefault);
    camera.setPipelineIndex(0);

    // LimelightHelpers.setCameraMode_Processor("null");
    // LimelightHelpers.setLEDMode_ForceOn("");
    // LimelightHelpers.setPipelineIndex("",0);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    // if(RobotContainer.engineerXbox.getRightY() > 0.1 || RobotContainer.engineerXbox.getRightY() < -0.1){
    //   ArmRotateSubsystem.m_armPIDController.setReference((ArmRotateSubsystem.ArmEncoder.getPosition()) +
    //                                                     (RobotContainer.engineerXbox.getRightY() * 20),
    //                                                     CANSparkMax.ControlType.kSmartMotion);                                                   
    // }
    if (RobotContainer.driverXbox.getRawButton(5) == true && RobotContainer.driverXbox.getRawButton(6) == true){
      System.out.println("HighSpd");
      //drivebase.maximumSpeed = Units.feetToMeters(14.5);
      //Constants.Drivebase.Max_Speed_Multiplier = 1;
      Constants.Drivebase.Max_Speed = 14.5;      
    }
    if (RobotContainer.driverXbox.getRawButton(5) == true && RobotContainer.driverXbox.getRawButton(6) == false){
      System.out.println("MedSpd");
      //drivebase.maximumSpeed = Units.feetToMeters(12.325);
      //Constants.Drivebase.Max_Speed_Multiplier = 0.75;
      Constants.Drivebase.Max_Speed = 12.325;
    }
    if (RobotContainer.driverXbox.getRawButton(5) == false && RobotContainer.driverXbox.getRawButton(6) == true){
      System.out.println("MedSpd");
      //drivebase.maximumSpeed = Units.feetToMeters(12.325);
      //Constants.Drivebase.Max_Speed_Multiplier = 0.75;
      Constants.Drivebase.Max_Speed = 12.325;
    }
    if (RobotContainer.driverXbox.getRawButton(5) == false && (RobotContainer.driverXbox.getRawButton(6) == false)){
      //drivebase.maximumSpeed = Units.feetToMeters(10.875);
      //Constants.Drivebase.Max_Speed_Multiplier = 0.5;
      Constants.Drivebase.Max_Speed = 10.875;
    }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
