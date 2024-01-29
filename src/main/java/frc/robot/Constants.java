// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.025;
    public static final double LEFT_Y_DEADBAND  = 0.025;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }
  
  // public static final class ArmConstants {
  //   public static final int kArmRotateMotor = 13;
  //   public static final int kManipulatorIntakeMotorL = 14;
  //   public static final int kManipulatorIntakeMotorR = 15;
  //   public static final int kMotorPort = 4;

  //   public static final double intakeSpeedOut = 1.00;
  //   public static final double intakeSpeedIn = 0.50;
  //   public static final double intakeSpeedHold = 0.062;
  //   public static final double posOffset = 72.5;
  //   public static final double posDrive = 190; //Was 200 see note in ArmRotateSubsystem.java
  //   public static final double posIntake = 90; //Was 132.5 see note in ArmRotateSubsystem.java
  // }
  public static final class AprilTagConstants {
  public static int ampID = 0;
  public static int speakerID = 0;
  public static int stageIDA = 0;
  public static int stageIDB = 0;
  public static int stageIDC = 0;
  
  }

}
