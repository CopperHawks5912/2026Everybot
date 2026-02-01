// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwerveConstants {
  public static final double MaxSpeed       = Units.feetToMeters( 10 ); // 14.5;
  public static final double WheelLockTime  = 10; // seconds
  public static final double RobotMass      = Units.lbsToKilograms(134);
  public static final Matter Chassis        = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);
  public static final double LoopTime       = 0.13; //s, 20ms + 110ms spark max velocity lag 
  
  public static final double Deadband       = 0.1;
  public static final double LeftYDeadbad   = 0.1;
  public static final double RightXDeadband = 0.1;
  public static final double TurnConstant   = 6;

  public static final double DefaultScaleTranslation = 0.8;
  public static final double SlowModeScaleTranlastion = 0.3;

  public static final double DefaultScaleRotation = 0.7;
  public static final double SlowModeScaleRotation = 0.3;
}
