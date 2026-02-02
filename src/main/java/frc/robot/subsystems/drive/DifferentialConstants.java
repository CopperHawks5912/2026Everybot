// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DifferentialConstants {
  // Drive control constants
  public static final double kTranslationScaling = 1.0; // Scale for maximum speed of the robot
  public static final double kRotationScaling = 1.0; // Scale for maximum rotational speed of the robot
  public static final double kTranslationalSlewRateLimit = 3.0; // meters per second squared
  public static final double kRotationalSlewRateLimit = 3.0; // radians per second squared
  public static final double kJoystickDeadband = 0.05;

  // Motion constraints (used by PathPlanner)
  public static final double kMaxSpeedMetersPerSecond = 4.0; // Max translational speed
  public static final double kMaxAccelMetersPerSecondSq = 3.0; // Max translational acceleration
  public static final double kMaxAngularSpeedRadsPerSecond = Math.PI; // Max rotational speed
  public static final double kMaxAngularAccelRadsPerSecondSq = Math.PI; // Max rotational acceleration

  // Physical characteristics
  public static final double kMaxDriveVelocityAt12VoltsMPS = 4.0; // Max velocity of the robot in meters per second
  public static final double kWheelCOF = 0.8; // Coefficient of friction for the wheels
  public static final double kRobotMassKg = Units.lbsToKilograms(100.0); // Mass of the robot
  public static final double kRobotMOI = 5.0; // Moment of inertia of the robot
  public static final double kTrackWidthMeters = Units.inchesToMeters(20.0); // Distance between left and right wheels
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // Diameter of the wheels
  public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0; // Radius of the wheels
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  public static final double kEncoderResolution = 4096; // Encoder counts per revolution

  // Drive PID + Feedforward
  public static final double kP = 1.00;  // usually 0.5 - 2.0, Start with 1.0
  public static final double kS = 0.15;  // usually 0.1 - 0.3, Static friction, start with 0.15
  public static final double kV = 2.50;  // usually 2.0 - 3.0, Velocity FF, start with 2.5
  public static final double kA = 0.30;  // usually 0.2 - 0.5, Acceleration FF, start with 0.3

  // ------------------------------------------------------------
  // Autobuilder RobotConfig for PathPlanner
  // ------------------------------------------------------------
  public static final RobotConfig kRobotConfig = new RobotConfig(
    kRobotMassKg,
    kRobotMOI,
    new ModuleConfig(
      kWheelRadiusMeters,
      kMaxDriveVelocityAt12VoltsMPS,
      kWheelCOF, 
      DCMotor.getCIM(1),
      60.0,
      2
    )
  );
}
