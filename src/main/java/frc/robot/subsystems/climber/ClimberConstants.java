// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/**
 * Constants for the Climber subsystem
 * All values should be tuned based on your specific robot
 */
public final class ClimberConstants {  
  /**
   * Rev through bore encoder v2 resolution (ticks per motor revolution)
   * Rev through bore encoder v2 encoders report 8192 counts per revolution by default
   */
  public static final int kEncoderTicksPerRevolution = 8192;
  /**
   * Gear ratio from motor to wheel
   * If motor spins X times, wheel spins 1 time
   * Example: 10.71:1 gearbox means motor spins 10.71 times per wheel rotation
   */
  public static final double kGearRatio = 100.0; // TODO: Check your gearbox

  /**
   * Position conversion factor: converts encoder ticks to meters
   * Formula: (wheel circumference) / gear ratio
   */
  public static final double kPositionConversionFactor = 1 / kGearRatio;
  
  /**
   * Velocity conversion factor: converts encoder ticks/minute to meters/second
   * Formula: position conversion factor / 60 (to convert minutes to seconds)
   */
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

  // Motor power percentages
  public static final double kUpPercent   =  0.80;  // Power for extending climber
  public static final double kDownPercent = -0.80;  // Power for retracting climber (negative)
  
  // Soft limits (in encoder rotations)
  // Set these based on your climber's physical range of motion
  public static final double kUpperLimitRotations = 100.0;  // Maximum extension
  public static final double kLowerLimitRotations = 0.0;    // Fully retracted
  public static final double kHomeRotations       = 25.0;   // Home position
  public static final double kPositionTolerance   = 0.1;    // Tolerance for position control (in rotations)
  
  // Stall detection thresholds
  public static final double kStallCurrentThreshold  = 35.0;   // Amps - indicates motor is working hard
  public static final double kStallVelocityThreshold = 0.5;    // Rotations/sec - indicates motor not moving
}
