// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/**
 * Constants for the Climber subsystem
 * All values should be tuned based on your specific robot
 */
public final class ClimberConstants {
  // Motor power percentages
  public static final double kUpPercent   =  0.80;  // Power for extending climber
  public static final double kDownPercent = -0.80;  // Power for retracting climber (negative)
  
  // Soft limits (in encoder rotations)
  // Set these based on your climber's physical range of motion
  public static final double kUpperLimitRotations = 100.0;  // Maximum extension
  public static final double kLowerLimitRotations = 0.0;    // Fully retracted (home position)
  public static final double kHomeRotations       = 25.0;   // Home position
  
  // Stall detection thresholds
  public static final double kStallCurrentThreshold  = 35.0;   // Amps - indicates motor is working hard
  public static final double kStallVelocityThreshold = 0.5;    // Rotations/sec - indicates motor not moving
}
