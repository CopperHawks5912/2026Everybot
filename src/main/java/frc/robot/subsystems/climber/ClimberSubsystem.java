// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class ClimberSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize hardware
    climberMotor = new SparkMax(CANConstants.kClimberMotorID, MotorType.kBrushed);
    
    // Configure motor
    configureMotor();
    
    // Initialize encoder
    climberEncoder = climberMotor.getEncoder();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Climber", this);
    
    // Output initialization progress
    Utils.logInfo("Climber subsystem initialized");
  }
  
  /**
   * Configure the climber motor with all settings
   */
  private void configureMotor() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    // configure the climber motor
    climbConfig
      .smartCurrentLimit(40) // amps
      .voltageCompensation(12) // Consistent behavior across battery voltage
      .idleMode(IdleMode.kBrake); // CRITICAL: Brake mode prevents falling

    // apply configuration
    climberMotor.configure(
      climbConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {}
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set climber motor power with safety limits
   * @param power Power to apply (-1.0 to 1.0)
   */
  private void setPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    
    // Safety: Stop at limits to prevent damage
    if (isAtUpperLimit() && clampedPower > 0) {
      climberMotor.set(0);
      return;
    }
    
    if (isAtLowerLimit() && clampedPower < 0) {
      climberMotor.set(0);
      return;
    }
    
    climberMotor.set(clampedPower);
  }
  
  /**
   * Reset the encoder position to zero
   */
  public void resetEncoder() {
    climberEncoder.setPosition(0);
  }
  
  // ==================== State Queries ====================
  
  /**
   * Get the current position of the climber
   * @return Position in rotations
   */
  public double getPosition() {
    return climberEncoder.getPosition();
  }
  
  /**
   * Get the current draw of the climber motor
   * @return Current in amps
   */
  public double getCurrent() {
    return climberMotor.getOutputCurrent();
  }
  
  /**
   * Get the temperature of the climber motor
   * @return Temperature in Celsius
   */
  public double getTemperature() {
    return climberMotor.getMotorTemperature();
  }
  
  /**
   * Check if climber is at upper soft limit
   * @return true if at or past upper limit
   */
  public boolean isAtUpperLimit() {
    return getPosition() >= ClimberConstants.kUpperLimitRotations;
  }
  
  /**
   * Check if climber is at lower soft limit
   * @return true if at or past lower limit
   */
  public boolean isAtLowerLimit() {
    return getPosition() <= ClimberConstants.kLowerLimitRotations;
  }
  
  /**
   * Check if climber is at home position
   * @return true if within tolerance of home position
   */
  public boolean isAtHomePosition() {
    return Math.abs(getPosition() - ClimberConstants.kHomeRotations) <= 0.1;
  }
  
  /**
   * Check if climber is stalled (high current, low velocity)
   * Useful for detecting when climber hits a hard stop
   * @return true if motor appears stalled
   */
  public boolean isStalled() {
    return Math.abs(getCurrent()) > ClimberConstants.kStallCurrentThreshold &&
           Math.abs(climberEncoder.getVelocity()) < ClimberConstants.kStallVelocityThreshold;
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to extend the climber upward
   * @return Command that runs climber up at configured speed
   */
  public Command upCommand() {
    return run(() -> setPower(ClimberConstants.kUpPercent))
      .withName("ClimberUp");
  }
  
  /**
   * Command to retract the climber downward
   * @return Command that runs climber down at configured speed
   */
  public Command downCommand() {
    return run(() -> setPower(ClimberConstants.kDownPercent))
      .withName("ClimberDown");
  }
  
  /**
   * Command to extend climber up until upper limit
   * @return Command that extends to upper limit then stops
   */
  public Command extendToLimitCommand() {
    return run(() -> setPower(ClimberConstants.kUpPercent))
      .until(this::isAtUpperLimit)
      .andThen(stopCommand())
      .withName("ExtendToLimit");
  }
  
  /**
   * Command to retract climber down until lower limit
   * @return Command that retracts to lower limit then stops
   */
  public Command retractToLimitCommand() {
    return run(() -> setPower(ClimberConstants.kDownPercent))
      .until(this::isAtLowerLimit)
      .andThen(stopCommand())
      .withName("RetractToLimit");
  }
  
  /**
   * Command the climber to move to the upright home position
   * @return Command that retracts to lower limit then stops
   */
  public Command homeCommand() {
    return run(() -> {
      if (getPosition() > ClimberConstants.kHomeRotations) {
        setPower(ClimberConstants.kDownPercent);
      } else {
        setPower(ClimberConstants.kUpPercent);
      }
    })
    .until(this::isAtHomePosition)
    .andThen(stopCommand())
    .withName("HomeClimber");
  }
  
  /**
   * Command to stop the climber
   * @return Command that stops the climber motor
   */
  public Command stopCommand() {
    return runOnce(() -> setPower(0))
      .withName("StopClimber");
  }
  
  /**
   * Command to reset encoder to zero at current position
   * Use this when climber is at a known position (e.g., fully retracted)
   * @return Command that resets the encoder
   */
  public Command resetEncoderCommand() {
    return runOnce(this::resetEncoder)
      .withName("ResetClimberEncoder");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ClimberSubsystem");
    builder.addDoubleProperty("Position (rotations)", this::getPosition, null);
    builder.addDoubleProperty("Current (A)", this::getCurrent, null);
    builder.addDoubleProperty("Temperature (C)", this::getTemperature, null);
    builder.addBooleanProperty("At Upper Limit", this::isAtUpperLimit, null);
    builder.addBooleanProperty("At Lower Limit", this::isAtLowerLimit, null);
    builder.addBooleanProperty("Stalled", this::isStalled, null);
  }
}
