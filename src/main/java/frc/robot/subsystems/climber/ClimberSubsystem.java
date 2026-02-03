// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
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
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize hardware
    climberMotor = new SparkMax(CANConstants.kClimberMotorID, MotorType.kBrushed);
    
    // Configure motor
    configureMotor();    
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Climber", this);
    
    // Output initialization progress
    Utils.logInfo("Climber subsystem initialized");
  }
  
  /**
   * Configure the roller motor with all settings
   */
  private void configureMotor() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    // configure the climber motor
    climbConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);

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
   * Set roller motor to a specific voltage
   * @param power Power to apply (-1.0 to 1.0)
   */
  private void setPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    climberMotor.set(clampedPower);
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to intake coral
   * @return Command that runs roller at intake speed for coral
   */
  public Command upCommand() {
    return run(() -> setPower(ClimberConstants.UpPercent))
      .withName("IntakeFuel");
  }
  
  /**
   * Command to intake coral
   * @return Command that runs roller at intake speed for coral
   */
  public Command downCommand() {
    return run(() -> setPower(ClimberConstants.DownPercent))
      .withName("IntakeFuel");
  }
  
  /**
   * Command to stop the roller
   * @return Command that stops the roller motor
   */
  public Command stopCommand() {
    return runOnce(() -> setPower(0))
      .withName("StopClimber");
  }
  
  // ==================== Telemetry Methods ====================
  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ClimberSubsystem");
    // builder.addDoubleProperty("Velocity (RPS)", () -> Utils.showDouble(getVelocityRPS()), null);
    // builder.addDoubleProperty("Voltage (V)", () -> Utils.showDouble(getVoltage()), null);
    // builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    // builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
  }
}
