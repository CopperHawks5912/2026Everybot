// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PWMConstants;
import frc.robot.util.Utils;

public class ClimberSubsystem extends SubsystemBase {
  // Hardware
  private final TalonFX climberMotor;
  
  // Control requests (reusable objects)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize hardware
    climberMotor = new TalonFX(PWMConstants.ClimberID);
    
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
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Motor outputs
    config.MotorOutput
      .withDutyCycleNeutralDeadband(0.001)  // 0.1% deadband (tight control)
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive);
    
    // Current limits (hardcoded here for safety)
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)   // Enable supply limits
      .withSupplyCurrentLimit(60)                 // Peak current spike limit in Amps
      .withSupplyCurrentLowerLimit(40)       // Continuous current limit in Amps
      .withSupplyCurrentLowerTime(0.5)        // Time until lower current in seconds
      .withStatorCurrentLimitEnable(true)   // Enable stator limits
      .withStatorCurrentLimit(80);                // Max stator current in Amps (prevents overheating)

    // Voltage compensation
    config.Voltage
      .withPeakForwardVoltage(12)                   // Max voltage when running motor forward
      .withPeakReverseVoltage(-12)                                        // Max voltage when running motor in reverse
      .withSupplyVoltageTimeConstant(0.02);  // Voltage filter time constant in seconds

    // Velocity PID (runs on onboard motor controller, tunable in constants)
    config.Slot0
      .withKP(0.2)    // Proportional gain
      .withKI(0)      // Integral gain
      .withKD(0)      // Derivative gain
      .withKS(0)      // Static feedforward
      .withKV(0)      // Velocity feedforward
      .withKA(0);     // Acceleration feedforward
    
    // Apply the configuration to the motor
    climberMotor.getConfigurator().apply(config);

    // -------------------------------------------------------
    // OPTIMIZE CAN STATUS FRAMES for reduced lag
    // -------------------------------------------------------

    // HIGH PRIORITY - Critical for control (100Hz = 10ms)
    climberMotor.getVelocity().setUpdateFrequency(100.0);     // Velocity feedback
    climberMotor.getPosition().setUpdateFrequency(100.0);     // Position feedback
    
    // MEDIUM PRIORITY - Useful for monitoring (50Hz = 20ms)
    climberMotor.getMotorVoltage().setUpdateFrequency(50.0);  // Motor voltage
    climberMotor.getSupplyCurrent().setUpdateFrequency(50.0); // Supply current
    climberMotor.getTorqueCurrent().setUpdateFrequency(50.0); // Stator/torque current
    
    // LOW PRIORITY - Reduce CAN traffic (4Hz = 250ms)
    climberMotor.getDeviceTemp().setUpdateFrequency(4.0);     // Temperature

    // Optimize CAN bus utilization
    climberMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {}
  
  // ==================== Sensor Methods ====================

  
  // ==================== Control Methods ====================
  
  /**
   * Set roller motor to a specific voltage
   * @param voltage Voltage to apply (-12.0 to 12.0)
   */
  private void setVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
    climberMotor.setControl(voltageRequest.withOutput(clampedVoltage));
  }
  
  /**
   * Stop the roller motor
   */
  private void stop() {
    setVoltage(0);
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to intake coral
   * @return Command that runs roller at intake speed for coral
   */
  public Command upCommand() {
    return run(() -> setVoltage(ClimberConstants.UpVoltage))
      .withName("IntakeFuel");
  }
  
  /**
   * Command to intake coral
   * @return Command that runs roller at intake speed for coral
   */
  public Command downCommand() {
    return run(() -> setVoltage(ClimberConstants.DownVoltage))
      .withName("IntakeFuel");
  }
  
  /**
   * Command to stop the roller
   * @return Command that stops the roller motor
   */
  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopIntake");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Get current motor velocity in rotations per second
   * @return velocity in RPS
   */
  private double getVelocityRPS() {
    return climberMotor.getVelocity().getValueAsDouble();
  }
  
  /**
   * Get current motor supply current
   * @return current in amps
   */
  private double getCurrent() {
    return climberMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Get current motor voltage
   * @return voltage in volts
   */
  private double getVoltage() {
    return climberMotor.getMotorVoltage().getValueAsDouble();
  }
  
  /**
   * Get motor temperature
   * @return temperature in Celsius
   */
  private double getTemperature() {
    return climberMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ClimberSubsystem");
    builder.addDoubleProperty("Velocity (RPS)", () -> Utils.showDouble(getVelocityRPS()), null);
    builder.addDoubleProperty("Voltage (V)", () -> Utils.showDouble(getVoltage()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
  }
}
