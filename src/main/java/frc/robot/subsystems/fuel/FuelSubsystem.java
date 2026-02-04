// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class FuelSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax leftIntakeLauncherMotor;
  private final SparkMax rightIntakeLauncherMotor;
  private final SparkMax feederMotor;

  // Shooter velocity control
  private SparkClosedLoopController leftController;
  private SparkClosedLoopController rightController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  // Shooter state
  private double targetRPM = 0;
  
  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    // Initialize hardware
    leftIntakeLauncherMotor = new SparkMax(CANConstants.kLeftIntakeLauncherMotorID, MotorType.kBrushless);
    rightIntakeLauncherMotor = new SparkMax(CANConstants.kRightIntakeLauncherMotorID, MotorType.kBrushless);
    feederMotor = new SparkMax(CANConstants.kFeederMotorID, MotorType.kBrushed);
    
    // Configure motors
    configureFeederMotor();
    configureIntakeLauncherMotors();

    // Initialize controllers
    leftController = leftIntakeLauncherMotor.getClosedLoopController();
    rightController = rightIntakeLauncherMotor.getClosedLoopController();

    // Initialize encoders
    leftEncoder = leftIntakeLauncherMotor.getEncoder();
    rightEncoder = rightIntakeLauncherMotor.getEncoder();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Fuel", this);
    
    // Output initialization progress
    Utils.logInfo("Fuel subsystem initialized");
  }
  
  /**
   * Configure the feeder motor with all settings
   */
  private void configureFeederMotor() {
    SparkMaxConfig feederConfig = new SparkMaxConfig();

    // motor output
    feederConfig
      .smartCurrentLimit(80) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake);

    // apply configuration
    feederMotor.configure(
      feederConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }
  
  /**
   * Configure the intake/launcher motors with all settings
   */
  private void configureIntakeLauncherMotors() {
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    // configure intake/launcher motors
    launcherConfig
      .smartCurrentLimit(80) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kCoast);

    // closed-loop velocity control parameters
    launcherConfig.closedLoop
      .p(FuelConstants.kLauncherP)
      .i(0)
      .d(0);

    // feedforward parameters
    launcherConfig.closedLoop.feedForward
      .kS(FuelConstants.kLauncherKS)
      .kV(FuelConstants.kLauncherKV)
      .kA(FuelConstants.kLauncherKA);

    // configure the right motor
    rightIntakeLauncherMotor.configure(
      launcherConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    // invert the left motor
    launcherConfig.inverted(true);

    // configure the left motor
    leftIntakeLauncherMotor.configure(
      launcherConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );    
  }

  @Override
  public void periodic() {}
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set launcher motors to a percentage of max power
   * Used for intake/eject operations
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setIntakeLauncherRoller(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    targetRPM = 0; // Not using velocity control
    leftIntakeLauncherMotor.set(clampedPower);
    rightIntakeLauncherMotor.set(clampedPower);
  }
  
  /**
   * Set launcher motors to a specific velocity
   * Used for shooting operations
   * @param rpm RPM to set the intake/launcher motors to
   */
  private void setLauncherVelocity(double rpm) {
    // clamp RPM to valid range (NEO max ~6000 RPM) 
    double clampedRPM = MathUtil.clamp(rpm, 0, 6000); 

    // cache target RPM
    targetRPM = clampedRPM;
    
    // Use PID + feedforward for better tracking
    leftController.setSetpoint(clampedRPM, ControlType.kVelocity);
    rightController.setSetpoint(clampedRPM, ControlType.kVelocity);
  }
  
  /**
   * Set feeder motor to a specific voltage
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setFeederRoller(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    feederMotor.set(clampedPower);
  }
  
  /**
   * Check if launcher is at target speed
   * @return true if both motors are within tolerance of target RPM
   */
  private boolean isAtSpeed() {
    if (targetRPM == 0) {
      return false;
    }
    
    double leftError = Math.abs(leftEncoder.getVelocity() - targetRPM);
    double rightError = Math.abs(rightEncoder.getVelocity() - targetRPM);
    
    return leftError < FuelConstants.kLauncherToleranceRPM && 
           rightError < FuelConstants.kLauncherToleranceRPM;
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to stop the rollers
   * @return Command that stops the roller motors
   */
  public Command stopCommand() {
    return runOnce(() -> {
      setIntakeLauncherRoller(0);
      setFeederRoller(0);
      targetRPM = 0;
    }).withName("StopIntake");
  }
  
  /**
   * Command to intake fuel from the ground
   * @return Command that runs roller at intake speed for fuel
   */
  public Command intakeCommand() {
    return run(() -> {
      setIntakeLauncherRoller(FuelConstants.kIntakeIntakingPercent);
      setFeederRoller(FuelConstants.kFeederIntakingPercent);
    }).withName("IntakeFuel");
  }
  
  /**
   * Command to eject fuel out of the ground intake
   * @return Command that runs rollers at eject speed for fuel
   */
  public Command ejectCommand() {
    return run(() -> {
      setIntakeLauncherRoller(-1 * FuelConstants.kIntakeEjectPercent);
      setFeederRoller(FuelConstants.kFeederLaunchingPercent);
    }).withName("EjectFuel");
  }
  
  /**
   * Command to spin up the launcher to a specific RPM
   * Does NOT feed - use with feedCommand() or launchCommand()
   * @param rpm Target RPM for the launcher
   * @return Command that spins up the launcher and waits until at speed
   */
  public Command spinUpCommand(double rpm) {
    return run(() -> {
      setLauncherVelocity(rpm);
      setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent); // Hold fuel back
    })
    .until(this::isAtSpeed)
    .withName("SpinUpLauncher");
  }
  
  /**
   * Command to spin up the launcher using default launch RPM
   * @return Command that spins up the launcher
   */
  public Command spinUpCommand() {
    return spinUpCommand(FuelConstants.kLauncherLaunchingRPM);
  }
  
  /**
   * Command to feed fuel into the launcher
   * Assumes launcher is already at speed!
   * @return Command that runs the feeder to launch fuel
   */
  public Command feedCommand() {
    return run(() -> {
      // Maintain launcher speed and feed
      setLauncherVelocity(targetRPM); // Keep spinning
      setFeederRoller(FuelConstants.kFeederLaunchingPercent);
    }).withName("FeedFuel");
  }
  
  /**
   * Command to pass fuel out of the launcher at a lower speed
   * Hold button: spins up → automatically feeds when ready
   * Release button: stops everything immediately
   * @return Command that runs rollers at passing speed
   */
  public Command passCommand() {
    return run(() -> {
      // Spin up to passing speed
      setLauncherVelocity(FuelConstants.kLauncherPassingRPM);
      
      // Only feed when at speed - otherwise hold fuel back
      if (isAtSpeed()) {
        setFeederRoller(FuelConstants.kFeederPassingPercent);
      } else {
        setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent);
      }
    }).withName("PassFuel");
  }
  
  /**
   * Command to launch/shoot fuel with distance-based RPM
   * Hold button: spins up → automatically feeds when ready
   * Release button: stops everything immediately
   * @param distanceToHub Supplier that provides the distance to the hub in meters
   * @return Command that intelligently spins up then launches
   */
  public Command launchCommand(DoubleSupplier distanceToHub) {
    return run(() -> {
      // Calculate target RPM based on distance
      double distance = distanceToHub.getAsDouble();
      double rpm;
      if (distance >= 0 && distance <= 8.27) {
        rpm = FuelConstants.klauncherRPM.get(distance);
      } else {
        rpm = FuelConstants.kLauncherLaunchingRPM;
      }
      
      // Spin up the launcher
      setLauncherVelocity(rpm);
      
      // Only feed when at speed - otherwise hold fuel back
      if (isAtSpeed()) {
        setFeederRoller(FuelConstants.kFeederLaunchingPercent);
      } else {
        setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent);
      }
    }).withName("LaunchFuel");
  }
  
  /**
   * Command to launch at a fixed RPM
   * @return Command that launches fuel at default RPM
   */
  public Command launchCommand() {
    return launchCommand(() -> 5.0); // Default to mid-range distance
  }
  
  // ==================== Telemetry Methods ====================

  /**
   * Get the average velocity of both launcher motors
   * @return Average velocity in RPM
   */
  public double getVelocityRPM() {
    return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
  }
  
  /**
   * Get the average current draw of both launcher motors
   * @return Average current in amps
   */
  public double getCurrent() {
    return (leftIntakeLauncherMotor.getOutputCurrent() + 
            rightIntakeLauncherMotor.getOutputCurrent()) / 2.0;
  }
  
  /**
   * Get the average temperature of both launcher motors
   * @return Average temperature in Celsius
   */
  public double getTemperature() {
    return (leftIntakeLauncherMotor.getMotorTemperature() + 
            rightIntakeLauncherMotor.getMotorTemperature()) / 2.0;
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("FuelSubsystem");
    builder.addDoubleProperty("Left RPM", () -> Utils.showDouble(leftEncoder.getVelocity()), null);
    builder.addDoubleProperty("Right RPM", () -> Utils.showDouble(rightEncoder.getVelocity()), null);
    builder.addDoubleProperty("Velocity (RPM)", () -> Utils.showDouble(getVelocityRPM()), null);
    builder.addDoubleProperty("Target RPM", () -> Utils.showDouble(targetRPM), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
    builder.addBooleanProperty("At Speed", this::isAtSpeed, null);
  }
}
