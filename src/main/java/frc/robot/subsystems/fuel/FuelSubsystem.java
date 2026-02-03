// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  
  // Lookup tables
  private final InterpolatingDoubleTreeMap launcherPower;
  
  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    // Initialize hardware
    leftIntakeLauncherMotor = new SparkMax(CANConstants.kLeftIntakeLauncherMotorID, MotorType.kBrushless);
    rightIntakeLauncherMotor = new SparkMax(CANConstants.kRightIntakeLauncherMotorID, MotorType.kBrushless);
    feederMotor = new SparkMax(CANConstants.kFeederMotorID, MotorType.kBrushed);
    
    // Configure motors
    configureFeederMotor();
    configureIntakeLauncherMotors();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize lookup table for shoot voltage based on distance
    // TO DO:
    // - populate with real data after testing
    launcherPower = new InterpolatingDoubleTreeMap();
    launcherPower.put(0.0, 0.0);
    launcherPower.put(5.0, 0.5);
    launcherPower.put(10.0, 1.0);

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
    feederConfig.smartCurrentLimit(80); // amps

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

    // configue intake/launcher motors
    launcherConfig
      .smartCurrentLimit(80) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kCoast);

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
   * Set roller motor to a specific voltage
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setIntakeLauncherRoller(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    leftIntakeLauncherMotor.set(clampedPower);
    rightIntakeLauncherMotor.set(clampedPower);
  }
  
  /**
   * Set feeder motor to a specific voltage
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setFeederRoller(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    feederMotor.set(clampedPower);
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
   * Command to spin up the launcher.
   * Chain with .timeout() to control length of spin up.
   * @return Command that runs roller at spin up speed for fuel
   */
  public Command spinUpCommand() {
    return run(() -> {
      setIntakeLauncherRoller(FuelConstants.kLauncherLaunchingPercent);
      setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent);
    }).withName("SpinUpLauncher");
  }
  
  /**
   * Command to pass fuel out of the launcher at a lower speed
   * @return Command that runs rollers at a speed for passing fuel
   */
  public Command passCommand() {
    return run(() -> {
      setIntakeLauncherRoller(FuelConstants.kLauncherPassingPercent);
      setFeederRoller(FuelConstants.kFeederPassingPercent);
    }).withName("PassFuel");
  }
  
  /**
   * Command to launch/shoot fuel
   * @return Command that runs rollers at launch speed for fuel
   */
  public Command launchCommand(DoubleSupplier distanceToHub) {
    return run(() -> {
      // get the distance to the hub
      double distance = distanceToHub.getAsDouble();

      // get the appropriate voltage from the lookup table based on distance
      if (distance > 0 && distance < 10) {
        setIntakeLauncherRoller(launcherPower.get(distance));
        setFeederRoller(FuelConstants.kFeederLaunchingPercent);
        return;
      }

      // unknown distance, use default launch voltage
      setIntakeLauncherRoller(FuelConstants.kLauncherLaunchingPercent);
      setFeederRoller(FuelConstants.kFeederLaunchingPercent);
    }).withName("LaunchFuel");
  }
  
  // ==================== Telemetry Methods ====================

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("FuelSubsystem");
    // builder.addDoubleProperty("Velocity (RPS)", () -> Utils.showDouble(getVelocityRPS()), null);
    // builder.addDoubleProperty("Voltage (V)", () -> Utils.showDouble(getVoltage()), null);
    // builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    // builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
  }
}
