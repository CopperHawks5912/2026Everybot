// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;
import frc.robot.util.Utils;

public class DifferentialSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax leftLeaderMotor;
  private final SparkMax leftFollowerMotor;
  private final SparkMax rightLeaderMotor;
  private final SparkMax rightFollowerMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDrive drive;
  private final AHRS gyro;

  private final DifferentialDrivePoseEstimator poseEstimator;
  private final DifferentialDriveKinematics kinematics;

  // PID controllers for driving
  private final PIDController leftPIDController;
  private final PIDController rightPIDController;
  private final SimpleMotorFeedforward feedForward;

  // PID controller for aiming
  private final PIDController aimPIDController;

  // Slew rate limiters to make joystick inputs smoother
  private final SlewRateLimiter xSpeedLimiter;
  private final SlewRateLimiter rSpeedLimiter;

  // Vision subsystem for pose correction
  private final VisionSubsystem visionSubsystem;

  // Field visualization
  private final Field2d field2d = new Field2d();

  /**
   * Creates a new DifferentialSubsystem.
   */
  public DifferentialSubsystem(VisionSubsystem vision) {
    // Store the vision subsystem
    this.visionSubsystem = vision;

    // Initialize the slew rate limiters
    xSpeedLimiter = new SlewRateLimiter(DifferentialConstants.kTranslationalSlewRateLimit);
    rSpeedLimiter = new SlewRateLimiter(DifferentialConstants.kRotationalSlewRateLimit);

    // Create the gyro
    gyro = new AHRS(NavXComType.kMXP_SPI);

    // Initialize drive motors
    leftLeaderMotor = new SparkMax(CANConstants.kLeftDifferentialLeaderMotorID, null);
    leftFollowerMotor = new SparkMax(CANConstants.kLeftDifferentialFollowerMotorID, null);
    rightLeaderMotor = new SparkMax(CANConstants.kRightDifferentialLeaderMotorID, null);
    rightFollowerMotor = new SparkMax(CANConstants.kRightDifferentialFollowerMotorID, null);

    // Get the encoders
    leftEncoder = leftLeaderMotor.getEncoder();
    rightEncoder = rightLeaderMotor.getEncoder();

    // Initialize PID controllers for driving
    leftPIDController = new PIDController(DifferentialConstants.kP, 0, 0);
    rightPIDController = new PIDController(DifferentialConstants.kP, 0, 0);
    feedForward = new SimpleMotorFeedforward(
      DifferentialConstants.kS,
      DifferentialConstants.kV,
      DifferentialConstants.kA
    );

    // Initialize PID controller for aiming
    aimPIDController = new PIDController(3.5, 0.0, 0.15);
    aimPIDController.enableContinuousInput(-Math.PI, Math.PI);
    aimPIDController.setTolerance(Math.toRadians(2.0)); // 2 degree tolerance

    // Configure motors (do this before creating DifferentialDrive b/c left inverted motors)
    configureMotors();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeaderMotor, rightLeaderMotor);

    // set up kinematics
    kinematics = new DifferentialDriveKinematics(DifferentialConstants.kTrackWidthMeters);

    // set up pose estimator
    poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics, 
      gyro.getRotation2d(), 
      0.0, 
      0.0, 
      new Pose2d()
    );

    // Configure AutoBuilder for path following
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier
      this::driveRobotRelative, // Method that will drive the robot given ChassisSpeeds
      new PPLTVController(0.02), // 20ms periodic cycle
      DifferentialConstants.kRobotConfig, // Robot configuration
      Utils::isRedAlliance, // Method to flip path based on alliance color
      this // Reference to this subsystem to set requirements
    );

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD THEM HERE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Reset odometry to starting pose
    resetOdometry();

    // set the default command for this subsystem
    setDefaultCommand(stopCommand());
    
    // Initialize field visualization
    field2d.setRobotPose(getPose());

    // Initialize dashboard
    SmartDashboard.putData("Differential", this);
    
    // Output initialization progress
    Utils.logInfo("Differential subsystem initialized");
  }
  
  /**
   * Configure the motors with all settings
   */
  private void configureMotors() {
    // Create a shared motor configuration instance
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Set CAN timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeaderMotor.setCANTimeout(250);
    leftFollowerMotor.setCANTimeout(250);
    rightLeaderMotor.setCANTimeout(250);
    rightFollowerMotor.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different battery 
    // voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping breakers.
    motorConfig
      .voltageCompensation(12) // volts
      .smartCurrentLimit(60) // amps
      .idleMode(IdleMode.kBrake);

    // Set the position conversion factor for the encoders    
    motorConfig.encoder
      .positionConversionFactor(DifferentialConstants.kWheelCircumferenceMeters / DifferentialConstants.kEncoderResolution)
      .velocityConversionFactor(DifferentialConstants.kWheelCircumferenceMeters / DifferentialConstants.kEncoderResolution);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped in and persisting 
    // in case of a controller reset due to breaker trip
    motorConfig.follow(leftLeaderMotor);
    leftFollowerMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    motorConfig.follow(rightLeaderMotor);
    rightFollowerMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    // Remove following, then apply config to right leader
    motorConfig.disableFollowerMode();
    rightLeaderMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    motorConfig.inverted(true);
    leftLeaderMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }


  @Override
  public void periodic() {
    // Update the pose estimator with the latest readings
    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(), 
      gyro.getRotation2d(), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition()
    );

    // Add vision measurements to the pose estimator
    addVisionMeasurements();
    
    // Update field visualization
    field2d.setRobotPose(getPose());
  }

  // ==================== Internal State Modifiers ====================

  /**
   * Updates pose estimation using vision measurements from VisionSubsystem
   */
  private void addVisionMeasurements() {
    // Exit early if vision subsystem is not available or is disabled
    if (visionSubsystem == null || !visionSubsystem.isEnabled()) return;

    // Get current time
    double now = Timer.getFPGATimestamp();
    
    // Get all latest vision measurements
    List<VisionMeasurement> measurements = visionSubsystem.getLatestMeasurements();

    // Process each vision measurement
    for (VisionMeasurement measurement : measurements) {
      try {
        Pose2d visionPose = measurement.getPose();
        double timestamp = measurement.getTimestampSeconds();
        double[] stdDevs = measurement.getStandardDeviations();

        // Reject timestamps older than 0.3 seconds
        if ((now - timestamp) > 0.3) {
          continue;
        }

        // Reject timestamps from the future
        if (timestamp > now) {
          continue;
        }

        // Get the robot's current pose
        Pose2d robotPose = poseEstimator.getEstimatedPosition();

        // Reject large translation jumps. 
        // Typical thresholds: 
        //    Auto:   0.5m – 0.75m
        //    Teleop: 1.0m – 1.50m
        if (robotPose.getTranslation().getDistance(visionPose.getTranslation()) > 1.0) {
          continue;
        }

        // Reject rotations > 30 degrees
        if (Math.abs(robotPose.getRotation().minus(visionPose.getRotation()).getDegrees()) > 30.0) {
          continue;
        }

        // Reject poses outside the field boundaries
        if (!visionSubsystem.isPoseOnField(visionPose)) {
          continue;
        }

        // If we make it here => add vision measurement to pose estimator
        poseEstimator.addVisionMeasurement(
          visionPose,
          timestamp,
          VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
        );          
      } catch (Exception e) {
        Utils.logError("Error adding vision measurement: " + e.getMessage());
      }
    }
  }

  /**
   * Reset the robot's odometry
   */
  private void resetOdometry() {
    // Reset the gyro
    gyro.reset();

    // Reset the encoders
    resetEncoders();

    // Reset the pose estimator to the origin
    poseEstimator.resetPosition(
      gyro.getRotation2d(),
      leftEncoder.getPosition(),
      rightEncoder.getPosition(),
      new Pose2d()
    );
  }

  /**
   * Drive the robot using robot-relative chassis speeds (used in path following)
   * @param speeds The desired robot-relative chassis speeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert chassis speeds to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    
    // Get current velocities
    double leftVelocity = leftEncoder.getVelocity();
    double rightVelocity = rightEncoder.getVelocity();
    
    // Calculate feedforward
    double leftFF = feedForward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFF = feedForward.calculate(wheelSpeeds.rightMetersPerSecond);
    
    // Calculate PID correction
    double leftPID = leftPIDController.calculate(leftVelocity, wheelSpeeds.leftMetersPerSecond);
    double rightPID = rightPIDController.calculate(rightVelocity, wheelSpeeds.rightMetersPerSecond);
    
    // Combine and set voltages
    leftLeaderMotor.setVoltage(leftFF + leftPID);
    rightLeaderMotor.setVoltage(rightFF + rightPID);
    drive.feed();
  }

  /**
   * Get the robot's current robot-relative chassis speeds
   * @return The current robot-relative chassis speeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Get the robot's current pose
   * @return The current estimated pose of the robot
   */
  private Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset the robot's pose to a specific location
   * @param pose
   */
  private void resetPose(Pose2d pose) {
    // reset encoders
    resetEncoders();

    // reset pose estimator
    poseEstimator.resetPosition(
      gyro.getRotation2d(),
      leftEncoder.getPosition(),
      rightEncoder.getPosition(),
      pose
    );
  }

  /**
   * Get the current wheel speeds
   * @return The current wheel speeds
   */
  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftEncoder.getVelocity(),
      rightEncoder.getVelocity()
    );
  }

  /**
   * Reset the encoders to zero
   */
  private void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  /**
   * Set the motor brake mode on all the motors
   * @param brake Whether to set brake (true) or coast (false) mode
   */
  private void setMotorBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();

    // set the idle mode
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

    // apply to all motors without resetting or persisting parameters
    rightLeaderMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
    rightFollowerMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
    leftLeaderMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
    leftFollowerMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
  }

  /**
   * Stop all the motors
   */
  private void stop() {
    drive.stopMotor();
  }

  /**
   * Drive the differential in arcade mode (used in teleop)
   * @param xSpeed    The forward/backward speed
   * @param zRotation The rotation rate
   */
  private void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(
      MathUtil.clamp(xSpeed, -1.0, 1.0), 
      MathUtil.clamp(zRotation, -1.0, 1.0)
    );
  }

  // ==================== State Methods ====================

  /**
   * Get the distance to the current alliance hub
   * @return Distance in meters to the alliance hub
   */
  public double getDistanceToAllianceHub() {
    return getPose().getTranslation().getDistance(
      Utils.isRedAlliance() ? FieldConstants.kRedHubCenter : FieldConstants.kBlueHubCenter
    );
  }

  // ==================== Command Factories ====================
  
  /**
   * Reset the robot's odometry
   * @return Command that resets the differential drive odometry
   */
  public Command resetOdometryCommand() {
    return runOnce(this::resetOdometry)
      .withName("ResetOdometryDifferential");
  }

  /**
   * Command to stop the roller
   * @return Command that stops the differential drive
   */
  public Command setMotorBrakeCommand(boolean brake) {
    return runOnce(() -> setMotorBrake(brake))
      .withName("SetMotorBrakeDifferential");
  }
  
  /**
   * Command to stop the roller
   * @return Command that stops the differential drive
   */
  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopDifferential");
  }

  /**
   * Creates a command to aim the robot at the alliance's scoring hub using PID
   * @return Command to aim at the scoring hub
   */  
  public Command aimAtHubCommand() {
    return run(() -> {
      // Get current pose
      Pose2d currentPose = getPose();
      
      // Get hub position (coordinates in meters to the center of the hub)
      // See: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
      Translation2d hubPosition = Utils.isRedAlliance() 
        ? FieldConstants.kRedHubCenter 
        : FieldConstants.kBlueHubCenter;
      
      // Calculate target angle
      Translation2d toHub = hubPosition.minus(currentPose.getTranslation());
      Rotation2d targetAngle = new Rotation2d(toHub.getX(), toHub.getY());
      
      // Calculate rotation speed using PID
      double rotationSpeed = aimPIDController.calculate(
        currentPose.getRotation().getRadians(),
        targetAngle.getRadians()
      );
      
      // Clamp to reasonable speed
      rotationSpeed = MathUtil.clamp(rotationSpeed, -0.6, 0.6);
      
      // Drive
      driveArcade(0.0, rotationSpeed);
    })
    .until(aimPIDController::atSetpoint)
    .finallyDo(this::stop)
    .withName("AimAtHubDifferential");
  }

  /**
   * Creates a command to drive to a specified pose using PathPlanner
   * @param targetPose The Pose2d to drive to
   * @return Command to drive via PathPlanner to the target pose
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    // Deferred command to get latest pose when scheduled
    // Set.of(this) ensures driveSubsystem is required
    return Commands.defer(() -> {
      // Check for null target pose
      if (targetPose == null) {
        return Commands.none();
      }

      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
        DifferentialConstants.kMaxSpeedMetersPerSecond, 
        DifferentialConstants.kMaxAccelMetersPerSecondSq,
        DifferentialConstants.kMaxAngularSpeedRadsPerSecond, 
        DifferentialConstants.kMaxAngularAccelRadsPerSecondSq
      );

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }, Set.of(this));
  }

  /**
   * Command to drive the differential in arcade mode
   * @param xSupplier The forward/backward speed
   * @param rSupplier The rotation rate
   * @return Command that drives the differential in arcade mode
   */
  public Command driveArcadeCommand(DoubleSupplier xSupplier, DoubleSupplier rSupplier) {
    return run(() -> {
      // 1. Apply deadband to the raw joystick inputs.
      //    This ignores noise from the joystick when it's in the neutral position.
      double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), DifferentialConstants.kJoystickDeadband);
      double rSpeed = MathUtil.applyDeadband(rSupplier.getAsDouble(), DifferentialConstants.kJoystickDeadband);

      // 2. Apply slew rate limiters for a smoother acceleration ramp 
      xSpeed = xSpeedLimiter.calculate(xSpeed);
      rSpeed = rSpeedLimiter.calculate(rSpeed);

      // 3. Scale the inputs to the maximum speeds of the robot
      xSpeed *= DifferentialConstants.kTranslationScaling;
      rSpeed *= DifferentialConstants.kRotationScaling;

      // 4. Drive the robot using the processed inputs (-1 to 1 range),
      //    arcadeDrive automatically squares inputs for finer control at low speeds
      driveArcade(xSpeed, rSpeed);
    }).withName("DriveArcadeDifferential");
  }
}
