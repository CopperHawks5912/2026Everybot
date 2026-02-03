// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.fuel.FuelConstants;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.DifferentialSubsystem;
import frc.robot.subsystems.drive.SwerveConstants;
import swervelib.SwerveInputStream;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize our controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
   
  // The robot's subsystems are defined here...
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final FeedbackSubsystem feedbackSubsystem = new FeedbackSubsystem(driverXbox);
  private final FuelSubsystem fuelSubsystem = new FuelSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final DifferentialSubsystem driveSubsystem = new DifferentialSubsystem(visionSubsystem);
  // private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/5912_2026"));

  // Auto choosers
  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
  private SendableChooser<Command> delayCommandChooser = new SendableChooser<>();

  // ------------------------------------------------------------------------
  // SWERVE DRIVE CODE
  // ------------------------------------------------------------------------
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
  //     driveSubsystem.getSwerveDrive(),
  //     () -> driverXbox.getLeftY() * -1,
  //     () -> driverXbox.getLeftX() * -1
  //   )
  //   .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
  //   .deadband(SwerveConstants.Deadband)
  //   .scaleTranslation(SwerveConstants.DefaultScaleTranslation)
  //   .allianceRelativeControl(true);  

  // Command driveFieldOrientedAnglularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);

  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands. 
   */
  public RobotContainer() {
    // set our default driving method (field relative - swerve drive)
    // driveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // set our default driving method (arcade - differential drive)
    driveSubsystem.setDefaultCommand(driveSubsystem.driveArcadeCommand(
      () -> -1 * driverXbox.getLeftY(),
      () -> -1 * driverXbox.getRightX()
    ));

    // configure our named commands
    configureNamedCommands();

    // configure our auto routines
    configureAutos();

    // configure our controller bindings
    configureBindings();

    // silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Register named commands to be used in PathPlanner autos.
   * Register named commands before the creation of any PathPlanner Autos or Paths. 
   * It is recommended to do this in RobotContainer, after subsystem 
   * initialization, but before the creation of any other commands.
   */
  public void configureNamedCommands() {
    // NamedCommands.registerCommand("ScoreLevel1Coral", new ScoreLevel1CoralCommand(
    //   driveSubsystem,
    //   elevatorSubsystem,
    //   armSubsystem,
    //   intakeSubsystem
    // ));
  }
  
  /**
   * Register named commands and configure the autonomous command chooser.
   * This will build the auto chooser using the AutoBuilder class, 
   * which pulls in all autos defined in the PathPlanner deploy folder.
   */
  private void configureAutos() {
    // Build the auto chooser and add it to the dashboard
    // This will use Commands.none() as the default option.
    autoCommandChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> DriverStation.isTest()
        ? stream // in test, show all autos
        : stream.filter(auto -> !auto.getName().toLowerCase().startsWith("test")) // in comp, filter out test autos
    );
    
    // Add auto chooser to dashboard
    SmartDashboard.putData("Auto Command", autoCommandChooser);

    // Configure the available auto delay options
    delayCommandChooser.setDefaultOption("No delay", Commands.none());
    delayCommandChooser.addOption("1.0 second", Commands.waitSeconds(1.0));
    delayCommandChooser.addOption("1.5 seconds", Commands.waitSeconds(1.5));
    delayCommandChooser.addOption("2.0 seconds", Commands.waitSeconds(2.0));
    delayCommandChooser.addOption("2.5 seconds", Commands.waitSeconds(2.5));
    delayCommandChooser.addOption("3.0 seconds", Commands.waitSeconds(3.0));
    delayCommandChooser.addOption("3.5 seconds", Commands.waitSeconds(3.5));
    delayCommandChooser.addOption("4.0 seconds", Commands.waitSeconds(4.0));
    delayCommandChooser.addOption("4.5 seconds", Commands.waitSeconds(4.5));
    delayCommandChooser.addOption("5.0 seconds", Commands.waitSeconds(5.0));
    
    // Add delay chooser to dashboard
    SmartDashboard.putData("Auto Delay", delayCommandChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {   
    // manually reset odometry
    driverXbox.start().onTrue((
      Commands.runOnce(driveSubsystem::resetOdometryCommand).ignoringDisable(true)
    ));

    // climb down while holding left trigger
    driverXbox.b().whileTrue(climberSubsystem.downCommand());

    // climb up while holding right trigger
    driverXbox.y().whileTrue(climberSubsystem.upCommand());

    // intake fuel from the ground while holding left trigger
    driverXbox.leftTrigger().whileTrue(fuelSubsystem.intakeCommand());

    // pass fuel from the launcher while holding left bumper
    driverXbox.leftBumper().whileTrue(
      fuelSubsystem.spinUpCommand().withTimeout(FuelConstants.kSpinUpSeconds)
      .andThen(fuelSubsystem.passCommand())      
    );

    // auto-aim at hub holding left trigger
    driverXbox.rightTrigger().onTrue(driveSubsystem.aimAtHubCommand()
      .andThen(feedbackSubsystem.aimedAtHubCommand())
    );

    // launch fuel while holding right bumper
    driverXbox.rightBumper().whileTrue(
      fuelSubsystem.spinUpCommand().withTimeout(FuelConstants.kSpinUpSeconds)
      .andThen(fuelSubsystem.launchCommand(() -> -1))      
    );

    // eject fuel through the intake while holding the A button
    driverXbox.a().whileTrue(fuelSubsystem.ejectCommand());

    // not used
    driverXbox.back().onTrue(Commands.none());
    driverXbox.x().onTrue(Commands.none());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      delayCommandChooser.getSelected(), // run the selected delay command
      autoCommandChooser.getSelected()   // then run the selected auto command
    );
  }

  /**
   * Toggle the motor brake mode
   * @param brake True to brake or false to coast
   */
  public void setMotorBrake(boolean brake) {
    CommandScheduler.getInstance().schedule(driveSubsystem.setMotorBrakeCommand(brake));
  }

  /**
   * Use this to schedule scoring shift feedback based
   * on the game data passed from the DriverStation.
   * @param gameData the game-specific message from the DriverStation
   */
  public void scheduleScoringShiftCommand(char inactiveAlliance) {
    CommandScheduler.getInstance().schedule(feedbackSubsystem.scoringShiftCommand(inactiveAlliance));
  }
}
