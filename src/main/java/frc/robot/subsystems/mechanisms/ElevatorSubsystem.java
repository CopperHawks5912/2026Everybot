package frc.robot.subsystems.mechanisms;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX leftElevatorMotor;
  private TalonFX rightElevatorMotor;
  private TalonFXConfiguration eleMotorConfig;
  private double targetPosition;
  private double zeroPoint;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor= new TalonFX(CANConstants.LeftElevatorID);
    rightElevatorMotor = new TalonFX(CANConstants.RightElevatorID);
    targetPosition = 0;

    eleMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode( NeutralModeValue.Brake))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(0.08910703) )
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(70)
        .withSupplyCurrentLowerLimit(40)
        .withSupplyCurrentLowerTime(1)
        .withSupplyCurrentLimitEnable(true))
      .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(16)
        .withPeakReverseVoltage(-16))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(ElevatorConstants.MMAcceleration)
        .withMotionMagicCruiseVelocity(ElevatorConstants.MMVelocity)
        .withMotionMagicJerk(ElevatorConstants.MMJerk));
      // .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
      //   .withForwardSoftLimitEnable(true)
      //   .withForwardSoftLimitThreshold(0)
      //   .withReverseSoftLimitEnable(true)
      //   .withReverseSoftLimitThreshold(ElevatorConstants.MaxHeightPosition));
    eleMotorConfig.Slot0 = new Slot0Configs()
        .withKP(1)
        .withKD(0)
        .withKG(0.0)
        .withKA(0.0)
        .withKV(0.0)
        .withKS(0.0);   
  
    leftElevatorMotor.getConfigurator().apply(eleMotorConfig);
    rightElevatorMotor.getConfigurator().apply(eleMotorConfig);
    
    rightElevatorMotor.setControl(new Follower(CANConstants.LeftElevatorID, false));
    zeroPoint = leftElevatorMotor.getPosition().getValueAsDouble();

    //leftElevatorMotor.setControl(new Follower(CANConstants.RightElevatorID, false));
    //zeroPoint = rightElevatorMotor.getPosition().getValueAsDouble();
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Pos", leftElevatorMotor.getPosition().getValueAsDouble() );
    SmartDashboard.putNumber("Elevator Target",targetPosition );
    isElevatorAtPose();
  }
  /**
   * Creates a zero from input
   * @param theDistance the distance that the distance sensor at the bottom of the elevator reads
   */
  // public void setZero(double theDistance){//Replace with sensor return
  //   double rof0 = theDistance * ElevatorConstants.MillimetersToRotations;
  //   zeroPoint = leftElevatorMotor.getPosition().getValueAsDouble() - rof0;    
  //   }
  /**
   * Sets the elevator to a position relative to the 0 set by createZero. 
   * @param height double that controls how many millimeters from the distance sensor
   */
   public void setElevatorPosition(double position){
    targetPosition = position + zeroPoint;
    MotionMagicVoltage request = new MotionMagicVoltage(targetPosition);
    leftElevatorMotor.setControl(request);
  }


  public boolean isElevatorAtPose() {
    boolean atPose = Math.abs( leftElevatorMotor.getPosition().getValueAsDouble() - targetPosition ) < ElevatorConstants.ErrorThreshold; 
    SmartDashboard.putNumber("Elevator Error",  leftElevatorMotor.getClosedLoopError().getValueAsDouble() );
    SmartDashboard.putBoolean("Elevator At Pose", atPose );
    return atPose;
    
  }
  public double getPIDTarget() {
    return leftElevatorMotor.getClosedLoopReference().getValueAsDouble();
  }
  public void stopElevator(){
    leftElevatorMotor.set(0);
  }

  public void setZeroPoint( ){
    zeroPoint = leftElevatorMotor.getPosition().getValueAsDouble();
    setElevatorPosition(ElevatorConstants.HomePosition);
  }

}