// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static class SwerveConstants
  {
    public static final double MaxSpeed  = Units.feetToMeters( 10 ); //14.5);
    public static final double WheelLockTime = 10; // seconds
    public static final double RobotMass = (134) * 0.453592; // 32lbs * kg per pound
    public static final Matter Chassis    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);
    public static final double LoopTime  = 0.13; //s, 20ms + 110ms sprk max velocity lag 
    
    public static final double Deadband       = 0.1;
    public static final double LeftYDeadbad   = 0.1;
    public static final double RightXDeadband = 0.1;
    public static final double TurnConstant   = 6;

    public static final double DefaultScaleTranslation = 0.8;
    public static final double L1ScaleTranslation = 0.8;
    public static final double L2ScaleTranslation = 0.8;
    public static final double L3ScaleTranslation = 0.5;
    public static final double L4ScaleTranslation = 0.4;
    public static final double SlowModeScaleTranlastion = 0.3;

    public static final double DefaultScaleRotation = 0.7;
    public static final double L1ScaleRotation = 0.6;
    public static final double L2ScaleRotation = 0.5;
    public static final double L3ScaleRotation = 0.4;
    public static final double L4ScaleRotation = 0.3;
    public static final double SlowModeScaleRotation = 0.3;
    
  }
  
  public static class ReefPoseConstants
  {
    public static enum ScoringAlignment {
      LEFT,
      CENTER,
      RIGHT
    }

    // mapped reef scoring positions
    private static final HashMap<Integer, Pose2d> leftScoringPoses = getLeftScoringPoses();
    private static final HashMap<Integer, Pose2d> centerScoringPoses = getCenterScoringPoses();
    private static final HashMap<Integer, Pose2d> rightScoringPoses = getRightScoringPoses();

    /**
     * Get a left/center/right scoring position based off of a given AprilTag ID
     * @param tagId The fiducial ID of the reef AprilTag to align with
     * @param align Which scoring alignment to move to relative to the center of the AprilTag
     * @return Pose2d
     */
    public static Pose2d getScoringPose(int tagId, ScoringAlignment align) {
      // get the left scoring pose
      if (align == ScoringAlignment.LEFT) {
        return leftScoringPoses.get(tagId);
      }

      // get the right scoring pose
      if (align == ScoringAlignment.RIGHT) {
        return rightScoringPoses.get(tagId);
      }

      // default -> get the center scoring pose
      return centerScoringPoses.get(tagId);
    }
    
    /**
     * Define a hashmap of AprilTag poses that represent our 
     * left side scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getLeftScoringPoses() {      
      HashMap<Integer, Pose2d> map = new HashMap<>();
      map.put( 6, new Pose2d(13.533212698834394, 2.845061489129294, Rotation2d.fromDegrees(300)) );
      map.put( 7, new Pose2d(14.3193412054922, 3.8461651986517804, Rotation2d.fromDegrees(0)) );   
      map.put( 8, new Pose2d(13.844522506657809, 5.027003709522487, Rotation2d.fromDegrees(60)) );
      map.put( 9, new Pose2d(12.584591301165608, 5.206738510870706, Rotation2d.fromDegrees(120)) );
      map.put( 10, new Pose2d(11.798462794507797, 4.20563480134822, Rotation2d.fromDegrees(180)) );
      map.put( 11, new Pose2d(12.273281493342191, 3.0247962904775134, Rotation2d.fromDegrees(240)) );
      map.put( 17, new Pose2d(3.7038294933421905, 3.0247962904775134, Rotation2d.fromDegrees(240)) );
      map.put( 18, new Pose2d(3.2287567945077993, 4.20563480134822, Rotation2d.fromDegrees(180)) );
      map.put( 19, new Pose2d(4.015139301165607, 5.206738510870706, Rotation2d.fromDegrees(120)) );
      map.put( 20, new Pose2d(5.274816506657808, 5.027003709522487, Rotation2d.fromDegrees(60)) );
      map.put( 21, new Pose2d(5.749889205492201, 3.8461651986517804, Rotation2d.fromDegrees(0)) );
      map.put( 22, new Pose2d(4.963506698834392, 2.845061489129294, Rotation2d.fromDegrees(300)) );

      // return the map
      return map;
    }
      
    /**
     * Define a hashmap of AprilTag poses that represent our 
     * right side scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getRightScoringPoses() {
      HashMap<Integer, Pose2d> map = new HashMap<>();
      map.put( 6, new Pose2d(13.844522506657809, 3.0247962904775134, Rotation2d.fromDegrees(300)) );
      map.put( 7, new Pose2d(14.3193412054922, 4.20563480134822, Rotation2d.fromDegrees(0)) );
      map.put( 8, new Pose2d(13.533212698834394, 5.206738510870706, Rotation2d.fromDegrees(60)) );
      map.put( 9, new Pose2d(12.273281493342191, 5.027003709522487, Rotation2d.fromDegrees(120)) );
      map.put( 10, new Pose2d(11.798462794507797, 3.8461651986517804, Rotation2d.fromDegrees(180)) );
      map.put( 11, new Pose2d(12.584591301165606, 2.845061489129294, Rotation2d.fromDegrees(240)) );
      map.put( 17, new Pose2d(4.015139301165607, 2.845061489129294, Rotation2d.fromDegrees(240)) );
      map.put( 18, new Pose2d(3.2287567945077993, 3.8461651986517804, Rotation2d.fromDegrees(180)) );
      map.put( 19, new Pose2d(3.703829493342191, 5.027003709522487, Rotation2d.fromDegrees(120)) );
      map.put( 20, new Pose2d(4.963506698834392, 5.206738510870706, Rotation2d.fromDegrees(60)) );
      map.put( 21, new Pose2d(5.749889205492201, 4.20563480134822, Rotation2d.fromDegrees(0)) );
      map.put( 22, new Pose2d(5.274816506657808, 3.0247962904775134, Rotation2d.fromDegrees(300)) );

      // return the map
      return map;
    }
  
    /**
     * Define a hashmap of AprilTag poses that represent
     * our center scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getCenterScoringPoses() {
      HashMap<Integer, Pose2d> map = new HashMap<>();
      map.put( 6, new Pose2d(13.69193393497587, 2.9296178465885565, Rotation2d.fromDegrees(300)) );
      map.put( 7, new Pose2d(14.32547386995174, 4.0259, Rotation2d.fromDegrees(0)) );
      map.put( 8, new Pose2d(13.69193393497587, 5.122182153411443, Rotation2d.fromDegrees(60)) );
      map.put( 9, new Pose2d(12.42587006502413, 5.122182153411443, Rotation2d.fromDegrees(120)) );
      map.put( 10, new Pose2d(11.792330130048258, 4.0259, Rotation2d.fromDegrees(180)) );
      map.put( 11, new Pose2d(12.42587006502413, 2.9296178465885565, Rotation2d.fromDegrees(240)) );
      map.put( 17, new Pose2d(3.856418065024129, 2.9296178465885565, Rotation2d.fromDegrees(240)) );
      map.put( 18, new Pose2d(3.22262413004826, 4.0259, Rotation2d.fromDegrees(180)) );
      map.put( 19, new Pose2d(3.8564180650241293, 5.122182153411443, Rotation2d.fromDegrees(120)) );
      map.put( 20, new Pose2d(5.12222793497587, 5.122182153411443, Rotation2d.fromDegrees(60)) );
      map.put( 21, new Pose2d(5.75602186995174, 4.0259, Rotation2d.fromDegrees(0)) );
      map.put( 22, new Pose2d(5.12222793497587, 2.9296178465885565, Rotation2d.fromDegrees(300)) );

      // return the map
      return map;
    }
  
  }  

  public static class Controller2Constants
  { 
    public static final int ButtonRed1    =  5; //Controller2
    public static final int ButtonRed2    =  4; //Controller2
    public static final int ButtonRed3    =  3; //Controller2
    public static final int ButtonRed4    =  2; //Controller2
    public static final int ButtonRed5    =  1; //Controller2
    public static final int ButtonBlue2   =  8; //Controller2
    public static final int ButtonBlue3   =  7; //Controller2
    public static final int ButtonBlue4   =  6; //Controller2
  }
  public static class Controller1Constants
  {  
    public static final int ButtonBlue1   =  3; //Controller1
    public static final int ButtonBlack1  =  5; //Controller1
    public static final int ButtonBlack2  =  6; //Controller1
    public static final int ButtonGreen   =  1; //Controller1   
    public static final int ButtonYellow  =  7; //Controller1  
    public static final int ButtonPlayer1 =  8; //Controller1   
    public static final int ButtonPlayer2 =  2; //Controller1 
  } 

  public static final class CANConstants
  {
    public static final int RightElevatorID  = 1;
    public static final int LeftElevatorID   = 2;
    public static final int ArmID            = 3;
    public static final int RollerID         = 4;
  }
  
  public static final class DIOConstants
  {
    // public static final int IntakeArmLimitSwitch = 0;
    public static final int BeamBreakSensorPort = 0;
  }
  
  public static final class AutoConstants
  {
    // public static final double ShooterDelaySeconds = 0.5;
    // public static final double ShooterFastDelaySeconds = 0.20;
    // public static final double IntakeDelaySeconds = 3;
  }

  public static final class ElevatorConstants  //positions in millimeters
  {
    public static double HomePosition = 0.0;
    public static double Lvl1Position = 50.0;
    public static double Lvl2Position = 255;//235.0;//255.0;
    public static double Lvl3Position = 570;//550.0;//605.0;
    public static double Lvl4Position = 1140; //1120.0;//1150.0;
    public static double MaxHeightPosition = 1150.0;
    
    public static double LowerAlgaePosition = 340.0;
    public static double UpperAlgaePosition = 660.0;
    public static double NetAlgaePosition = 1080.0;

    public static double ErrorThreshold = 30.0;

    public static double MMVelocity = 1200;
    public static double MMAcceleration = 3000;
    public static double MMJerk = 0;    
  }

  public static final class ArmConstants
  {
    public static double ErrorThreshold = 0.3; 
    
    public static double CoralMovingPosition = 3.5;
    public static double AlgaeMovingPosition = 18.001465;
    
    public static double HomePosition = 0.0;
    public static double Lvl1Position = 15.0;
    public static double Lvl2Position = 3.0;
    public static double Lvl3Position = 3.0;
    public static double Lvl4Position = 5.7;
    public static double ProcessorAlgaePosition = 19.001465;
    public static double LowerAlgaePosition = 18.001465;
    public static double UpperAlgaePosition = 18.001465;
    public static double NetAlgaePosition = 19.001465;
  }

  public static final class RollerConstants
  {
    //motor speeds from -0.1 to 1.0
    public static double IntakeCoralSpeed = -0.5; 
    public static double OutputCoralSpeed = -0.8; 
    public static double IntakeAlgaeSpeed = 0.8; 
    public static double OutputAlgaeSpeed = -0.5; 
    public static double HoldAlgaeSpeed = 0.1; 
  }

  public static class PWMConstants{
    public static final int LEDStringID = 0;
  }

  // public static class LEDConstants{
  //   public static final int LEDStringLength = 66;
  //   public static final int LEDModeOff = -1;
  //   public static final int LEDModeAllianceBlue = 0;
  //   public static final int LEDModeAllianceRed = 1;  
  //   public static final int LEDModeRainbow = 2;  
  //   public static final int LEDModeCopperHawks = 3;  
  //   public static final int LEDModeNoteEaten = 10;
  //   public static final int LEDModeShooting = 11;
    
  // }

}
