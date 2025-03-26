package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
    
    public static class ElevatorConstants{

        public static final double distancePerRotation = 2; // Inches

<<<<<<< HEAD
        public static final double minimumHeight = 0.4 + 0.3;
        public static final double holdingHeight = 0.9;
        public static final double level1Height = 2.05 + 0.3;
        public static final double level2Height = 2.27 + 0.47;
        public static final double level3Height = 3.74 + 0.4;
        public static final double level4Height =  5 + 0.3;  // 5.5
        public static final double intakeHeight = 0.9; // .82
=======
        public static final double minimumHeight = 0.4;
        public static final double holdingHeight = 0.4;
        public static final double level1Height = 2.05;
        public static final double level2Height = 2.27;
        public static final double level3Height = 3.74;
        public static final double level4Height =  5;  // 5.5
        public static final double intakeHeight = .55;
>>>>>>> origin/elevatorBranch

        public static final double elevatorIntakeEFSpeed = -0.5;
        public static final double elevatorEjectEFSpeed = 0.5;

        public static final int elevatorMotorID = 25;

        public static final double elevatorEncoderZeroHeight = 0; // Degrees
        public static final int elevatorEncoderChannel = 0;

        public static final double elevatorUpwardkp = 14;
        public static final double elevatorUpwardki = 0;
        public static final double elevatorUpwardkd = 0;

        public static final double elevatorDownwardkp = 14;
        public static final double elevatorDownwardki = 0;
        public static final double elevatorDownwardkd = 0.0;

        public static final int clawMotorID = 30;

        public static final int lowerBeamBreakChannel = 4;
        public static DigitalInput lowerBeamBreak = new DigitalInput(lowerBeamBreakChannel);
        public static final int upperBeamBreakChannel = 8;
        public static DigitalInput upperbeamBreak = new DigitalInput(upperBeamBreakChannel);

        public static final int upperLimitSwtichChannel = 1; 
        public static final int lowerLimitSwitchChannel = 3;

    }
    public static class ShooterConstants{

        // Measured in degrees
        public static final double zeroOffset = 0.444; // Rotations, pointing horizontally left
        public static final double holdingAngle = 110; 
        public static final double holdingNoAlgaeAngle = 60;
        public static final double lowerReefAngle = 145;
        public static final double upperReefAngle = 170;
        public static final double processorAngle = 120;
        public static final double netAngle = 210;

        public static final int shooterArmEncoderChannel = 2;

        public static final double shooterIntakeEFSpeed = -.5;
        public static final double shooterShootEFSpeed = 1;
        public static final double shooterPlaceEFSpeed = 1;

        public static final int armEFID = 21;
        public static final int armID = 17;

        public static final double armUpwardkp = 0.075; // .075
        public static final double armUpwardki = 0.02; // .02
        public static final double armUpwardkd = 0.01; // .01

        public static final double armDownwardkp = 0.03; //.025
        public static final double armDownwardki = 0.0;
        public static final double armDownwardkd = 0.0;

    }
    public static class ClimbConstants{
        // Measured in encoder ticks
        public static final double defaultAngle = 0;
        public static final double climbAngle = 30;

        public static final int climbMotorID = 20;

        public static final double climbkp =.1;
        public static final double climbki = 0;
        public static final double climbkd = 0;

    }
    public static class DriveConstants{

        // Robot Dimensions: 37x37 in, .9398x.9398m
        public static double robotSideLength = .9398;
        
        public static List<Pose2d> aprilTagAlignmentPoses = new ArrayList<Pose2d>();
        static{
          aprilTagAlignmentPoses.add(new Pose2d(3.44, 3.47, new Rotation2d(Math.toRadians(-60)))); // Tag 19 left
          aprilTagAlignmentPoses.add(new Pose2d(3.45, 4.51, new Rotation2d(Math.toRadians(-120))));
          aprilTagAlignmentPoses.add(new Pose2d(4.58, 2.83, new Rotation2d(Math.toRadians(0))));

          /* 
            aprilTagAlignmentPoses.add(new Pose2d(3.949, 5.237, new Rotation2d(Math.toRadians(-60)))); // Tag 19 left
            aprilTagAlignmentPoses.add(new Pose2d(3.660, 5.067, new Rotation2d(Math.toRadians(-60)))); // Tag 19 right

            aprilTagAlignmentPoses.add(new Pose2d(3.161, 4.19, new Rotation2d(Math.toRadians(0)))); 
            aprilTagAlignmentPoses.add(new Pose2d(3.161, 3.841, new Rotation2d(Math.toRadians(0)))); 
            
  //          aprilTagAlignmentPoses.add(new Pose2d(3.689, 2.923, new Rotation2d(Math.toRadians(60)))); 
  //          aprilTagAlignmentPoses.add(new Pose2d(3.969, 2.764, new Rotation2d(Math.toRadians(60)))); 

            aprilTagAlignmentPoses.add(new Pose2d(3.689, 3.923, new Rotation2d(Math.toRadians(-60)))); 
            aprilTagAlignmentPoses.add(new Pose2d(3.969, 3.764, new Rotation2d(Math.toRadians(-60)))); 
            
            aprilTagAlignmentPoses.add(new Pose2d(5.046, 2.803, new Rotation2d(Math.toRadians(120)))); 
            aprilTagAlignmentPoses.add(new Pose2d(3.305, 2.953, new Rotation2d(Math.toRadians(120)))); 
            
            aprilTagAlignmentPoses.add(new Pose2d(5.833, 3.841, new Rotation2d(Math.toRadians(180)))); 
            aprilTagAlignmentPoses.add(new Pose2d(5.833, 4.19, new Rotation2d(Math.toRadians(180)))); 
            
            aprilTagAlignmentPoses.add(new Pose2d(5.275, 5.097, new Rotation2d(Math.toRadians(-120)))); 
            aprilTagAlignmentPoses.add(new Pose2d(4.986, 5.247, new Rotation2d(Math.toRadians(-120))));
            
            aprilTagAlignmentPoses.add(new Pose2d(FieldConstants.fieldLength - 4.986, FieldConstants.fieldWidth-5.247, new Rotation2d(Math.toRadians(-120)))); 
            aprilTagAlignmentPoses.add(new Pose2d(FieldConstants.fieldLength-4.986, FieldConstants.fieldWidth-5.247, new Rotation2d(Math.toRadians(-120)))); 

             
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.949, 5.237, new Rotation2d(Math.toRadians(-60)))));
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.660, 5.067, new Rotation2d(Math.toRadians(-60)))));

            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.161, 4.19, new Rotation2d(Math.toRadians(0)))));
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.161, 3.841, new Rotation2d(Math.toRadians(0)))));

            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.689, 2.923, new Rotation2d(Math.toRadians(60)))));
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.969, 2.764, new Rotation2d(Math.toRadians(60)))));

            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(5.046, 2.803, new Rotation2d(Math.toRadians(120)))));
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(3.305, 2.953, new Rotation2d(Math.toRadians(120)))));

            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(5.833, 3.841, new Rotation2d(Math.toRadians(180)))));
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(5.833, 4.19, new Rotation2d(Math.toRadians(180)))));

            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(5.275, 5.097, new Rotation2d(Math.toRadians(-120)))));
            aprilTagAlignmentPoses.add(AllianceFlipUtil.apply(new Pose2d(4.986, 5.247, new Rotation2d(Math.toRadians(-120)))));
            */
            }


        public static final double xP = 2.5;
        public static final double xI = 0;
        public static final double xD = 0;
        public static final double xTolerance = .05;

        public static final double yP = 2.5;
        public static final double yI = 0;
        public static final double yD = 0;
        public static final double yTolerance = .05;

        public static final double thetaP = 0.1;
        public static final double thetaI = 0;
        public static final double thetaD = 0.05;
        public static final double thetaTolerance = 2;


        public static double maxSpeed = 1;
    }
    public static class RobotConstants{

        public static final double kDriverControllerPort = 0;
        public static final double kOperatorControllerPort = 1;

        public static final double robotWidth = 0;
        public static final double robotLength = 0;

        public static final double visionkP = .07;
        public static final double visionkI = 0.01;
        public static final double visionkD = 0.0;

    }
    public static class AutoConstants{

            public static final ModuleConfig moduleConfig = new ModuleConfig(2, 4.73, 1, DCMotor.getKrakenX60(1), 6.746, 40, 1);
            public static final Translation2d[] translations = {new Translation2d(.302,.302), new Translation2d(.302, -.302), new Translation2d(-.302, .302), new Translation2d(-.302, -.302)};
            
        

            public static final RobotConfig config = new RobotConfig(68, 6.149, moduleConfig, translations);

            public static final RobotConfig robotConfig = createRobotConfig();

            static {
                // Initialization is done in the createRobotConfig method
            }
            
            private static RobotConfig createRobotConfig() {
                RobotConfig config = null;
                try {
                    config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                    e.printStackTrace();
                }
                return config;
            }
    }

    public static final double loopPeriodSecs = 0.02;
    private static RobotType robotType = RobotType.COMPBOT;
    public static final boolean tuningMode = false;

    @SuppressWarnings("resource")
    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
            .set(true);
        robotType = RobotType.COMPBOT;
        }
    return robotType;
    }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

}
