package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Constants {
    
    
    public static class ElevatorConstants{

        // All measurements in inches

        public static final double holdingHeight = 0;
        public static final double level1Height = 0;
        public static final double level2Height = 0;
        public static final double level3Height = 0;
        public static final double level4Height = 0;
        public static final double intakeHeight = 0;

        public static final double elevatorIntakeEFSpeed = 0;
        public static final double elevatorEjectEFSpeed = 0;

        public static final int elevatorMotorID = 0;

        public static final double elevatorEncoderZeroHeight = 0; // Rotations
        public static final int elevatorEncoderChannel = 0;

        public static final double elevatorkp = 3;
        public static final double elevatorki = .1;
        public static final double elevatorkd = 0;

        public static final double distancePerRotation = 2; // Inches

        public static final int clawMotorID = 0;
        public static final int clawEncoderChannel = 0;
        public static final double clawZeroOffset = 0; // Measured in rotations

        public static final double clawkp = 1;
        public static final double clawki = .1;
        public static final double clawkd = 0;

        public static final double clawRetractedAngle = 0;
        public static final double clawExtendedAngle = 0;

    }
    public static class ShooterConstants{

        // Measured in rotations
        public static final double zeroOffset = 0; // Zero should be directly downward
        public static final double holdingAngle = 0; // Directly downward
        public static final double lowerReefAngle = 0;
        public static final double upperReefAngle = 0;
        public static final double processorAngle = 0;
        public static final double netAngle = 0;

        public static final int shooterArmEncoderChannel = 0;

        public static final double shooterIntakeEFSpeed = -.5;
        public static final double shooterShootEFSpeed = 1;
        public static final double shooterPlaceEFSpeed = .5;

        public static final int armLowerEFID = 0;
        public static final int armUpperEFID = 0;
        public static final int armID = 0;

        public static final int armkp = 1;
        public static final int armki = 0;
        public static final int armkd = 0;

    }
    public static class ClimbConstants{
        // Measured in encoder ticks
        public static final double defaultAngle = 0;
        public static final double climbAngle = 30;

        public static final int climbMotorID = 0;

        public static final double climbkp = 0;
        public static final double climbki = 0;
        public static final double climbkd = 0;

    }
    public static class DriveConstants{
        public static final Pose2d tag6Pose = new Pose2d(0,0, new Rotation2d(0));
        public static final Pose2d tag7Pose = new Pose2d(0,0, new Rotation2d(0));
        public static final Pose2d tag8Pose = new Pose2d(0,0, new Rotation2d(0));
        public static final Pose2d tag9Pose = new Pose2d(0,0, new Rotation2d(0));
        public static final Pose2d tag10Pose = new Pose2d(0,0, new Rotation2d(0));
        public static final Pose2d tag11Pose = new Pose2d(0,0, new Rotation2d(0));
    }
    public static class RobotConstants{

        public static final CommandSwerveDrivetrain driveSubsystem = TunerConstants.createDrivetrain();
        public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public static final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

        public static final StateControllerSub stateController = new StateControllerSub();


        public static final double kDriverControllerPort = 0;
        public static final double kOperatorControllerPort = 1;

    }
    public static class AutoConstants{

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


}
