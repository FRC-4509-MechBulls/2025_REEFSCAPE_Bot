package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.nio.file.Path;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
    
    public static class ElevatorConstants{

        public static final double distancePerRotation = 2; // Inches

        public static final double minimumHeight = 0.4;
        public static final double holdingHeight = 0.4;
        public static final double level1Height = 2.05;
        public static final double level2Height = 2.27;
        public static final double level3Height = 3.74;
        public static final double level4Height =  5;  // 5.5
        public static final double intakeHeight = .55;

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

        public static final int clawMotorID = 20;

        public static final int beamBreakChannel = 4;
        public static DigitalInput beamBreak = new DigitalInput(beamBreakChannel);

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

        public static final int climbMotorID = 30;

        public static final double climbkp =.1;
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

        public static double maxSpeed = 1;
    }
    public static class RobotConstants{

        public static final double kDriverControllerPort = 0;
        public static final double kOperatorControllerPort = 1;

        public static final double robotWidth = 0;
        public static final double robotLength = 0;

        public static final double visionkP = .1;

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


}
