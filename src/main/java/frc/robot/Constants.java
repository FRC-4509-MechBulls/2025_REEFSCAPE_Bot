package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class Constants {
    
    public static class EFConstants{

    }
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

        public static final double elevatorEncoderZeroHeight = 0;
        public static final int elevatorEncoderChannel = 0;

        public static final double elevatorkp = 3;
        public static final double elevatorki = .1;
        public static final double elevatorkd = 0;

        public static final double distancePerRotation = 2; // inches
    }
    public static class ShooterConstants{
        public static final double intakeAngle = 0;
        public static final double processorAngle = 0;

        public static final double shooterIntakeEFSpeed = 0;
        public static final double shooterShootEFSpeed = 0;
        public static final double shooterPlaceEFSpeed = 0;

        public static final int armEFID = 99;
        public static final int armID = 99;

        public static final int armkp = 0;
        public static final int armki = 0;
        public static final int armkd = 0;

    }
    public static class ClimbConstants{
        public static final double defaultAngle = 0;
        public static final double climbAngle = 0;

        public static final double climbMotorID = 99;
    }
    public static class RobotConstants{

        public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public static final CommandSwerveDrivetrain driveSubsystem = TunerConstants.createDrivetrain();
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
