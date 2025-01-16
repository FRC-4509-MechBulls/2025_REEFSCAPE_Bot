package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class Constants {
    
    public static class EFConstants{

    }
    public static class ElevatorConstants{
        public final double holdingHeight = 0;
        public final double level1Height = 0;
        public final double level2Height = 0;
        public final double level3Height = 0;
        public final double level4Height = 0;
        public final double intakeHeight = 0;

        public final double elevatorIntakeEFSpeed = 0;
        public final double elevatorEjectEFSpeed = 0;
    }
    public static class ShooterConstants{
        public final double intakeAngle = 0;
        public final double processorAngle = 0;

        public final double shooterIntakeEFSpeed = 0;
        public final double shooterShootEFSpeed = 0;
        public final double shooterPlaceEFSpeed = 0;

        public final int upperEFID = 0;
        public final int lowerEFID = 0;
        public final int armID = 0;

    }
    public static class ClimbConstants{
        public final double defaultAngle = 0;
        public final double climbAngle = 0;
    }
    public static class RobotConstants{
        public static final StateControllerSub stateController = new StateControllerSub();
        public static final VisionSubsystem visionSubsystem = new VisionSubsystem();

        public static final double kDriverControllerPort = 0;
        public static final double kOperatorControllerPort = 1;

    }
    public static class AutoConstants{

    }


}
