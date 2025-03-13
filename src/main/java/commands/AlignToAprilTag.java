package commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTag extends Command{

    private final CommandSwerveDrivetrain driveTrain;
    private final VisionSubsystem vision;
    private final PIDController xController, yController, thetaController;
    
    private final double targetDistance; // Desired distance from the april tag
    private Pose2d targetPose;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.maxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * .1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToAprilTag(CommandSwerveDrivetrain drive, VisionSubsystem vision, double targetDistance) {
        this.driveTrain = drive;
        this.vision = vision;
        this.targetDistance = targetDistance;

        xController = new PIDController(0.8, 0, 0.1);
        yController = new PIDController(0.8, 0, 0.1);
        thetaController = new PIDController(1, 0, 0.1);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, vision);
    }

    public void initialize() {
        Pose2d aprilTagPose = vision.getNearestAprilTagPose();
        if(aprilTagPose != null) {
            targetPose = new Pose2d(
                aprilTagPose.getTranslation().plus(new Translation2d(targetDistance, 0)),
                aprilTagPose.getRotation()
            );
        }
    }

    public void execute() {
        if(targetPose != null) {
            Pose2d currentPose = driveTrain.getState().Pose;

            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotationSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
 //           driveTrain.drive(speeds);
        }
    }

    public boolean isFinished() {
        return  Math.abs(xController.getError()) < .05 &&
                Math.abs(yController.getError()) < .05 &&
                Math.abs(thetaController.getError()) < Math.toRadians(2);
    }

    public void end(boolean interrupted) {
 //       driveTrain.drive(0,0,0);
    }
    
}
