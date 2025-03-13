package commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.sound.sampled.Line;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToAprilTag2D extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final VisionSubsystem vision;

    // PID controllers for rotation, X (left/right), and Y (forward/backward)
    private final PIDController rotationPID;
    private final PIDController xPID;
    private final PIDController yPID;

    // PID Gains (Tune these values)
    private static final double ROTATION_kP = 5;
    private static final double X_kP = 5;
    private static final double Y_kP = 5;

    private static final double ROTATION_TOLERANCE = 1.0; // Degrees
    private static final double X_TOLERANCE = 0.05; // Meters
    private static final double Y_TOLERANCE = 0.1; // Meters

    private final double targetDistance; // Desired distance to AprilTag

    boolean resultExists = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.maxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * .1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToAprilTag2D(CommandSwerveDrivetrain swerve, VisionSubsystem vision, double targetDistanceMeters) {
        this.swerve = swerve;
        this.vision = vision;
        this.targetDistance = targetDistanceMeters;

        this.rotationPID = new PIDController(ROTATION_kP, 0, 0);
        this.xPID = new PIDController(X_kP, 0, 0);
        this.yPID = new PIDController(Y_kP, 0, 0);

        rotationPID.setTolerance(ROTATION_TOLERANCE);
        xPID.setTolerance(X_TOLERANCE);
        yPID.setTolerance(Y_TOLERANCE);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        PhotonPipelineResult result = vision.getPipelineResult();
        resultExists = false;

        if(result != null){
            resultExists = true;
        if (result.hasTargets()) {
            double yaw = result.getBestTarget().getYaw(); // Rotation error
            double xOffset = result.getBestTarget().getBestCameraToTarget().getX(); // Left/right offset
            double zDistance = result.getBestTarget().getBestCameraToTarget().getZ(); // Forward/backward distance

            if(yaw >4){
                yaw = 4;
            } else if(yaw < -4){
                yaw = -4;
            }
            double rotationSpeed = rotationPID.calculate(yaw, 0); // Align rotation
            double xSpeed = xPID.calculate(xOffset, 0); // Strafe left/right
            double ySpeed = yPID.calculate(zDistance, targetDistance); // Move forward/backward


            ChassisSpeeds speeds = new ChassisSpeeds(yaw, 0, 0);

//            swerve.drive(speeds);

/* 
            // Drive using swerve (X = strafe, Y = forward/backward, Rotation)
            // Generate the drive command
        Command driveCommand = swerve.applyRequest(() ->
        robotCentricDrive.withVelocityX(xSpeed * MaxSpeed)
                         .withVelocityY(ySpeed * MaxSpeed)
                         .withRotationalRate(rotationSpeed * MaxAngularRate)
        );

        // Run the command manually
        driveCommand.initialize(); // Start the command if needed
        driveCommand.execute();    // Call execute to apply movement
 */
        } else {


/*      
            Command stopCommand = swerve.applyRequest(() ->
            robotCentricDrive.withVelocityX(0)
                             .withVelocityY(0)
                             .withRotationalRate(0)
            );

            stopCommand.initialize();
            stopCommand.execute();
*/
        }
    }
    SmartDashboard.putBoolean("result", resultExists);
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        Command stopCommand = swerve.applyRequest(() ->
            robotCentricDrive.withVelocityX(0)
                             .withVelocityY(0)
                             .withRotationalRate(0)
            );

            stopCommand.initialize();
            stopCommand.execute();
    }
}
