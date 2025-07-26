package commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTag2D extends Command{

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.maxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond) * Constants.DriveConstants.maxSpeed; // 3/4 of a rotation per second max angular velocity
    private CommandSwerveDrivetrain driveSubsystem;
    public VisionSubsystem visionSubsystem;
    private double yawToAlign;
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * .1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToAprilTag2D(CommandSwerveDrivetrain swerve, double yaw, VisionSubsystem vision){
        driveSubsystem = swerve;
        yawToAlign = yaw;
        visionSubsystem = vision;
        addRequirements(swerve);
    }

    public void execute(){
        double yaw = visionSubsystem.getYaw();
        double yawSpeed = yaw - yawToAlign;
        if(yawSpeed > 5){
            yawSpeed = 5;
        } else if(yawSpeed < -5){
            yawSpeed = -5;
        }

        driveSubsystem.setControl(robotCentricDrive.withVelocityX(0)
                                    .withVelocityY(-yawSpeed)
                                    .withRotationalRate(0));
    }

    public boolean isFinished(){
        return true;
    }

    public void end(boolean interrupted){
        driveSubsystem.setControl(robotCentricDrive.withVelocityX(0)
                                    .withVelocityY(0)
                                    .withRotationalRate(0));
    }
    
}
