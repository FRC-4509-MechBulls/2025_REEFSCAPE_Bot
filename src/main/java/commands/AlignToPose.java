package commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToPose extends Command{

    private CommandSwerveDrivetrain driveSubsystem;
    private Pose2d targetPose;
    PIDController xController = new PIDController(0.1, 0, 0);
    PIDController yController = new PIDController(0.1, 0, 0);
    PIDController thetaController = new PIDController(0.1, 0, 0);
    
    public AlignToPose(CommandSwerveDrivetrain driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        addRequirements(driveSubsystem);
    }


    public void execute() {

        Pose2d currentPose = driveSubsystem.getPose();

        Transform2d error = currentPose.minus(targetPose);

        double xSpeed = xController.calculate(error.getX());
        double ySpeed = xController.calculate(error.getY());
        double thetaSpeed = xController.calculate(error.getRotation().getRadians());

        driveSubsystem.drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));

    }
    public boolean isFinished() {

        Pose2d currentPose = driveSubsystem.getPose();
        return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 1;

    }
    public void end(boolean interrupted){
        driveSubsystem.drive(new ChassisSpeeds(0,0,0));
    }
}
