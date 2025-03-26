package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateControllerSub;

public class VisionSubsystem extends SubsystemBase{

    
    
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera camera1 = new PhotonCamera("FrontCamera");
    PhotonCamera camera2 = new PhotonCamera("camera2");
    PhotonCamera camera3 = new PhotonCamera("Microsoft_LifeCam_HD-3000");
   
    Transform3d robotToCamera1;
    PhotonPoseEstimator photonPoseEstimator1;
    double alignmentPoint;
    PIDController yawController;
//    PhotonPoseEstimator photonPoseEstimator2;

    public VisionSubsystem() {
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
        }catch (Exception e) {
            e.printStackTrace();
        }
                                                                // 12.7147inx, 12.2904iny, 6.704
        robotToCamera1 = new Transform3d(new Translation3d(Units.inchesToMeters(-12.2904),Units.inchesToMeters(12.7147),Units.inchesToMeters(0)), new Rotation3d(0,15,45));
  //      Transform3d robotToCamera2 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        

        photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera1);
  //      photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera2);
        photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
  //      photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        alignmentPoint = 4;
        yawController = new PIDController(Constants.RobotConstants.visionkP, Constants.RobotConstants.visionkI, Constants.RobotConstants.visionkD);
        yawController.setIZone(3);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         
        if(camera1.isConnected()){
            List<PhotonPipelineResult> unreadResults = camera1.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> camera1Estimate = photonPoseEstimator1.update(latestResult);
                if(camera1Estimate.isPresent()) {
                    return camera1Estimate;
                }
            }
            return Optional.empty();
        }
        
        return Optional.empty();
    }
    public Optional<Transform3d> getRobotToTag(){
        if(camera1.isConnected()){
            List<PhotonPipelineResult> unreadResults = camera1.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                PhotonTrackedTarget target = latestResult.getBestTarget();
                if(target != null){
                    return Optional.of(target.getBestCameraToTarget().plus(robotToCamera1));
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }
    public ChassisSpeeds driveToAprilTag(){
        Optional<Transform3d> robotToTag = getRobotToTag();
        if(!robotToTag.isEmpty()){
            
            Transform3d desiredOffset = new Transform3d(
                new Translation3d(-1.0, 0.0, 0.0), // 1 meter "back" from the tag
                new Rotation3d(0, 0, Math.PI) // Turn to face the tag
            );
            Transform3d robotToGoal = robotToTag.get().plus(desiredOffset);
            ChassisSpeeds speeds = new ChassisSpeeds(
                robotToGoal.getX() * 1,  // Forward/backward
                robotToGoal.getY() * 1,  // Strafe
                robotToGoal.getRotation().getZ() // Rotate
            );
            double[] driveSpeeds = {robotToGoal.getX(),  
                robotToGoal.getY(),  
                robotToGoal.getRotation().getZ()};
            SmartDashboard.putNumberArray("aprilTagSpeeds", driveSpeeds);
            double[] transformArray = {robotToGoal.getX(), robotToGoal.getY(), robotToGoal.getRotation().toRotation2d().getDegrees()};
            SmartDashboard.putNumberArray("transformCoordinates", transformArray);
            return speeds;
        }
        return new ChassisSpeeds();
    }

    public void periodic(){
        
    }
        
    public PhotonPipelineResult getPipelineResult(){
 
      if(camera1.isConnected()){
        if(!camera1.getAllUnreadResults().isEmpty()){
            return camera1.getAllUnreadResults().get(camera1.getAllUnreadResults().size()-1);
        } else if(camera2.isConnected()){
            if(!camera2.getAllUnreadResults().isEmpty()){
                return camera2.getAllUnreadResults().get(camera2.getAllUnreadResults().size()-1);
            }
        }
        }
        return new PhotonPipelineResult();
    }

    public AprilTagFieldLayout getAprilTagFieldLayout(){
        return aprilTagFieldLayout;
    }

    public Pose2d getNearestAprilTagPose() {
        PhotonPipelineResult result = this.getPipelineResult();

        Optional<Pose3d> optionalPose3d = aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId());
        if(optionalPose3d.isPresent()) {
            Pose3d pose3d = optionalPose3d.get();

            Translation2d translation2d = new Translation2d(pose3d.getX(), pose3d.getY());

            Rotation2d rotation2d = new Rotation2d(pose3d.getRotation().getZ());

            return new Pose2d(translation2d, rotation2d);
        }

        return null;
    }

    public double getYaw(){
        if(camera1.isConnected()){
            List<PhotonPipelineResult> unreadResults = camera1.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                PhotonTrackedTarget target = latestResult.getBestTarget();
                if(target != null){
                    return target.getYaw();
                }
            }
        }
        return 0;
    }
    public double getClampedYawLeft(){
        double yaw = getYaw();
        double targetYaw = 7.6;

        double yawSpeed = yawController.calculate(yaw, targetYaw);
        
        if(yawSpeed > 5){
            yawSpeed = 5;
        } else if(yawSpeed < -5){
            yawSpeed = -5;
        }
        SmartDashboard.putNumber("yawSpeed", yawSpeed);
        return -yawSpeed;
    }
    public double getClampedYawRight(){
        double yaw = getYaw();
        double targetYaw = -13;

        double yawSpeed = yawController.calculate(yaw, targetYaw);
        
        if(yawSpeed > 5){
            yawSpeed = 5;
        } else if(yawSpeed < -5){
            yawSpeed = -5;
        }
        SmartDashboard.putNumber("yawSpeed", yawSpeed);
        return -yawSpeed;
    }

    
    
   
}
