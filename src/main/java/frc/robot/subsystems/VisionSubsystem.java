package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateControllerSub;

public class VisionSubsystem extends SubsystemBase{

    
    
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera camera1 = new PhotonCamera("BackLeftCamera");
    PhotonCamera camera2 = new PhotonCamera("camera2");
    double alignmentPoint = 0;
//    PhotonPoseEstimator photonPoseEstimator1;
//    PhotonPoseEstimator photonPoseEstimator2;

    public VisionSubsystem() {
        /* Causes the code to crash for some reason
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kBaseResourceDir);
        }catch (Exception e) {
            e.printStackTrace();
        }
        */
    }

    public VisionSubsystem(StateControllerSub stateControllerSub) {
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kBaseResourceDir);
        }catch (Exception e) {
            e.printStackTrace();
        }

        Transform3d robotToCamera1 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        Transform3d robotToCamera2 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        

//        photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera1);
//        photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera2);
        
//        photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//        photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    

    public void setAlignmentPoint(double point){
        alignmentPoint = point;
    }
        
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        /* 
        if(camera1.isConnected()&&camera2.isConnected()){
            Optional<EstimatedRobotPose> camera1Estimate = photonPoseEstimator1.update(camera1.getAllUnreadResults().get(0)); // is the index right?
            if(camera1Estimate.isPresent()) {
                return camera1Estimate;
            }
            Optional<EstimatedRobotPose> camera2Estimate = photonPoseEstimator2.update(camera2.getAllUnreadResults().get(0));
            return camera2Estimate;
        }
        */
        return Optional.empty();
    }

    public void periodic(){
        if(getPipelineResult().getBestTarget() != null){
            SmartDashboard.putNumber("TargetYaw", getYaw());
            SmartDashboard.putNumber("TargetSkew", getSkew());
            SmartDashboard.putNumber("TargetPitch", getPitch());
        }
    }
        
    public PhotonPipelineResult getPipelineResult(){
 
      if(camera1.isConnected() && camera2.isConnected()){
        if(!camera1.getAllUnreadResults().isEmpty()){
            return camera1.getAllUnreadResults().get(camera1.getAllUnreadResults().size()-1);
        } else {
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

    public double getYaw() {
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                var target = result.getBestTarget();
                    targetYaw = target.getYaw();
                    targetVisible = true;    
            }
        }
        targetYaw -= alignmentPoint;
        SmartDashboard.putBoolean("targetVisible", targetVisible);
        if(targetYaw > 4) {
            targetYaw = 4;
        } else if(targetYaw <-4) {
            targetYaw = -4;
        }
        return targetYaw *-1 *Constants.RobotConstants.visionkP;
    }
    public double getSkew() {
        double targetSkew = 0.0;
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                var target = result.getBestTarget();
                    targetSkew = target.getSkew(); 
            }
        }
        return targetSkew *-1 *Constants.RobotConstants.visionkP;
    }
    public double getPitch() {
        double targetPitch = 0.0;
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                var target = result.getBestTarget();
                targetPitch = target.getPitch(); 
            }
        }
        return targetPitch *-1 *Constants.RobotConstants.visionkP;
    }
}
