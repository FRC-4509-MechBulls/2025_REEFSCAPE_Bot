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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateControllerSub;

public class VisionSubsystem extends SubsystemBase{

    public VisionSubsystem() {}

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera camera1;
    PhotonCamera camera2;
    PhotonPoseEstimator photonPoseEstimator1;
    PhotonPoseEstimator photonPoseEstimator2;

    StateControllerSub stateController = Constants.RobotConstants.stateController;

    public VisionSubsystem(StateControllerSub stateController) {
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kBaseResourceDir);
        }catch (Exception e) {
            e.printStackTrace();
        }
        camera1 = new PhotonCamera("Camera 1");
        camera2 = new PhotonCamera("Camera2");
        Transform3d robotToCamera1 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        Transform3d robotToCamera2 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        
        photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera1);
        photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera2);
        
        photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        Optional<EstimatedRobotPose> camera1Estimate = photonPoseEstimator1.update(camera1.getAllUnreadResults().get(0)); // is the index right?
        if(camera1Estimate.isPresent()) {
            return camera1Estimate;
        }
        Optional<EstimatedRobotPose> camera2Estimate = photonPoseEstimator2.update(camera2.getAllUnreadResults().get(0));
        return camera2Estimate;
    }

    public void periodic(){
        stateController.updatePoseFromVision();
    }
        
    public PhotonPipelineResult getPipelineResult(){
        if(camera1.getLatestResult().hasTargets()){
            return camera1.getLatestResult();
        }
        else{
            return camera2.getLatestResult();
        }
    }

    public AprilTagFieldLayout getAprilTagFieldLayout(){
        return aprilTagFieldLayout;
    }
    
}
