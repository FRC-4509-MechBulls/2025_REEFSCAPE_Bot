package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class StateControllerSub extends SubsystemBase{

    public enum ItemType{
        coral, algae
    }
    public enum Level{
        level1, level2, level3, level4
    }
    public enum State{
        holding, intaking, pre_placing, placing, climbing
    }
    public enum AlgaeObjective{
        net, processor
    }

    private State state;
    private State lastState;
    private ItemType itemType;
    private Level level;
    private AlgaeObjective algaeObjective;
    private VisionSubsystem visionSubsystem;
    private CommandSwerveDrivetrain driveSubsystem;

    public StateControllerSub() {
        state = State.holding;
        State lastState = State.holding;
        itemType = ItemType.coral;
        level = Level.level1;
        algaeObjective = AlgaeObjective.net;
        visionSubsystem = Constants.RobotConstants.visionSubsystem;
        driveSubsystem = Constants.RobotConstants.driveSubsystem;
    }

    private Pose2d robotPose = new Pose2d();

    public State getRobotState(){
        return state;
    }
    public ItemType getItemType(){
        return itemType;
    }
    public Level getLevel(){
        return level;
    }
    public AlgaeObjective getAlgaeObjective(){
        return algaeObjective;
    }
    
    public void setRobotState(State desiredState){
        lastState = state;
        if(lastState.equals(State.climbing) && !desiredState.equals(State.holding)){
            return;
        }
        state = desiredState;
    }
    public void setItemType(ItemType desiredItemType){
        itemType = desiredItemType;
    }
    public void setLevel(Level desiredLevel){
        level = desiredLevel;
    }
    public void setAlgaeObjective(AlgaeObjective desiredAlgaeObjective) {
        algaeObjective = desiredAlgaeObjective;
    }

    public void updatePoseFromVision() {
//        Optional<EstimatedRobotPose> result = visionSubsystem.getEstimatedGlobalPose(robotPose);
//        if(result.isPresent()){
//            // swerveDriveTrain.odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
//            Pose2d estimatedPose = result.get().estimatedPose.toPose2d();
//            driveSubsystem.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp());
//        }
        
    }
    public void periodic() {
        SmartDashboard.putString("Robot State", state.toString());
        SmartDashboard.putString("Item Type", itemType.toString());
        SmartDashboard.putString("Level", level.toString());
        SmartDashboard.putString("Algae Objective", algaeObjective.toString());

        switch(state){
            case holding: 
                // elevatorSubsystem.set(Constants.holdingHeight);
                // shooterSubsystem.setEFSpeed(0);
                // climbSubsystem.setAngle(Constants.defaultAngle);
                break;

            case intaking: 
                if(itemType.equals(ItemType.algae)){
                    // shooterSubsystem.setAngle(Constants.IntakeAngle);
                    // shooterSubsystem.setEFSpeed(Constants.shooterIntakeEFSpeed);
                }
                else if(itemType.equals(ItemType.coral)){
                    // elevatorSubsystem.set(Constants.intakeHeight);
                    // elevatorSubsystem.setEFSpeed(Constants.elevatorIntakeEFSpeed);
                }
                break;

            case pre_placing: 
                if(itemType.equals(ItemType.coral)){
                    switch(level){
                        case level1:
                                // elevatorSubsystem.set(Constants.level1Height);
                            break;
                        case level2:
                                // elevatorSubsystem.set(Constants.level2Height);
                            break;
                        case level3:
                                // elevatorSubsystem.set(Constants.level3Height);
                            break;
                        case level4:
                                // elevatorSubsystem.set(Constants.level4Height);
                            break;
                    }
                    // CommandSwerveDrivetrain.align();
                }
                else if(itemType.equals(ItemType.algae)){
                    if(algaeObjective.equals(AlgaeObjective.net)){
                        //shooterSubsystem.updateAngle();
                    }
                    else if(algaeObjective.equals(AlgaeObjective.processor)) {
                        //shooterSubsystem.setAngle(Constants.processorAngle);
                    }
                    
                }
                break;

            case placing: 
                    if(itemType.equals(ItemType.coral)){
                        //elevatorSubsystem.setEFSpeed(Constants.elevatorEjectEFSpeed);
                    }
                    else if(itemType.equals(ItemType.algae)){
                        if(algaeObjective.equals(AlgaeObjective.net)){
                            //shooterSubsystem.setIntakeEFSpeed(Constants.shooterIntakeEFSpeed);
                        }
                        else if(algaeObjective.equals(AlgaeObjective.processor)) {
                            //shooterSubsystem.setShooterEFSpeed(Constants.shooterIntakeEFSpeed);
                        }
                    }
                break;
            case climbing:
                // elevatorSubsystem.set(Constants.holdingHeight);
                // shooterSubsystem.setEFSpeed(0);
                // climbSubsystem.setAngle(Constants.climbAngle);
                break;
        }
    }
}
