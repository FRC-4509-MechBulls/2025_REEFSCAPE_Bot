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
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
    public enum AlgaeIntakeSource{
        level2, level3
    }

    private State state;
    private State lastState;
    private ItemType itemType;
    private Level level;
    private AlgaeObjective algaeObjective;
    private AlgaeIntakeSource algaeIntakeSource;
    private VisionSubsystem visionSubsystem;
    private CommandSwerveDrivetrain driveSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ClimbSubsystem climbSubsystem;

    public StateControllerSub() {
        state = State.holding;
        State lastState = State.holding;
        itemType = ItemType.coral;
        level = Level.level1;
        algaeObjective = AlgaeObjective.net;
        algaeIntakeSource = AlgaeIntakeSource.level3;
        visionSubsystem = Constants.RobotConstants.visionSubsystem;
        driveSubsystem = Constants.RobotConstants.driveSubsystem;
        elevatorSubsystem = Constants.RobotConstants.elevatorSubsystem;
        shooterSubsystem = Constants.RobotConstants.shooterSubsystem;
        climbSubsystem = Constants.RobotConstants.climbSubsystem;
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
    public AlgaeIntakeSource getAlgaeIntakeSource() {
        return algaeIntakeSource;
    }
    
    public void setRobotState(State desiredState){
        lastState = state;
        if((lastState.equals(State.climbing) && !desiredState.equals(State.holding)) || (lastState.equals(State.placing) && !desiredState.equals(State.holding))){
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
    public void setAlgaeIntakeSource(AlgaeIntakeSource desiredAlgaeIntakeSource){
        algaeIntakeSource = desiredAlgaeIntakeSource;
    }

    public void updatePoseFromVision() {
        Optional<EstimatedRobotPose> result = visionSubsystem.getEstimatedGlobalPose(robotPose);
        if(result.isPresent()){
            // swerveDriveTrain.odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
            Pose2d estimatedPose = result.get().estimatedPose.toPose2d();
            driveSubsystem.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp());
        }
        
    }

    public void extendElevatorClaw(){
        elevatorSubsystem.setClawExtended(true);
    }
    public void retractElevatorClaw() {
        elevatorSubsystem.setClawExtended(false);
    }
    public void setElevatorHeight(){
        if(state.equals(State.pre_placing)){
            switch(level){
                case level1:
                        elevatorSubsystem.setHeight(Constants.ElevatorConstants.level1Height);
                    break;
                case level2:
                        elevatorSubsystem.setHeight(Constants.ElevatorConstants.level2Height);
                    break;
                case level3:
                        elevatorSubsystem.setHeight(Constants.ElevatorConstants.level3Height);
                    break;
                case level4:
                        elevatorSubsystem.setHeight(Constants.ElevatorConstants.level4Height);
                    break;
            }   
        }
        else if(state.equals(State.intaking)){
            elevatorSubsystem.setHeight(Constants.ElevatorConstants.intakeHeight);
        }
        else if(state.equals(State.holding) || state.equals(State.climbing)){
            elevatorSubsystem.setHeight(Constants.ElevatorConstants.holdingHeight);
        }
        
    }

    public void setShooterEF(double speed){
        shooterSubsystem.setEF(speed);
    }
    public void setShooterAngle(double angle){
        shooterSubsystem.rotateShooter(angle);
    }

    public void setClimb(double position){
        climbSubsystem.setAngle(position);
    }

    public void periodic() {
        updateSmartDashboard();

        switch(state){
            case holding: 
                retractElevatorClaw();
                setElevatorHeight();
                setShooterEF(0);
                setShooterAngle(0);
                setClimb(0);
                break;

            case intaking: 
                if(itemType.equals(ItemType.algae)){
                    if(algaeIntakeSource.equals(AlgaeIntakeSource.level3)){
                        setShooterAngle(Constants.ShooterConstants.upperReefAngle);
                    } else{
                        setShooterAngle(Constants.ShooterConstants.lowerReefAngle);
                    }
                    setShooterEF(Constants.ShooterConstants.shooterIntakeEFSpeed);
                }
                else if(itemType.equals(ItemType.coral)){
                    elevatorSubsystem.setHeight(Constants.ElevatorConstants.intakeHeight);
                    retractElevatorClaw(); // ?
                }
                break;

            case pre_placing: 
                if(itemType.equals(ItemType.coral)){
                    setElevatorHeight();
                    // CommandSwerveDrivetrain.align();
                }
                else if(itemType.equals(ItemType.algae)){
                    if(algaeObjective.equals(AlgaeObjective.net)){
                        setShooterAngle(Constants.ShooterConstants.netAngle);
                    }
                    else if(algaeObjective.equals(AlgaeObjective.processor)) {
                        setShooterAngle(Constants.ShooterConstants.processorAngle);
                    }
                }
                break;

            case placing: 
                    if(itemType.equals(ItemType.coral)){
                        extendElevatorClaw();
                    }
                    else if(itemType.equals(ItemType.algae)){
                        if(algaeObjective.equals(AlgaeObjective.net)){
                            setShooterEF(Constants.ShooterConstants.shooterPlaceEFSpeed);
                        }
                        else if(algaeObjective.equals(AlgaeObjective.processor)) {
                            setShooterEF(Constants.ShooterConstants.shooterShootEFSpeed);
                        }
                    }
                break;
            case climbing:
                setElevatorHeight();
                setShooterAngle(0);
                setShooterEF(0);
                setClimb(Constants.ClimbConstants.climbAngle);
                break;
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putString("Robot State", state.toString());
        SmartDashboard.putString("Item Type", itemType.toString());
        SmartDashboard.putString("Level", level.toString());
        SmartDashboard.putString("Algae Objective", algaeObjective.toString());
        SmartDashboard.putString("Algae Intake Source", algaeIntakeSource.toString());
    }
}
