package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    public enum IntakeSource{
        source, floor
    }
    public enum AlgaeObjective{
        net, processor
    }

    private State state = State.holding;
    private State lastState = State.holding;
    private ItemType itemType = ItemType.coral;
    private Level level = Level.level1;
    private IntakeSource intakeSource = IntakeSource.source;
    private AlgaeObjective algaeObjective = AlgaeObjective.net;

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
    public IntakeSource getIntakeSource(){
        return intakeSource;
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
    public void setIntakeSource(IntakeSource desiredIntakeSource){
        intakeSource = desiredIntakeSource;
    }

    public void periodic() {
        SmartDashboard.putString("Robot State", state.toString());
        SmartDashboard.putString("Item Type", itemType.toString());
        SmartDashboard.putString("Level", level.toString());
        SmartDashboard.putString("Intake Source", intakeSource.toString());

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
