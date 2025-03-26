package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
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
    public enum ControlState{
        stateController, manual
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
  //  private ShooterSubsystem shooterSubsystem;
    private ClimbSubsystem climbSubsystem;
    private static ControlState controlState = ControlState.stateController;
    boolean climbed;
    boolean previousBeamBreakState;
    double  alignmentPoint;

    boolean holdingAlgae;
    SendableChooser<ControlState> controlStateChooser;

    public StateControllerSub(VisionSubsystem vision, ElevatorSubsystem elevator, ClimbSubsystem climb, CommandSwerveDrivetrain drivetrain) {
        state = State.holding;
        State lastState = State.holding;
        itemType = ItemType.coral;
        level = Level.level1;
        algaeObjective = AlgaeObjective.processor;
        algaeIntakeSource = AlgaeIntakeSource.level2;
        visionSubsystem = vision;
        driveSubsystem = drivetrain;
        elevatorSubsystem = elevator;
        climbSubsystem = climb;
        climbed = false;
        controlStateChooser = new SendableChooser<ControlState>();
        controlStateChooser.setDefaultOption("StateControllerControl", ControlState.stateController);
        controlStateChooser.addOption("ManualControl", ControlState.manual);
        SmartDashboard.putData("ControlStateChooser", controlStateChooser);

        holdingAlgae = false;
        previousBeamBreakState = false;
        alignmentPoint = 0;
    }



    public static ControlState getControlState(){
        return controlState;
    }
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
    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
    public double getAlignmentPoint() {
        return alignmentPoint;
    }
    
    public void setRobotState(State desiredState){
        lastState = state;
        if(desiredState.equals(State.holding) && lastState.equals(State.placing) && level.equals(Level.level4)){
            state = State.pre_placing;
            outputCoral(0);
            level = Level.level1;
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
        Optional<EstimatedRobotPose> result = visionSubsystem.getEstimatedGlobalPose(driveSubsystem.getState().Pose);
        if(result.isPresent()){
            Pose2d estimatedPose = result.get().estimatedPose.toPose2d();
            driveSubsystem.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp(), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)));
        }
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
  //      shooterSubsystem.setEF(speed);
    }
    public void setShooterAngle(double angle){
 //       shooterSubsystem.rotateShooter(angle);
    }
    public void outputCoral(double speed){
        elevatorSubsystem.outputCoral(speed);
    }
    public void outputCoral(){
        if(state.equals(State.intaking)){
            elevatorSubsystem.outputCoral(-.5);
        } else{
            elevatorSubsystem.outputCoral(-1);
        }
    }
    public void stopCoral(){
        elevatorSubsystem.stopCoral();
    }
    public void reverseCoral(){
        elevatorSubsystem.retractCoral();
    }

    public void periodic() {
        updateSmartDashboard();
 //       updatePoseFromVision();

        switch(state){
            case holding: 
                if(!Constants.ElevatorConstants.lowerBeamBreak.get()){
                    outputCoral(.05);
                } else {
                    stopCoral();
                }   
               
                setElevatorHeight();
                setShooterEF(0);
                if(holdingAlgae){
 //                   setShooterAngle(Constants.ShooterConstants.holdingAngle);
                } else{
 //                   setShooterAngle(Constants.ShooterConstants.holdingNoAlgaeAngle);
                }
                
                // toggleClimb(0);
                break;

            case intaking: 
                if(itemType.equals(ItemType.algae)){
                    if(algaeIntakeSource.equals(AlgaeIntakeSource.level3)){
 //                       setShooterAngle(Constants.ShooterConstants.upperReefAngle);
                    } else{
 //                       setShooterAngle(Constants.ShooterConstants.lowerReefAngle);
                    }
 //                   setShooterEF(Constants.ShooterConstants.shooterIntakeEFSpeed);
                }
                else if(itemType.equals(ItemType.coral)){
                    elevatorSubsystem.setHeight(Constants.ElevatorConstants.intakeHeight);
                    outputCoral(-.3);

                    if(!Constants.ElevatorConstants.lowerBeamBreak.get() && Constants.ElevatorConstants.upperbeamBreak.get()){
                         state = State.holding;
                         stopCoral();
                    }            
                    /*  After second beamBreak

                    if(!Constants.ElevatorConstants.lowerBeamBreak.get() && Constants.ElevatorConstants.upperBeamBreak.get()){
                        state = State.holding;
                    }

                    */
                }
                break;

            case pre_placing: 
                if(itemType.equals(ItemType.coral)){
                    setElevatorHeight();
                }
                else if(itemType.equals(ItemType.algae)){
                    if(algaeObjective.equals(AlgaeObjective.net)){
 //                       setShooterAngle(Constants.ShooterConstants.netAngle);
                    }
                    else if(algaeObjective.equals(AlgaeObjective.processor)) {
  //                      setShooterAngle(Constants.ShooterConstants.processorAngle);
                    }
                }
                break;

            case placing: 
                    if(itemType.equals(ItemType.coral)){
                            if(level.equals(Level.level4)){
                                outputCoral(-3);
                            } else if (level.equals(Level.level1)){
                                outputCoral(-.3);
                            } else{
                                outputCoral(-.6);
                            }
                    }
                    else if(itemType.equals(ItemType.algae)){
                        if(algaeObjective.equals(AlgaeObjective.net)){
 //                           setShooterEF(Constants.ShooterConstants.shooterShootEFSpeed);
                        }
                        else if(algaeObjective.equals(AlgaeObjective.processor)) {
 //                           setShooterEF(Constants.ShooterConstants.shooterPlaceEFSpeed);
                        }
                    }
                break;
            case climbing:
                setElevatorHeight();
 //               setShooterAngle(Constants.ShooterConstants.holdingAngle);
                setShooterEF(0);
                break;
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putString("Robot State", state.toString());
        SmartDashboard.putString("Item Type", itemType.toString());
        SmartDashboard.putString("Level", level.toString());
        SmartDashboard.putString("Algae Objective", algaeObjective.toString());
        SmartDashboard.putString("Algae Intake Source", algaeIntakeSource.toString());
        SmartDashboard.putBoolean("holdingAlgae", holdingAlgae);
 //       SmartDashboard.putNumber("pipelineResult", visionSubsystem.getPipelineResult().getBestTarget().fiducialId);
        SmartDashboard.putNumber("AlignmentPoint", alignmentPoint);
        SmartDashboard.putNumber("flAngle", driveSubsystem.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("frAngle", driveSubsystem.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("blAngle", driveSubsystem.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("brAngle", driveSubsystem.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble());
}

    public void toggleControlMode(){
        if(controlState.equals(ControlState.manual)){
            controlState = ControlState.stateController;
        } else{
            controlState = ControlState.manual;
        }
    }
    public void toggleHoldingAlgae() {
        holdingAlgae = !holdingAlgae;
    }

    public void rightBumper() {
        if(state.equals(State.holding)){
            if(itemType.equals(ItemType.coral)){
                setItemType(ItemType.algae);
            } else {
                setItemType(ItemType.coral);
            }
        }
    }

    public void leftBumper() {
        if(algaeIntakeSource.equals(AlgaeIntakeSource.level2)){
            algaeIntakeSource = AlgaeIntakeSource.level3;
        } else{
            algaeIntakeSource = AlgaeIntakeSource.level2;
        }
    }

    public void leftTrigger(){
        if(!state.equals(State.placing)){
            if(algaeObjective.equals(AlgaeObjective.net)){
                    algaeObjective = AlgaeObjective.processor;
                } else {
                    algaeObjective = AlgaeObjective.net;
                }
        }   
    }

    public void toggleClimb(double speed){
        climbSubsystem.setSpeed(speed);
    }
    public void setShooterIntakeSource(AlgaeIntakeSource algaeIntakeSource){
        this.algaeIntakeSource = algaeIntakeSource;
    }
    public ElevatorSubsystem getElevator(){
        return elevatorSubsystem;
    }

    private ChassisSpeeds lastKnownSpeeds = new ChassisSpeeds();
    public ChassisSpeeds driveToAprilTag(){
        Optional<Transform3d> robotToTag = visionSubsystem.getRobotToTag();
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
            lastKnownSpeeds = speeds;
            double[] driveSpeeds = {robotToGoal.getX(),  
                robotToGoal.getY(),  
                robotToGoal.getRotation().getZ()};
            SmartDashboard.putNumberArray("aprilTagSpeeds", driveSpeeds);
            double[] transformArray = {robotToGoal.getX(), robotToGoal.getY(), robotToGoal.getRotation().toRotation2d().getDegrees()};
            SmartDashboard.putNumberArray("transformCoordinates", transformArray);
            return speeds;
        }
        return lastKnownSpeeds;
    }
    
}
