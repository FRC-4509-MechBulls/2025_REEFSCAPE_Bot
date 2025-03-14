package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateControllerSub;
import frc.robot.StateControllerSub.ControlState;

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX elevator;
    TalonFXConfiguration elevatorMotorConfig;
    DutyCycleEncoder elevatorEncoder;
    PIDController elevatorUpwardController;
    PIDController elevatorDownwardController;
    DigitalInput upperLimitSwitch;
    DigitalInput lowerLimitSwitch;

    public double targetHeight;
    public double lastPosition;
    public int rotationCount;
    public double currentPosition;

    SparkMax coralClaw;
    SparkMaxConfig coralClawConfig;

    XboxController operatorController = new XboxController(1);

    public ElevatorSubsystem() {

        elevator = new TalonFX(Constants.ElevatorConstants.elevatorMotorID);
        elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorMotorConfig.Slot0.kV = 0;
        elevator.getConfigurator().apply(elevatorMotorConfig);

        elevatorEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.elevatorEncoderChannel);
        elevatorEncoder.setInverted(true);
        lastPosition = elevatorEncoder.get();

        elevatorUpwardController = new PIDController(Constants.ElevatorConstants.elevatorUpwardkp, Constants.ElevatorConstants.elevatorUpwardki, Constants.ElevatorConstants.elevatorUpwardkd);
        elevatorUpwardController.setSetpoint(Constants.ElevatorConstants.holdingHeight);
        elevatorDownwardController = new PIDController(Constants.ElevatorConstants.elevatorDownwardkp, Constants.ElevatorConstants.elevatorDownwardki, Constants.ElevatorConstants.elevatorDownwardkd);
        elevatorDownwardController.setSetpoint(Constants.ElevatorConstants.holdingHeight);
        
        coralClaw = new SparkMax(Constants.ElevatorConstants.clawMotorID, MotorType.kBrushless);
        coralClawConfig = new SparkMaxConfig();
        coralClawConfig.smartCurrentLimit(40);
        coralClawConfig.secondaryCurrentLimit(60);
        coralClawConfig.voltageCompensation(12);
        coralClaw.configure(coralClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        rotationCount = 0;

        upperLimitSwitch = new DigitalInput(Constants.ElevatorConstants.upperLimitSwtichChannel);
        lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.lowerLimitSwitchChannel);


    }

    public void setHeight(double desiredHeight){
        targetHeight = desiredHeight;
        elevatorUpwardController.setSetpoint(desiredHeight);
        elevatorDownwardController.setSetpoint(desiredHeight);
    }

    public void periodic() {
        currentPosition = getContinuousPosition();
        boolean isUpperLimitHit = upperLimitSwitch.get(); 

        if(StateControllerSub.getControlState().equals(ControlState.stateController)){
            if(targetHeight > currentPosition && !isUpperLimitHit){
            elevator.setVoltage(elevatorUpwardController.calculate(getContinuousPosition(), targetHeight));
            } else if(targetHeight < currentPosition) {
            elevator.setVoltage(elevatorDownwardController.calculate(getContinuousPosition(), targetHeight));
            }
        } else{
            if(operatorController.getLeftY() < 0 || !isUpperLimitHit){
               elevator.setVoltage(-operatorController.getLeftY()*12);
            }
        }
        updateSmartDashboard();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("rawElevatorEncoder", elevatorEncoder.get());
        SmartDashboard.putNumber("continuousPosition", getContinuousPosition());
        SmartDashboard.putNumber("elevatorSetpoint", targetHeight);
        SmartDashboard.putBoolean("upperLimitSwitch", upperLimitSwitch.get());
        SmartDashboard.putBoolean("lowerLimitSwitch", lowerLimitSwitch.get());
        SmartDashboard.putBoolean("coralBeamBreak", Constants.ElevatorConstants.beamBreak.get());
        SmartDashboard.putNumber("elevatorUpwardPIDValue", elevatorUpwardController.calculate(getContinuousPosition(), targetHeight));
        SmartDashboard.putNumber("elevatorDownwardPIDValue", elevatorDownwardController.calculate(getContinuousPosition(), targetHeight));
    }
    
    public void outputCoral(double speed){
        coralClaw.set(speed);
    }
    public void retractCoral(){
        coralClaw.set(0.5);
    }
    public void stopCoral(){
        coralClaw.set(0);
    }
    public void resetNumRotations(){
        rotationCount = 0;
    }
    public void adjustNumRotations(double rotations){
        rotationCount += rotations;
    }


    public double getContinuousPosition(){
        double currentPosition = elevatorEncoder.get();

        if(lastPosition > 0.85 && currentPosition < 0.1){
            rotationCount++;
        } else if (lastPosition<0.1 && currentPosition>0.85) {
            rotationCount--;
        }

        lastPosition = currentPosition;

        return rotationCount + currentPosition;
    }
}
