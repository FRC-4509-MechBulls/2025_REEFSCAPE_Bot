package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        
        rotationCount = 0;

   //     upperLimitSwitch = new DigitalInput(Constants.ElevatorConstants.upperLimitSwtichChannel);
   //     lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.lowerLimitSwitchChannel);
    }

    public void setHeight(double desiredHeight){
        targetHeight = desiredHeight;
        elevatorUpwardController.setSetpoint(desiredHeight);
        elevatorDownwardController.setSetpoint(desiredHeight);
    }

    public void periodic() {
        currentPosition = getContinuousPosition();

        if(targetHeight > currentPosition){
//            elevator.setVoltage(elevatorUpwardController.calculate(getContinuousPosition(), targetHeight));
        } else if(targetHeight < currentPosition) {
//            elevator.setVoltage(elevatorDownwardController.calculate(getContinuousPosition(), targetHeight));
        }
        SmartDashboard.putNumber("elevatorUpwardPIDValue", elevatorUpwardController.calculate(getContinuousPosition(), targetHeight));
        SmartDashboard.putNumber("elevatorDownwardPIDValue", elevatorDownwardController.calculate(getContinuousPosition(), targetHeight));

        updateSmartDashboard();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("rawElevatorEncoder", elevatorEncoder.get());
        SmartDashboard.putNumber("continuousPosition", getContinuousPosition());
        SmartDashboard.putNumber("elevatorSetpoint", targetHeight);
 //       SmartDashboard.putBoolean("upperLimitSwitch", upperLimitSwitch.get());
 //       SmartDashboard.putBoolean("lowerLimitSwitch", lowerLimitSwitch.get());
 //       SmartDashboard.putBoolean("coralBeamBreak", Constants.ElevatorConstants.beamBreak.get());
    }
    
    public void outputCoral(){
 //       coralClaw.set(0.2);
    }
    public void retractCoral(){
 //       coralClaw.set(-0.1);
    }
    public void stopCoral(){
 //       coralClaw.set(0);
    }

    public double getContinuousPosition(){
        double currentPosition = elevatorEncoder.get();

        if(lastPosition > 0.9 && currentPosition < 0.1){
            rotationCount++;
        } else if (lastPosition<0.1 && currentPosition>0.9) {
            rotationCount--;
        }

        lastPosition = currentPosition;

        return rotationCount + currentPosition;
    }

}
