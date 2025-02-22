package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX elevator;
    TalonFXConfiguration elevatorMotorConfig;
    DutyCycleEncoder elevatorEncoder;
    private double zeroHeight;
    PIDController elevatorController;
    public double targetHeight;
    public double lastPosition;
    public int rotationCount;
    public double currentPosition;

    TalonFX coralClaw;

    double elevatorGearRatio = 1;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ElevatorSubsystem() {

        elevator = new TalonFX(Constants.ElevatorConstants.elevatorMotorID);
        elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorMotorConfig.Slot0.kP = Constants.ElevatorConstants.elevatorkp;
        elevatorMotorConfig.Slot0.kI = Constants.ElevatorConstants.elevatorki;
        elevatorMotorConfig.Slot0.kD = Constants.ElevatorConstants.elevatorkd;
        elevatorMotorConfig.Slot0.kV = 0;
        elevator.getConfigurator().apply(elevatorMotorConfig);

        elevatorEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.elevatorEncoderChannel);
        lastPosition = elevatorEncoder.get();

        elevatorController = new PIDController(Constants.ElevatorConstants.elevatorkp, Constants.ElevatorConstants.elevatorki, Constants.ElevatorConstants.elevatorkd);
        elevatorController.setSetpoint(Constants.ElevatorConstants.holdingHeight);
        elevatorController.setTolerance(2); // Inches of error

        
        coralClaw = new TalonFX(Constants.ElevatorConstants.clawMotorID);
        coralClaw.getConfigurator().apply(elevatorMotorConfig); // They have the same configs, might as well reuse it

        rotationCount = 0;
    }

    public void setHeight(double desiredHeight){
        targetHeight = desiredHeight;
        elevatorController.setSetpoint(desiredHeight);

        double ticks = targetHeight * elevatorGearRatio;

   //     elevator.setControl(positionRequest.withPosition(ticks).withFeedForward(0.1));

    }

    public void periodic() {
        currentPosition = getContinuousPosition();

 //       elevator.setVoltage(elevatorController.calculate(getContinuousPosition(), targetHeight));

//        elevator.setControl(new MotionMagicDutyCycle(targetHeight)); // Get rid of the PIDController?
//        coralClaw.setControl(new MotionMagicDutyCycle(targetClaw));

        updateSmartDashboard();

        SmartDashboard.putNumber("rawElevatorEncoder", elevatorEncoder.get());
        SmartDashboard.putNumber("continuousPosition", getContinuousPosition());
        SmartDashboard.putBoolean("elevatorEncoderConnected",elevatorEncoder.isConnected());
        SmartDashboard.putNumber("elevatorEncoderFrequency", elevatorEncoder.getFrequency());
        
    }

    public void updateSmartDashboard(){
        
    }
    
    public void outputCoral(){
        coralClaw.set(0.2);
    }
    public void retractCoral(){
        coralClaw.set(-0.1);
    }
    public void stopCoral(){
        coralClaw.set(0);
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
