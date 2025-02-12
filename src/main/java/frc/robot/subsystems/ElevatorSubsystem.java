package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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

    TalonFX coralClaw;
    DutyCycleEncoder clawEncoder;
    private double clawZeroOffset;
    PIDController clawController;
    private boolean clawExtended;
    private double targetClaw;

    public ElevatorSubsystem() {

        elevator = new TalonFX(Constants.ElevatorConstants.elevatorMotorID);
        elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevator.getConfigurator().apply(elevatorMotorConfig);

        elevatorEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.elevatorEncoderChannel);
        zeroHeight = Constants.ElevatorConstants.elevatorEncoderZeroHeight;
        targetHeight = zeroHeight;

        elevatorController = new PIDController(Constants.ElevatorConstants.elevatorkp, Constants.ElevatorConstants.elevatorki, Constants.ElevatorConstants.elevatorkd);
        elevatorController.setSetpoint(Constants.ElevatorConstants.holdingHeight);
        elevatorController.setTolerance(2); // Inches of error
        
        coralClaw = new TalonFX(Constants.ElevatorConstants.clawMotorID);
        coralClaw.getConfigurator().apply(elevatorMotorConfig); // They have the same configs, might as well reuse it

        clawEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.clawEncoderChannel);
        clawZeroOffset = Constants.ElevatorConstants.clawZeroOffset;

        clawController = new PIDController(Constants.ElevatorConstants.clawkp, Constants.ElevatorConstants.clawki, Constants.ElevatorConstants.clawkd);
        clawController.setSetpoint(Constants.ElevatorConstants.clawRetractedAngle);
        clawController.setTolerance(5);

        clawExtended = false;
        targetClaw = Constants.ElevatorConstants.clawRetractedAngle;

    }

    public void setHeight(double desiredHeight){
        targetHeight = desiredHeight;
        elevatorController.setSetpoint(desiredHeight);
    }
    
    public void setClawExtended(boolean extended){
        clawExtended = extended;
        if(extended){
            clawController.setSetpoint(Constants.ElevatorConstants.clawExtendedAngle);
            targetClaw = Constants.ElevatorConstants.clawExtendedAngle;
        }
        else {
            clawController.setSetpoint(Constants.ElevatorConstants.clawRetractedAngle);
            targetClaw = Constants.ElevatorConstants.clawRetractedAngle;
        }
    }

    public void periodic() {
//        elevator.setControl(new MotionMagicDutyCycle(targetHeight)); // Get rid of the PIDController?
//        coralClaw.setControl(new MotionMagicDutyCycle(targetClaw));

  //      elevatorController.calculate((elevatorEncoder.get()-zeroHeight)*Constants.ElevatorConstants.distancePerRotation);
   
   //     clawController.calculate((clawEncoder.get()-clawZeroOffset)*360);

  //      coralClaw.set(clawController.calculate((clawEncoder.get()-clawZeroOffset)*360)); 

        updateSmartDashboard();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putBoolean("Claw Extended", clawExtended);
        SmartDashboard.putNumber("Claw Error", clawController.getError());
        SmartDashboard.putNumber("Elevator Height", (elevatorEncoder.get()*Constants.ElevatorConstants.distancePerRotation-zeroHeight)*Constants.ElevatorConstants.distancePerRotation);
        SmartDashboard.putNumber("Elevator Error", elevatorController.getError());
    }
    

}
