package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

    DutyCycleEncoder absoluteEncoder;
    PIDController armUpwardPIDController;
    PIDController armDownwardPIDController;

    SparkMax arm;
    SparkMax lowerEF;
    SparkMax upperEF;

    SparkMaxConfig config;

    double desiredAngle;
    double zeroOffset;

    public ShooterSubsystem() {
        
        absoluteEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterArmEncoderChannel);
        
        arm = new SparkMax(Constants.ShooterConstants.armID, MotorType.kBrushless); // Brake Mode must be configured physically, not through code
        lowerEF = new SparkMax(Constants.ShooterConstants.armLowerEFID, MotorType.kBrushless);
        upperEF = new SparkMax(Constants.ShooterConstants.armUpperEFID, MotorType.kBrushless);

        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.secondaryCurrentLimit(50);
        config.smartCurrentLimit(40, 40);
        config.voltageCompensation(12);
        
        arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerEF.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        upperEF.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        zeroOffset = Constants.ShooterConstants.zeroOffset;
        desiredAngle = Constants.ShooterConstants.holdingAngle;

        armUpwardPIDController = new PIDController(Constants.ShooterConstants.armUpwardkp, Constants.ShooterConstants.armUpwardki, Constants.ShooterConstants.armUpwardkd);
        armUpwardPIDController.setIZone(15);
        armUpwardPIDController.setSetpoint(desiredAngle);

        armDownwardPIDController = new PIDController(Constants.ShooterConstants.armDownwardkp, Constants.ShooterConstants.armDownwardki, Constants.ShooterConstants.armDownwardkd);
        armDownwardPIDController.setIZone(10);
        armDownwardPIDController.setSetpoint(desiredAngle);

    }

    public void periodic() {

        if(((absoluteEncoder.get()*360+(zeroOffset*360))%360) > desiredAngle || desiredAngle == Constants.ShooterConstants.holdingAngle) { // If current angle is above setpoint, arm must go downwards
            arm.setVoltage(armDownwardPIDController.calculate((absoluteEncoder.get()*360+(zeroOffset*360))%360));
        }
        else{ // Current angle is below setpoint, arm must go upwards
            arm.setVoltage(armUpwardPIDController.calculate((absoluteEncoder.get()*360+(zeroOffset*360))%360));
        }
        
        SmartDashboard.putNumber("algaeEncoderRotations", absoluteEncoder.get()+zeroOffset);
        SmartDashboard.putNumber("RawAbsoluteArmEncoder", (absoluteEncoder.get()*360)%360);
        SmartDashboard.putNumber("OffsetAbsoluteArmEncoder", (absoluteEncoder.get()*360+(zeroOffset*360))%360);
        SmartDashboard.putNumber("algaeUpwardSetpoint", armUpwardPIDController.getSetpoint());
        SmartDashboard.putNumber("algaeDownwardSetpoint", armDownwardPIDController.getSetpoint());
        
    }

    public void rotateShooter(double setPoint){
        desiredAngle = setPoint;
        armUpwardPIDController.setSetpoint(desiredAngle);
        armDownwardPIDController.setSetpoint(desiredAngle);
    }

    public void setEF(double speed){
        lowerEF.set(-speed);
        upperEF.set(speed);
    }

}
