package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

    DutyCycleEncoder absoluteEncoder;
    PIDController armPIDController;

    SparkMax arm;
    SparkMax lowerEF;
    SparkMax upperEF;

    SparkMaxConfig config;

    double desiredAngle;
    double zeroOffset;

    public ShooterSubsystem() {
        
        arm = new SparkMax(Constants.ShooterConstants.armID, MotorType.kBrushless); // Brake Mode must be configured physically, not through code
        lowerEF = new SparkMax(Constants.ShooterConstants.armLowerEFID, MotorType.kBrushless);
        upperEF = new SparkMax(Constants.ShooterConstants.armUpperEFID, MotorType.kBrushless);

        config.idleMode(IdleMode.kBrake);
        config.secondaryCurrentLimit(50);
        config.smartCurrentLimit(40, 40);
        config.voltageCompensation(12);

        arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerEF.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        upperEF.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterArmEncoderChannel);
        zeroOffset = Constants.ShooterConstants.zeroOffset;

        armPIDController = new PIDController(Constants.ShooterConstants.armkp, Constants.ShooterConstants.armki, Constants.ShooterConstants.armkd);
        
        desiredAngle = 0;
        
    }

    public void periodic() {
        arm.setVoltage(armPIDController.calculate(absoluteEncoder.get()-zeroOffset)); // This actually moves the arm
     
        updateSmartDashboard();
    }

    public void rotateShooter(double setPoint){
        desiredAngle = setPoint;
        armPIDController.setSetpoint(desiredAngle);
    }

    public void setEF(double speed){
        lowerEF.set(speed);
        upperEF.set(-speed);
    }
    
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("RawAbsoluteArmEncoder", absoluteEncoder.get());
        SmartDashboard.putNumber("OffsetAbsoluteArmEncoder", absoluteEncoder.get()-zeroOffset);
        SmartDashboard.putNumber("DesiredArmAngle", desiredAngle);
    }
}
