package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase{

    SparkMax climb;
    SparkMaxConfig config;
    SoftLimitConfig softLimitConfig;
    RelativeEncoder encoder;

    PIDController pidController;

    double desiredPosition;

    public ClimbSubsystem() {
        climb = new SparkMax(Constants.ClimbConstants.climbMotorID, MotorType.kBrushless);
        config.smartCurrentLimit(40, 40);
        config.secondaryCurrentLimit(50);
        config.idleMode(IdleMode.kBrake); // ??
        config.softLimit.forwardSoftLimit(30);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = climb.getEncoder();
        encoder.setPosition(0);
    }

    public void setAngle(double position){
        desiredPosition = position;
    }

    public void periodic() {
        if(encoder.getPosition()>desiredPosition){
            climb.set(0);
        } else{
            climb.set((desiredPosition-encoder.getPosition())/100);
        }
    }
    
}
