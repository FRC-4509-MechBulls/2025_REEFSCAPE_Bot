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
        config = new SparkMaxConfig();
        config.smartCurrentLimit(40, 40);
        config.secondaryCurrentLimit(50);
        config.idleMode(IdleMode.kBrake); // ??

        config.softLimit.forwardSoftLimitEnabled(false);
        config.softLimit.reverseSoftLimitEnabled(false);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = climb.getEncoder();
        encoder.setPosition(0);

        pidController = new PIDController(Constants.ClimbConstants.climbkp, Constants.ClimbConstants.climbki, Constants.ClimbConstants.climbkd);
        pidController.setSetpoint(0);
    }

    public void setAngle(boolean climbing){
  
        
    }

    public void setSpeed(double speed){
        climb.set(speed/4);
    }

    public void periodic() {
  //      climb.set((desiredPosition-encoder.getPosition())/100);
   //     climb.set(-.1);
    }
    
}
