package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbySubsystem extends SubsystemBase {
    private SparkMax climbMotor;
    SparkMaxConfig climbyConfig = new SparkMaxConfig();

    public ClimbySubsystem() {
        climbMotor = new SparkMax(Constants.ClimbConstants.climbMotorID, MotorType.kBrushless);
        
        climbyConfig.smartCurrentLimit(40);
        climbyConfig.idleMode(IdleMode.kBrake);
        climbyConfig.openLoopRampRate(.5);
        climbyConfig.inverted(false);


        climbMotor.configure(climbyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       }  

    public void setVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    } 

    public void stopMotor(){
        climbMotor.setVoltage(0);
    }
} 