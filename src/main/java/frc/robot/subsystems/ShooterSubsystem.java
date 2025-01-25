package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

    TalonFX EF;
    TalonFX arm;
    AbsoluteEncoder absoluteEncoder;
    PIDController armPIDController;

    public ShooterSubsystem() {
        EF = new TalonFX(Constants.ShooterConstants.armEFID);
        arm = new TalonFX(Constants.ShooterConstants.armID);
        armPIDController = new PIDController(Constants.ShooterConstants.armkp, Constants.ShooterConstants.armki, Constants.ShooterConstants.armkd);

        absoluteEncoder = new AbsoluteEncoder() {

            @Override
            public double getPosition() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
            }

            @Override
            public double getVelocity() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
            }
            
        };
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", absoluteEncoder.getPosition());
    }

    public void rotateShooter(double setPoint){
        armPIDController.setSetpoint(setPoint);
    }

    public void stopEF(){
        EF.set(0);
    }
    public void intakeEF(){
        EF.set(.5);
    }
    public void outputEF(){
        EF.set(-1);
    }
}
