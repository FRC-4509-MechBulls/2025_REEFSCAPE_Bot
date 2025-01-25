package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX elevator;
    AbsoluteEncoder elevatorEncoder;

    public ElevatorSubsystem() {
        elevator = new TalonFX(0);

    }

}
