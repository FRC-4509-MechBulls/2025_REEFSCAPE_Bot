package commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class IntakeCoral extends Command{

    ElevatorSubsystem elevatorSubsystem;
    private DigitalInput beamSensor;

    public IntakeCoral(ElevatorSubsystem elevator, DigitalInput beamBreak){
        elevatorSubsystem = elevator;
        beamSensor = beamBreak;
    }

    public void execute(){
        elevatorSubsystem.outputCoral(-.2);
    }

    public boolean isFinished(){
        return !beamSensor.get();
    }

    public void end(boolean interrupted){
        elevatorSubsystem.stopCoral();
    }
    
}
