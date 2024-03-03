package frc.robot.commands;

import static frc.robot.constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorPositions position;
//    private final Timer timer;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        elevatorSubsystem.setPosition(position);
    }

    @Override
    public void execute(){
        elevatorSubsystem.setPosition(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
