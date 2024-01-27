package frc.robot.commands;

import static frc.robot.constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ElevatorPositions position;
    private final Timer timer;

    public ElevatorCommand(ElevatorPositions position) {
        this.position = position;

        timer = new Timer();
        timer.start();
    }

    public void execute(){
        elevatorSubsystem.SetPosition(position);
    }
}
