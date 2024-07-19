package frc.robot.commands;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private PivotSubsystem pivotSubsystem;

    //    private final Timer timer;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        elevatorSubsystem.setPosition(ElevatorPositions.AMP);

        if (pivotSubsystem.getEncAngle() > .27) {
            pivotSubsystem.moveUp(.1);
        } else if (pivotSubsystem.getEncAngle() <= .27) {
            pivotSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPosition(ElevatorPositions.DOWN);
    }
}
