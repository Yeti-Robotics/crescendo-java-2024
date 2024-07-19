package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotMoveCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    public double position;

    public PivotMoveCommand(PivotSubsystem pivotSubsystem, double position) {
        this.pivotSubsystem = pivotSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        pivotSubsystem.setPivotPosition(position);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return pivotSubsystem.getEncAngle() == position;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pivotSubsystem.stop();
        }
    }
}
