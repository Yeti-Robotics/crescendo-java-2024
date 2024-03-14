package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotHomeCommand extends Command {

    private final PivotSubsystem pivotSubsystem;
    private boolean down;

    public PivotHomeCommand(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.setPivotPosition(0.5);
    }

    @Override
    public void execute() {
    }


    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}