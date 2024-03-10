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

        if(pivotSubsystem.getEncAngle() < .48) {
            down = true;
        } else {
            down = false;
        }

    }

    @Override
    public void execute() {
        if (down) {
            pivotSubsystem.moveUp(.15);
        }

        if (!down) {
            pivotSubsystem.moveDown(.15);
        }
    }


    @Override
    public boolean isFinished() {
        return (pivotSubsystem.getEncAngle() == .50);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stop();
    }
}