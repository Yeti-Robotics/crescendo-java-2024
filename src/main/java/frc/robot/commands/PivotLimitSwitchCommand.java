package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotLimitSwitchCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private boolean moveUpTrue;
    private boolean moveDownTrue;

    public PivotLimitSwitchCommand(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize() {
        moveUpTrue = pivotSubsystem.getForwardLimitSwitch();
        moveDownTrue = pivotSubsystem.getReverseLimitSwitch();
    }

    @Override
    public void execute() {
        if (moveUpTrue) {
            pivotSubsystem.moveDown(0.05);
        }
        if (moveDownTrue) {
            pivotSubsystem.moveUp(0.05);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !pivotSubsystem.getForwardLimitSwitch() && !pivotSubsystem.getReverseLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        if (moveUpTrue) {
            pivotSubsystem.moveDown(0.01);
        } else {
            pivotSubsystem.stop();
        }
    }
}
