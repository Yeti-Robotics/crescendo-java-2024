package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;


public  class PivotLimitSwitchCommand extends Command
{
    private final PivotSubsystem pivotSubsystem;
    public boolean moveUpTrue;
    public boolean moveDownTrue;

 public PivotLimitSwitchCommand(PivotSubsystem pivotSubsystem)
    {
        this.pivotSubsystem = pivotSubsystem;
        this.pivotSubsystem.forwardLimitSwitch = pivotSubsystem.forwardLimitSwitch;
        this.pivotSubsystem.reverseLimitSwitch = pivotSubsystem.reverseLimitSwitch;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize()
    {
        moveUpTrue = pivotSubsystem.forwardLimitSwitch.get();
        moveDownTrue = pivotSubsystem.reverseLimitSwitch.get();
    }

    @Override
    public void execute()
    {
        if (moveUpTrue){
            pivotSubsystem.moveUp(0.05);
        }
        if(moveDownTrue){
            pivotSubsystem.moveDown(0.05);
        }
    }

    @Override
    public boolean isFinished()
    {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !moveUpTrue && !moveDownTrue;
    }

    @Override
    public void end(boolean interrupted)
    {
        pivotSubsystem.stop();
    }
}
