package frc.robot.commands.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.constants.PivotConstants.*;
import static frc.robot.constants.FieldConstants.*;

public class PivotAimCommand extends Command {
    private final PivotSubsystem pivotSubsystem;

    private PivotPositions positions;
    public PivotAimCommand(PivotSubsystem pivotSubsystem, PivotPositions positions1) {
        this.pivotSubsystem = new PivotSubsystem();
        this.positions = positions1;
    }

    @Override
    public void initialize() {    }

    @Override
    public void execute() {
        pivotSubsystem.setPosition(pivotSubsystem.vertAngle);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }
    @Override
    public void end(boolean interrupted) {

    }
}
