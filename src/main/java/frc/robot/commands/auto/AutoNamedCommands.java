package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.RobotCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.PivotSubsystem;

public class AutoNamedCommands {
    private final IntakeSubsystem intakeSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final ArmSubsystem armSubsystem;
    private final FeederSubsystem feederSubsystem;

    private final RobotCommands robotCommands;


    public AutoNamedCommands(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem, FeederSubsystem feederSubsystem, RobotCommands robotCommands) {
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.armSubsystem = armSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.robotCommands = robotCommands;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand("rollIn", intakeSubsystem.rollIn(0.9));
        NamedCommands.registerCommand("rollOut", intakeSubsystem.rollOut(-0.7));
        NamedCommands.registerCommand("armUp", armSubsystem.moveUpAndStop(.5).until(() -> armSubsystem.getEnc() >= 0.75));
        NamedCommands.registerCommand("armDown", armSubsystem.moveDownAndStop(.5).until(
                () -> armSubsystem.getEnc() <= .5 && armSubsystem.getEnc() >= .44).alongWith(pivotSubsystem.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF)));

        NamedCommands.registerCommand("shootBump", robotCommands.setTargetAndShoot(0.53, 100));
        NamedCommands.registerCommand("shootLine", robotCommands.setTargetAndShoot(0.47, 100));
        NamedCommands.registerCommand("shootLine2", robotCommands.setTargetAndShoot(0.39, 100));
        NamedCommands.registerCommand("shootNearSource", robotCommands.setTargetAndShoot(0.48, 90));
        NamedCommands.registerCommand("shuttleMid", robotCommands.setTargetAndShoot(0.535, 100));
        NamedCommands.registerCommand("shootBumpCorner", robotCommands.setTargetAndShoot(0.55, 100));
        NamedCommands.registerCommand("shootBumpLast", robotCommands.setTargetAndShoot(0.55, 100));
        NamedCommands.registerCommand("shootBumpLast", robotCommands.setTargetAndShoot(0.45, 100));

        NamedCommands.registerCommand("shootOut", feederSubsystem.feed(-1).alongWith(intakeSubsystem.rollOut(-1).withTimeout(1)));
        NamedCommands.registerCommand("handoff", robotCommands.handoff().withTimeout(1.5));
        NamedCommands.registerCommand("handoffMidLine", intakeSubsystem.rollIn(.275).withTimeout(0.5).andThen(robotCommands.handoff().withTimeout(1.5)));

        NamedCommands.registerCommand("pivotHandoff", pivotSubsystem.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF));
    }
}