package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HandoffCommandGroup;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoNamedCommands {
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final ArmSubsystem armSubsystem;


    public AutoNamedCommands(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.armSubsystem = armSubsystem;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand("rollIn", intakeSubsystem.rollIn(0.9));
        NamedCommands.registerCommand("rollOut", intakeSubsystem.rollOut(-0.7));
        NamedCommands.registerCommand("armUp", new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() >= 0.75).andThen(armSubsystem::stop));
        NamedCommands.registerCommand("armDown", new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(
                () -> armSubsystem.getEnc() <= .5 && armSubsystem.getEnc() >= .44).alongWith(pivotSubsystem.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF)));
        NamedCommands.registerCommand("shootBump", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(0.53),
                shooterSubsystem.shooterBumpFire(),
                new WaitCommand(.75),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));

        NamedCommands.registerCommand("shootLine", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.47),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.25),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1.7))
        ));

        NamedCommands.registerCommand("shootLine2", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.39),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.2),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));

        NamedCommands.registerCommand("shootNearSource", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.48),
                new InstantCommand(() -> shooterSubsystem.setVelocity(90)),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.2),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));

        NamedCommands.registerCommand("shuttleMid", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.535),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                intakeSubsystem.rollOut(-0.1).withTimeout(0.2),
                new WaitCommand(.45),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootBumpCorner", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.55),
                shooterSubsystem.shooterBumpFire(),
                new WaitCommand(.75),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));

        NamedCommands.registerCommand("shootBumpLast", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.55),
                shooterSubsystem.shooterBumpFire(),
                new WaitCommand(.25),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootFar", new SequentialCommandGroup(
                pivotSubsystem.adjustPivotPositionTo(.45),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                new WaitCommand(.75),
                shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootOut", shooterSubsystem.spinFeederMaxAndStop().alongWith(intakeSubsystem.rollOut(-1).withTimeout(1)));
        NamedCommands.registerCommand("handoff", new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(1.5));
        NamedCommands.registerCommand("handoffMidLine", intakeSubsystem.rollIn(.275)
                .withTimeout(0.5).andThen(
                        new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(1.5)));

        NamedCommands.registerCommand("pivotHandoff", pivotSubsystem.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF));
    }
}