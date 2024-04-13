package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.HandoffCommandGroup;
import frc.robot.commands.pivot.PivotHomeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoNamedCommands {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private PivotSubsystem pivotSubsystem;
    private ArmSubsystem armSubsystem;


    public AutoNamedCommands(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.armSubsystem = armSubsystem;
        registerCommands();
    }

    public void registerCommands() {
    NamedCommands.registerCommand("rollIn", new StartEndCommand(() -> intakeSubsystem.roll(.9), intakeSubsystem::stop));
    NamedCommands.registerCommand("rollOut", new StartEndCommand(() -> intakeSubsystem.roll(-.70), intakeSubsystem::stop));
    NamedCommands.registerCommand("armUp", new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() >= 0.75).andThen(armSubsystem::setMotorsBrake));
    NamedCommands.registerCommand("armDown", new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(
                    () -> armSubsystem.getEnc() <= .5 && armSubsystem.getEnc() >= .44).alongWith(new PivotHomeCommand(pivotSubsystem)));
    NamedCommands.registerCommand("shootBump", new SequentialCommandGroup(
            new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.515)),
            new InstantCommand(() -> shooterSubsystem.bumpFire()),
            new WaitCommand(.75),
            new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
    ));
    NamedCommands.registerCommand("shootLine", new SequentialCommandGroup(
            new InstantCommand(() -> pivotSubsystem.setPivotPosition(.475)),
            new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
            new StartEndCommand(() -> intakeSubsystem.roll(-.1), intakeSubsystem::stop).withTimeout(0.2),
            new WaitCommand(.45),
            new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
    ));

        NamedCommands.registerCommand("shootLine2", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(.39)),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                new StartEndCommand(() -> intakeSubsystem.roll(-.1), intakeSubsystem::stop).withTimeout(0.2),
                new WaitCommand(.45),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));

        NamedCommands.registerCommand("shootNearSource", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(.43)),
                new InstantCommand(() -> shooterSubsystem.setVelocity(90)),
                new StartEndCommand(() -> intakeSubsystem.roll(-.1), intakeSubsystem::stop).withTimeout(0.2),
                new WaitCommand(.45),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));

        NamedCommands.registerCommand("shuttleMid", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(.535)),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                new StartEndCommand(() -> intakeSubsystem.roll(-.1), intakeSubsystem::stop).withTimeout(0.2),
                new WaitCommand(.45),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));


        NamedCommands.registerCommand("shootBumpCorner", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(.53)),
                new InstantCommand(() -> shooterSubsystem.bumpFire()),
                new WaitCommand(.75),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));

        NamedCommands.registerCommand("shootBumpLast", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(.515)),
                new InstantCommand(() -> shooterSubsystem.bumpFire()),
                new WaitCommand(.25),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));



        NamedCommands.registerCommand("shootFar", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(.45)),
                new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                new WaitCommand(.75),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));



        NamedCommands.registerCommand("shootOut", new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)));
    NamedCommands.registerCommand("handoff", new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(1.5));
    NamedCommands.registerCommand("handoffMidLine", new StartEndCommand(() -> intakeSubsystem.roll(.275), intakeSubsystem::stop)
            .withTimeout(0.5).andThen(
                    new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(1.5)));

        NamedCommands.registerCommand("pivotHandoff", new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.5)));
    }

    public static Command getAutoCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

}