package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.HandoffCommandGroup;
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
    NamedCommands.registerCommand("rollIn", new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop));
    NamedCommands.registerCommand("rollOut", new StartEndCommand(() -> intakeSubsystem.roll(-.70), intakeSubsystem::stop));
    NamedCommands.registerCommand("armUp", new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() >= 0.75).andThen(armSubsystem::setMotorsBrake));
    NamedCommands.registerCommand("armDown", new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(
            () -> armSubsystem.getEnc() <= .6 && armSubsystem.getEnc() >= .5));
    NamedCommands.registerCommand("shootBump", new SequentialCommandGroup(
            new InstantCommand(() -> pivotSubsystem.setPivotPosition(.48)),
            new InstantCommand(() -> shooterSubsystem.setVelocity(75)),
            new WaitCommand(.75),
            new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
    ));
    NamedCommands.registerCommand("shootLine", new SequentialCommandGroup(
            new InstantCommand(() -> pivotSubsystem.setPivotPosition(.43)),
            new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
            new WaitCommand(.75),
            new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
    ));

    NamedCommands.registerCommand("shootOut", new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)));
    NamedCommands.registerCommand("handoff", new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(1.5));
    }

    public static Command getAutoCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

}