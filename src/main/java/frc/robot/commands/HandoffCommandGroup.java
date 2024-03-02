package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.pivot.PivotHomeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HandoffCommandGroup extends SequentialCommandGroup {

    private PivotSubsystem pivotSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    public HandoffCommandGroup(PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                new PivotHomeCommand(pivotSubsystem).until(() -> pivotSubsystem.getEncAngle() >= .48).andThen(
                        new StartEndCommand(() -> armSubsystem.moveUp(.4), armSubsystem::stop).until(() ->
                                armSubsystem.getEnc() <= .055).andThen(
                                new StartEndCommand(() -> shooterSubsystem.spinNeo(),
                                        shooterSubsystem::stopFlywheel).alongWith(
                                                new StartEndCommand(() -> intakeSubsystem.roll(-.35), intakeSubsystem::stop))
                        ).until(shooterSubsystem::getBeamBreak)
                        )
                );

        addRequirements(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem);

    }
}