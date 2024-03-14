package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.pivot.PivotHomeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HandoffCommandGroup extends SequentialCommandGroup {

    public HandoffCommandGroup(PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
//                new StartEndCommand(() -> intakeSubsystem.roll(.60), intakeSubsystem::stop).withTimeout(.2),
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.5)).andThen(
                        new StartEndCommand(() -> armSubsystem.moveUp(.9), armSubsystem::stop).until(() ->
                                armSubsystem.getEnc() <= .125).andThen(
                                new StartEndCommand(() -> shooterSubsystem.spinNeo(),
                                        shooterSubsystem::stopFlywheel).alongWith(
                                        new StartEndCommand(() -> intakeSubsystem.roll(-.20), intakeSubsystem::stop))
                        ).until(shooterSubsystem::getBeamBreak)
                )
        );

        addRequirements(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem);

    }
}