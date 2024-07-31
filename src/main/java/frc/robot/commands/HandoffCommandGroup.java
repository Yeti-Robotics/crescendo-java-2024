package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HandoffCommandGroup extends SequentialCommandGroup {

    public HandoffCommandGroup(PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        super(
                 pivotSubsystem.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF).andThen(
                        new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() ->
                                armSubsystem.getEnc() <= ArmConstants.ARM_HANDOFF_POSITION).andThen(
                                shooterSubsystem.spinFeederAndStop(-0.3).alongWith(intakeSubsystem.rollOut(-0.35))
                        ).until(shooterSubsystem::getBeamBreak)
                ),
                pivotSubsystem.movePivotPositionTo(PivotConstants.PivotPosition.HANDOFF)
        );
    }
}