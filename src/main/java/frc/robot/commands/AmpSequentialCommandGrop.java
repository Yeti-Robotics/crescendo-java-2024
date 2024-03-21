package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpSequentialCommandGrop extends SequentialCommandGroup {

    public AmpSequentialCommandGrop(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {

        super(
                new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.AMP)),
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.23)),
                new InstantCommand(() -> shooterSubsystem.setVelocity(30)));
    }
}