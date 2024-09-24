package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.AllianceFlipUtil;

import java.util.function.Supplier;

public class RobotCommands {

    private final PivotSubsystem pivot;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final FeederSubsystem feeder;
    private final FlywheelSubsystem flywheel;

    public RobotCommands(PivotSubsystem pivot, CommandSwerveDrivetrain commandSwerveDrivetrain, ArmSubsystem arm, ElevatorSubsystem elevator, FeederSubsystem feeder, FlywheelSubsystem flywheel) {
        this.pivot = pivot;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.arm = arm;
        this.elevator = elevator;
        this.feeder = feeder;
        this.flywheel = flywheel;
    }

    private Command updateTarget(Supplier<ShooterTarget> shooterTarget) {
        return pivot.updatePivotPositionWith(() -> shooterTarget.get().angle)
                .alongWith(flywheel.reachVelocity(() -> shooterTarget.get().rps));
    }

    public Command setTarget(double angle, double velocity) {
        return pivot.adjustPivotPositionTo(angle)
                .alongWith(flywheel.reachVelocity(velocity));
    }

    public Command setTargetAndShoot(double angle, double velocity) {
        return setTarget(angle, velocity).andThen(feeder.feed(-1));
    }


    private Command createShooterTargetCommand(Pose2d target, InterpolatingTreeMap<Double, ShooterTarget> targetMap) {
        Supplier<ShooterTarget> shooterTargetSupplier = () -> targetMap.get(commandSwerveDrivetrain.getState().Pose.relativeTo(target).getTranslation().getNorm());
        return updateTarget(shooterTargetSupplier);
    }

    /**
     * Requires
     * Sets shooter state in preparation for shooting
     *
     * @return {@code Command} instance
     */
    public Command setShooterState() {
        return createShooterTargetCommand(AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0))), ShooterValues.SHOOTER_MAP);
    }

    public Command setShuttleState() {
        return createShooterTargetCommand(AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0))), ShooterValues.SHUTTLE_MAP);
    }

    public Command handoff() {
        return arm.moveUpAndStop(.5).until(() ->
                        arm.getEnc() >= ArmSubsystem.ArmConstants.ARM_HANDOFF_POSITION)
                        .andThen(pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF))
                        .andThen(feeder.ingest(-1));
    }

    public Command setAmp() {
        return elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.AMP).andThen(pivot.moveDown(-0.25).unless(
                        () -> pivot.getPosition() < 0.4).withTimeout(0.6).andThen(pivot.adjustPivotPositionTo(0.03).unless(() -> !elevator.getMagSwitch())));
    }

    public Command stowAmp() {
        return pivot.movePivotPositionTo(PivotSubsystem.PivotConstants.PivotPosition.HANDOFF).andThen(elevator.goDownAndStop(0.2).withTimeout(0.3).andThen(elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN)));
    }

    public Command bumpFire(){
        return setTargetAndShoot(0.53, 100);
    }
}
