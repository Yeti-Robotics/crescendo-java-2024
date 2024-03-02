package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

import java.util.function.DoubleSupplier;

public class AutoAimCommand extends Command {

    private CommandSwerveDrivetrain drivetrain;
    TurnToTarget swerveRequest;
    private DoubleSupplier xVelSupplier;
    private DoubleSupplier yVelSupplier;
    private double poseY = 0;

    public AutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier) {

        this.drivetrain = drivetrain;
        this.xVelSupplier = xVelSupplier;
        this.yVelSupplier = yVelSupplier;

        addRequirements(drivetrain);

        swerveRequest = new TurnToTarget();
        swerveRequest.HeadingController.setPID(10,0,0);
        swerveRequest.HeadingController.enableContinuousInput(-180.0,180.0);
    }

    @Override
    public void initialize() {
        poseY = drivetrain.getState().Pose.getY();
        Translation2d speakerCenter = AllianceFlipUtil.apply(
                FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
        );

        swerveRequest.setTurnToTarget(speakerCenter);
    }

    @Override
    public void execute() {

        if (poseY > 2.58) {
            double xVel = -(xVelSupplier.getAsDouble());
            double yVel = -(yVelSupplier.getAsDouble());
            drivetrain.setControl(
                    swerveRequest.withVelocityX(xVel)
                            .withVelocityY(yVel)
                            .withDeadband(0.1)
                            .withRotationalDeadband(1.5 * Math.PI * 0.1)
            );
        }
        else {
            double xVel = xVelSupplier.getAsDouble();
            double yVel = yVelSupplier.getAsDouble();
            drivetrain.setControl(
                    swerveRequest.withVelocityX(xVel)
                            .withVelocityY(yVel)
                            .withDeadband(0.1)
                            .withRotationalDeadband(1.5 * Math.PI * 0.1)
            );
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

