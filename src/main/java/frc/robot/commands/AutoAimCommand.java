package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

import java.util.function.DoubleSupplier;

public class AutoAimCommand extends Command {

    private CommandSwerveDrivetrain drivetrain;
    TurnToAngle swerveRequest;
    private DoubleSupplier xVelSupplier;
    private TurnToPoint poseAimRequest;
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

        poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5,0,0);
        poseAimRequest.HeadingController.enableContinuousInput(-180.0,180.0);
    }

    @Override
    public void initialize() {
        poseY = drivetrain.getState().Pose.getX();
        Translation2d speakerCenter = AllianceFlipUtil.apply(
                FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
        );

        poseAimRequest.setPointToFace(speakerCenter);
    }

    @Override
    public void execute() {

        drivetrain.setControl(
                poseAimRequest
        );
    }



    @Override
    public boolean isFinished() {
        return false;
    }
}

