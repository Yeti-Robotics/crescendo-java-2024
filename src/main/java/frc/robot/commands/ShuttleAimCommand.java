package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LimelightHelpers;

import java.util.function.DoubleSupplier;


public class ShuttleAimCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private DoubleSupplier xVelSupplier;
    private TurnToPoint poseAimRequest;
    private DoubleSupplier yVelSupplier;
    double currentTag;

    public ShuttleAimCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier xVelSupplier,
        DoubleSupplier yVelSupplier) {

            this.drivetrain = drivetrain;
            this.xVelSupplier = xVelSupplier;
            this.yVelSupplier = yVelSupplier;

            addRequirements(drivetrain);

            poseAimRequest = new TurnToPoint();
            poseAimRequest.HeadingController.setPID(5,0,0);
            poseAimRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
        }

        @Override
        public void initialize() {
            currentTag = LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME);

            Translation2d shuttleTarget = AllianceFlipUtil.apply(
                    FieldConstants.Shuttle.shuttleTargetZone
            );

            poseAimRequest.setPointToFace(shuttleTarget);
        }

    @Override
    public void execute() {
        if(LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME) == currentTag) {
            drivetrain.setControl(
                    poseAimRequest.withVelocityX(xVelSupplier.getAsDouble() * 1.5).withVelocityY(yVelSupplier.getAsDouble() * 1.5)
            );
        } else {
            end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}