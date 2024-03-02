package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurnToTarget  extends SwerveRequest.FieldCentricFacingAngle {

    Translation2d turnToTarget;

    @Override
    public StatusCode apply(
            SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        this.TargetDirection = parameters.currentPose.getTranslation().minus(turnToTarget).getAngle();

        return super.apply(parameters, modulesToApply);
    }

    @Override
    public FieldCentricFacingAngle withTargetDirection(Rotation2d target) {
        return this;
    }

    public void setTurnToTarget(Translation2d target) {
        turnToTarget = target;
    }

    public FieldCentricFacingAngle withTurnToTarget(Translation2d target) {
        return this;
    }

    public Rotation2d getTarget() {
        return this.TargetDirection;
    }
}
