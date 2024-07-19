package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurnToAngle extends FieldCentricFacingAngle {

    Rotation2d fieldCentricTargetDirection;

    @Override
    public StatusCode apply(
            SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
            // This is an angle from the frame of the reference of the field. Subtract
            // the operator persepctive to counteract CTRE adding it later
            this.TargetDirection =
                    this.fieldCentricTargetDirection.minus(parameters.operatorForwardDirection);
        }
        return super.apply(parameters, modulesToApply);
    }

    public Rotation2d getTargetDirection() {
        return this.TargetDirection;
    }

    public void setFieldCentricTargetDirection(Rotation2d fieldCentricTargetDirection) {
        this.fieldCentricTargetDirection = fieldCentricTargetDirection;
    }

    public Rotation2d getFieldCentricTargetDirection() {
        return this.fieldCentricTargetDirection;
    }

    public TurnToAngle withFieldCentricTargetDirection(Rotation2d fieldCentricTargetDirection) {
        setFieldCentricTargetDirection(fieldCentricTargetDirection);
        return this;
    }
}
