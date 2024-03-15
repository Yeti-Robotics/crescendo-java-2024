package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;


public class ShooterStateCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final PivotSubsystem pivotSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final IntakeSubsystem intakeSubsystem;
    private double poseY = 0;
    private double poseX = 0;
    private double rps = 0;
    private double angle = 0;
    private final double botAngle = 0;
    private Pose2d speakerPose;
    private Rotation2d yawTarget;
    DoubleSupplier xVel;
    DoubleSupplier yVel;

    public ShooterStateCommand(CommandSwerveDrivetrain commandSwerveDrivetrain,
                               PivotSubsystem pivotSubsystem,
                               ShooterSubsystem shooterSubsystem,
                               IntakeSubsystem intakeSubsystem) {
       this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.pivotSubsystem = pivotSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.commandSwerveDrivetrain, this.pivotSubsystem);
    }

    @Override
    public void initialize() {
        poseY = commandSwerveDrivetrain.getState().Pose.getY();
        poseX = commandSwerveDrivetrain.getState().Pose.getX();
        rps = ShooterConstants.SHOOTER_MAP().get(poseY).rps;
        angle = ShooterConstants.SHOOTER_MAP().get(poseY).angle;
        speakerPose = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                ? new Pose2d(0.0, 5.55, Rotation2d.fromRotations(0))
                : new Pose2d(16.5, 5.55, Rotation2d.fromRotations(0));
    }


    public void execute() {

       Pose2d robotPose = commandSwerveDrivetrain.getState().Pose;
       Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
        yawTarget = Rotation2d.fromRadians(Math.atan2(relativeSpeaker.getY(), relativeSpeaker.getX()) + Math.PI);
        double distance = relativeSpeaker.getTranslation().getNorm();
        rps = ShooterConstants.SHOOTER_MAP().get(distance).rps;
        angle = ShooterConstants.SHOOTER_MAP().get(distance).angle;
       /* SwerveRequest pointCmd = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withTargetDirection(yawTarget);*/
        //commandSwerveDrivetrain.setControl(pointCmd);
        shooterSubsystem.setVelocity(rps);
        pivotSubsystem.setPivotPosition(angle);
        intakeSubsystem.roll(-.25);
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("angle", angle);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
        intakeSubsystem.stop();
    }
}
