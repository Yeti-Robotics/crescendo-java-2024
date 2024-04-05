package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

import java.lang.reflect.Field;
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
    private Translation2d speakerCenter;
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
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize() {
        poseY = commandSwerveDrivetrain.getState().Pose.getY();
        poseX = commandSwerveDrivetrain.getState().Pose.getX();
        rps = ShooterConstants.SHOOTER_MAP().get(poseY).rps;
        angle = ShooterConstants.SHOOTER_MAP().get(poseY).angle;
        speakerPose = AllianceFlipUtil.apply(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0)));
    }



    public void execute() {
        System.out.print("Robot pose: ");

       Pose2d robotPose = commandSwerveDrivetrain.getState().Pose;
       Pose2d relativeSpeaker = robotPose.relativeTo(speakerPose);
        yawTarget = Rotation2d.fromRadians(Math.atan2(relativeSpeaker.getY(), relativeSpeaker.getX()) + Math.PI);
        double distance = relativeSpeaker.getTranslation().getNorm();
        System.out.println(distance);
        rps = ShooterConstants.SHOOTER_MAP().get(distance).rps;
        angle = ShooterConstants.SHOOTER_MAP().get(distance).angle;
        SmartDashboard.putNumber("shooter angle", angle);

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
        pivotSubsystem.setPivotPosition(0.5);
    }
}
