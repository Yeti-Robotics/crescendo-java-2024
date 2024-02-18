// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IndexCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    ControllerContainer controllerContainer = new ControllerContainer();
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());

    private final CommandXboxController joystick = new CommandXboxController(1); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(DriveConstants.MaFxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);


    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {

        buttonHelper.createButton(1, 0, new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop).until(intakeSubsystem::getBeamBreak).
                andThen(new RunCommand(() -> armSubsystem.moveUp(.5)).withTimeout(2).andThen(armSubsystem::stop)), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(11,0, new RunCommand(() -> armSubsystem.moveUp(.5)).withTimeout(.5), MultiButton.RunCondition.WHEN_PRESSED);
//        buttonHelper.createButton(1, 0, new ConditionalCommand(new RunCommand(() ->intakeSubsystem.roll(.5)), new RunCommand(() -> intakeSubsystem.roll(0)), intakeSubsystem::getBeamBreak), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(2, 0, new StartEndCommand(() -> intakeSubsystem.roll(-.70), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(3, 0, new StartEndCommand(() -> climberSubsystem.climbUp(), () -> climberSubsystem.stopClimb()), MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(7, 0, new StartEndCommand(() -> climberSubsystem.climbDown(), () -> climberSubsystem.stopClimb()), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(4, 0, new StartEndCommand(() -> shooterSubsystem.motionMagicTest(75), shooterSubsystem::stopFlywheel), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(6, 0, new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(8, 0, new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel)
                .alongWith(new StartEndCommand(() -> shooterSubsystem.spinNeo(),
                        shooterSubsystem::stopFlywheel))
                .until(shooterSubsystem::getBeamBreak), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(5, 0, new StartEndCommand(() -> armSubsystem.moveUp(.3), armSubsystem::stop).until(armSubsystem::atAngle), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(10, 0, new StartEndCommand(() -> armSubsystem.moveDown(.3), armSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);



        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.
                                        withVelocityX(-joystick.getLeftY() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(-joystick.getLeftX() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                                        .withRotationalRate(-joystick.getRightX() * DriveConstants.MaFxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.pov(0)
                .whileTrue(
                        new RunCommand(() -> climberSubsystem.climbUp()));
        joystick.pov(180)
                .whileTrue(
                        new RunCommand(() -> climberSubsystem.climbDown()));
        joystick.a().onTrue(
                new RunCommand(() -> climberSubsystem.setClimberBrake(5))
        );



    }

        public Command getAutonomousCommand () {
            return new InstantCommand();
        }

}