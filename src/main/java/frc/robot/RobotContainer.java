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
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.HandoffCommandGroup;
import frc.robot.commands.ShooterStateCommand;
import frc.robot.commands.led.SetLEDToRGBCommand;
import frc.robot.commands.pivot.PivotHomeCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {


    public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final PivotSubsystem pivotSubsystem = new PivotSubsystem();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    ControllerContainer controllerContainer = new ControllerContainer();
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());

    private final CommandXboxController joystick = new CommandXboxController(1); // My joystick
    final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
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
        ledSubsystem.setDefaultCommand(new SetLEDToRGBCommand(ledSubsystem, 255, 0, 0, 1.0, 0));
        configureBindings();
    }


    private void configureBindings() {

        buttonHelper.createButton(1, 0, new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop),MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(10,0,  new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(2), MultiButton.RunCondition.WHEN_PRESSED);
//        buttonHelper.createButton(1, 0, new ConditionalCommand(new RunCommand(() ->intakeSubsystem.roll(.5)), new RunCommand(() -> intakeSubsystem.roll(0)), intakeSubsystem::getBeamBreak), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(2, 0, new StartEndCommand(() -> intakeSubsystem.roll(-.70), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(3, 0, new StartEndCommand(() -> climberSubsystem.climbUp(), () -> climberSubsystem.stopClimb()), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(7, 0, new StartEndCommand(() -> climberSubsystem.climbDown(), () -> climberSubsystem.stopClimb()).until(() -> climberSubsystem.getRightEncoder() >= .200195 && climberSubsystem.getLeftEncoder() <= -0.242676), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(4, 0, new StartEndCommand(() -> shooterSubsystem.setVelocity(125), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> pivotSubsystem.moveUp(.45), pivotSubsystem::stop).until(() -> pivotSubsystem.getEncAngle() < .45)), MultiButton.RunCondition.WHILE_HELD); //45 amp 52 bumpfire
        buttonHelper.createButton(6, 0, new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopNeo).until(shooterSubsystem::getBeamBreak), MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(10, 0, new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() >= 0.).andThen(armSubsystem::setMotorsBrake), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(5, 0, new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(
                () -> armSubsystem.getEnc() <= .6 && armSubsystem.getEnc() >= .5).alongWith(new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.5))), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(8,0,new InstantCommand(climberSubsystem::engageBrake),MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(9,0,new InstantCommand(climberSubsystem::disengageBrake),MultiButton.RunCondition.WHEN_PRESSED);

        //TO TEST



        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.
                                        withVelocityX(-joystick.getLeftY() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(-joystick.getLeftX() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                                        .withRotationalRate(-joystick.getRightX() * DriveConstants.MaFxAngularRate) // Drive counterclockwise with negative X (left)
                ));

//        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        //joystick.rightBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.AMP)).alongWith(new StartEndCommand(() -> pivotSubsystem.moveUp(.1), pivotSubsystem::stop).until(() -> pivotSubsystem.getEncAngle() < .27)));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x()
                .whileTrue(
                        new StartEndCommand(() -> pivotSubsystem.moveUp(.1), pivotSubsystem::stop).until(() -> pivotSubsystem.getEncAngle() < .2));
        joystick.y()
                .whileTrue(
                        new StartEndCommand(() -> pivotSubsystem.moveDown(.1), pivotSubsystem::stop));
        joystick.leftTrigger().whileTrue(
                new ShooterStateCommand(drivetrain, pivotSubsystem, shooterSubsystem, intakeSubsystem)


        );

        joystick.rightTrigger().whileTrue(new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)));
//        joystick.leftTrigger().whileTrue(new StartEndCommand(() -> pivotSubsystem.moveUp(.15), pivotSubsystem::stop).until(() -> pivotSubsystem.getEncAngle() <= ShooterConstants.SHOOTER_MAP().get(drivetrain.getState().Pose.getX()).angle));

        joystick.povUp().onTrue(new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(2));
//        joystick.povDown().onTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.DOWN)).until(elevatorSubsystem::getmagSwitch).andThen(new InstantCommand(() -> elevatorSubsystem.setRotations(0))));
        joystick.povLeft().onTrue(new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.5)).until(pivotSubsystem::isMotionFinished));


    }



        public Command getAutonomousCommand () {
            return new InstantCommand();
        }

}
