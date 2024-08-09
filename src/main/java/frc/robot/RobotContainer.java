// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    public final CommandXboxController joystick = new CommandXboxController(1); // My joystick
    public final VisionSubsystem vision = new VisionSubsystem();

    final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(CommandSwerveDrivetrain.MaFxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public ControllerContainer controllerContainer = new ControllerContainer();
    public SendableChooser<AutoConstants.AutoMode> autoChooser;
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());
    private boolean autoNeedsRebuild = true;
    private Command auto;

    private final RobotCommands robotCommands = new RobotCommands(
            intake, pivot, elevator, shooter, vision, drivetrain, arm
    );

    public RobotContainer() {
        NamedCommands.registerCommand("shootBump", Commands.sequence(
                pivot.adjustPivotPositionTo(0.55),
                shooter.shooterBumpFire(),
                Commands.waitSeconds(.75), // is this waiting for a specific speed or something? should prob be replaced
                shooter.spinFeederMaxAndStop().alongWith(intake.rollOut(-1).withTimeout(1))
        ));

        var field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);

        PathPlannerLogging.setLogTargetPoseCallback(pose ->
                field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(poses ->
                field.getObject("path").setPoses(poses)
        );


        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        Telemetry logger = new Telemetry(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND);
        drivetrain.registerTelemetry(logger::telemeterize);

        configureBindings();

        buildAutoChooser();
        rebuildAutoIfNecessary();

        vision.ledOff();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SmartDashboard.putNumber("shooterstate-position", 0.5);
    }

    private void configureBindings() {

        buttonHelper.createButton(1, 0, robotCommands.setShooterState(), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(8, 0, shooter.setVelocityAndStop(-70), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(7, 0, shooter.setVelocityAndStop(15), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(5, 0,
                pivot.adjustPivotPositionTo(.42).andThen(
                        new StartEndCommand(() -> arm.moveUp(.7), arm::stop).until(() ->
                                arm.getEnc() <= ArmSubsystem.ARM_HANDOFF_POSITION).andThen(
                                shooter.spinFeederAndStop(-0.3).alongWith(intake.rollOut(-.2))
                        ).until(shooter::getBeamBreak),
                        pivot.movePivotPositionTo(PivotSubsystem.PivotPosition.HANDOFF)
                ).andThen(
                        shooter.setVelocityContinuous(100)).andThen(
                        intake.rollOut(-.1).withTimeout(0.2)).andThen(
                        Commands.waitSeconds(.45)).andThen(
                        shooter.spinFeederMaxAndStop().alongWith(intake.rollOut(-1).withTimeout(1)).andThen(
                                robotCommands.handoff().withTimeout(2))
                ), MultiButton.RunCondition.WHEN_PRESSED);

        buttonHelper.createButton(10, 0, robotCommands.handoff().withTimeout(2), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(2, 0, intake.rollOut(-.65), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(4, 0, new StartEndCommand(() -> elevator.goDown(0.2), elevator::stop).withTimeout(0.3).andThen(new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.ElevatorPositions.DOWN)).andThen(pivot.movePivotPositionTo(PivotSubsystem.PivotPosition.HANDOFF))), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(6, 0, shooter.spinFeederAndStop(-.1).alongWith(intake.rollIn(0.5)), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(9,
                0, new InstantCommand(() -> elevator.setPosition2(ElevatorSubsystem.ElevatorPositions.AMP)).andThen(pivot.moveDown(-0.25).unless(
                        () -> pivot.getEncoderAngle() < 0.4).withTimeout(0.6).andThen(pivot.adjustPivotPositionTo(0.03).unless(() -> !elevator.getmagSwitch()))), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(11, 0, shooter.shooterTrap(), MultiButton.RunCondition.WHILE_HELD);

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        // +X in velocity = forward, -Y in joystick = forward
                                        .withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +Y in velocity = left, -X in joystick = left
                                        .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +rotational rate = counterclockwise (left), -X in joystick = left
                                        .withRotationalRate(-joystick.getRightX() * CommandSwerveDrivetrain.MaFxAngularRate)
                ));

        // Lock on to speaker
        joystick.leftTrigger().whileTrue(new AutoAimCommand(drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()));

        // Swerve lock
        joystick.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Reset the field-centric heading
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        // Suck in note
        joystick.rightBumper().whileTrue(intake.rollIn(.7));

        // Arm down
        joystick.leftBumper().onTrue(new StartEndCommand(() -> arm.moveDown(.5), arm::stop, arm).until(
                () -> arm.getEnc() <= .54 && arm.getEnc() >= .52).alongWith(pivot.movePivotPositionTo(PivotSubsystem.PivotPosition.HANDOFF)));

        // (This is unassigned on the gamepad map??)
        joystick.a().onTrue(
                shooter.setVelocityAndStop(45).withTimeout(0.5)
        );
        // Shoot
        joystick.rightTrigger().whileTrue(shooter.spinFeederMaxAndStop());
        // Handoff
        joystick.povUp().onTrue(robotCommands.handoff().withTimeout(2));
        // Move elevator down
        joystick.povDown().onTrue(elevator.positionDownCmd());
        // (These are also unassigned on the gamepad map?)
        joystick.povLeft().whileTrue(pivot.moveUpWithBrake(0.05, -0.01));
        joystick.povRight().whileTrue(pivot.moveDownWithBrake(-0.05, 0.01));
        // Spin feeder
        joystick.x().whileTrue(shooter.spinFeederAndStop(.3));
    }

    public void updateOdometryVision() {
        var visionResult = vision.getTargetingResults();

        if (DriverStation.getAlliance().isPresent()) { // this is here because it's sometimes not present during simulation
            if (visionResult != null && visionResult.valid && Math.abs(drivetrain.getPigeon2().getRate()) < 230) {
                Pose2d llPose = visionResult.getBotPose2d_wpiBlue();
                drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            }
        }
    }

    public void buildAutoChooser() {
        var namedCommands = new AutoNamedCommands(intake, shooter, pivot, arm, robotCommands);
        namedCommands.registerCommands();

        autoChooser = new SendableChooser<>();

        for (var autoMode : AutoConstants.AutoMode.values()) {
            autoChooser.addOption(autoMode.name, autoMode);
        }
        // note: setDefaultOption overwrites the name in the map, so we won't have duplicate options
        autoChooser.setDefaultOption(AutoConstants.AutoMode.BUMP_ONLY.name, AutoConstants.AutoMode.BUMP_ONLY);

        autoChooser.onChange(obj -> autoNeedsRebuild = true);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void rebuildAutoIfNecessary() {
        if (autoNeedsRebuild) {
            // why is there a timeout here? can't we use the FMS/practice mode timeout?
            auto = AutoBuilder.buildAuto(autoChooser.getSelected().name).withTimeout(15);
            System.out.println("AUTO NAME: " + auto);
            autoNeedsRebuild = false;
        }
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
