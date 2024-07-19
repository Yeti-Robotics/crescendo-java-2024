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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.HandoffCommandGroup;
import frc.robot.commands.PivotLimitSwitchCommand;
import frc.robot.commands.ShooterStateCommand;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.commands.led.SetLEDToRGBCommand;
import frc.robot.commands.pivot.PivotHomeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
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
    public final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final CommandXboxController joystick = new CommandXboxController(1); // My joystick
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();

    final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(DriveConstants.MaFxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    public ControllerContainer controllerContainer = new ControllerContainer();
    public SendableChooser<AutoConstants.AutoMode> autoChooser;
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());
    private boolean autoNeedsRebuild = true;
    private Command auto;

    public RobotContainer() {
        ledSubsystem.setDefaultCommand(new SetLEDToRGBCommand(ledSubsystem, 255, 0, 0, 1.0, 0));

        NamedCommands.registerCommand(
                "shootBump",
                new SequentialCommandGroup(
                        new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.55)),
                        new InstantCommand(() -> shooterSubsystem.bumpFire()),
                        new WaitCommand(.75),
                        new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel)
                                .alongWith(
                                        new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)
                                                .withTimeout(1))));
        var field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    field.getObject("target pose").setPose(pose);
                });

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> {
                    field.getObject("path").setPoses(poses);
                });

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);

        configureBindings();

        buildAutoChooser();
        rebuildAutoIfNecessary();

        visionSubsystem.ledOff();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SmartDashboard.putNumber("shooterstate-position", 0.5);
    }

    private void configureBindings() {
        buttonHelper.createButton(
                1,
                0,
                new ShooterStateCommand(drivetrain, pivotSubsystem, shooterSubsystem, intakeSubsystem),
                MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(
                8,
                0,
                new StartEndCommand(
                        () -> shooterSubsystem.setVelocity(-70), shooterSubsystem::stopFlywheel),
                MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(
                7,
                0,
                new StartEndCommand(() -> shooterSubsystem.setVelocity(15), shooterSubsystem::stopFlywheel),
                MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(
                5,
                0,
                new SequentialCommandGroup(
                        new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.42))
                                .andThen(
                                        new StartEndCommand(() -> armSubsystem.moveUp(.7), armSubsystem::stop)
                                                .until(() -> armSubsystem.getEnc() <= ArmConstants.ARM_HANDOFF_POSITION)
                                                .andThen(
                                                        new StartEndCommand(
                                                                        () -> shooterSubsystem.spinFeeder(-0.3),
                                                                        shooterSubsystem::stopFlywheel)
                                                                .alongWith(
                                                                        new StartEndCommand(
                                                                                () -> intakeSubsystem.roll(-.2), intakeSubsystem::stop)))
                                                .until(shooterSubsystem::getBeamBreak),
                                        new PivotHomeCommand(pivotSubsystem)),
                        new InstantCommand(() -> shooterSubsystem.setVelocity(100)),
                        new StartEndCommand(() -> intakeSubsystem.roll(-.1), intakeSubsystem::stop)
                                .withTimeout(0.2),
                        new WaitCommand(.45),
                        new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel)
                                .alongWith(
                                        new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)
                                                .withTimeout(1),
                                        new HandoffCommandGroup(
                                                        pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem)
                                                .withTimeout(2))),
                MultiButton.RunCondition.WHEN_PRESSED);

        buttonHelper.createButton(
                10,
                0,
                new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem)
                        .withTimeout(2),
                MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(
                2,
                0,
                new StartEndCommand(() -> intakeSubsystem.roll(-.65), intakeSubsystem::stop),
                MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(
                4,
                0,
                new StartEndCommand(() -> elevatorSubsystem.goDown(0.2), elevatorSubsystem::stop)
                        .withTimeout(0.3)
                        .andThen(
                                new InstantCommand(
                                                () ->
                                                        elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.DOWN))
                                        .andThen(new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.5)))),
                MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(
                6,
                0,
                new StartEndCommand(() -> shooterSubsystem.spinFeeder(-0.1), shooterSubsystem::stopNeo)
                        .alongWith(
                                new StartEndCommand(() -> intakeSubsystem.rollOut(0.5), intakeSubsystem::stop)),
                MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(
                9,
                0,
                new InstantCommand(
                                () -> elevatorSubsystem.setPosition2(ElevatorConstants.ElevatorPositions.AMP))
                        .andThen(
                                new StartEndCommand(() -> pivotSubsystem.moveDown(0.25), pivotSubsystem::stop)
                                        .unless(() -> pivotSubsystem.getEncAngle() < 0.4)
                                        .withTimeout(0.6)
                                        .andThen(
                                                new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.03))
                                                        .unless(() -> !elevatorSubsystem.getmagSwitch()))),
                MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(
                11,
                0,
                new StartEndCommand(shooterSubsystem::shootTrap, shooterSubsystem::stopFlywheel),
                MultiButton.RunCondition.WHILE_HELD);

        new Trigger(
                        () -> pivotSubsystem.getForwardLimitSwitch() || pivotSubsystem.getReverseLimitSwitch())
                .onTrue(new PivotLimitSwitchCommand(pivotSubsystem));

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        // +X in velocity = forward, -Y in joystick = forward
                                        .withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +Y in velocity = left, -X in joystick = left
                                        .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +rotational rate = counterclockwise (left), -X in joystick = left
                                        .withRotationalRate(-joystick.getRightX() * DriveConstants.MaFxAngularRate)));

        // Lock on to speaker
        joystick
                .leftTrigger()
                .whileTrue(
                        new AutoAimCommand(drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()));

        // Swerve lock
        joystick
                .b()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Reset the field-centric heading
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        // Suck in note
        joystick
                .rightBumper()
                .whileTrue(new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop));

        // Arm down
        joystick
                .leftBumper()
                .onTrue(
                        new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop)
                                .until(() -> armSubsystem.getEnc() <= .54 && armSubsystem.getEnc() >= .52)
                                .alongWith(new PivotHomeCommand(pivotSubsystem)));

        // (This is unassigned on the gamepad map??)
        joystick
                .a()
                .onTrue(
                        new StartEndCommand(
                                        () -> shooterSubsystem.setVelocity(45), shooterSubsystem::stopFlywheel)
                                .withTimeout(0.5));
        // Shoot
        joystick
                .rightTrigger()
                .whileTrue(
                        new StartEndCommand(shooterSubsystem::spinNeo, shooterSubsystem::stopFlywheel)
                                .alongWith(
                                        new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)));
        // Handoff
        joystick
                .povUp()
                .onTrue(
                        new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem)
                                .withTimeout(2));
        // Move elevator down
        joystick
                .povDown()
                .onTrue(
                        new InstantCommand(
                                () -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.DOWN)));
        // (These are also unassigned on the gamepad map?)
        joystick
                .povLeft()
                .whileTrue(
                        new StartEndCommand(
                                () -> pivotSubsystem.moveUp(0.05), () -> pivotSubsystem.moveDown(0.01)));
        joystick
                .povRight()
                .whileTrue(
                        new StartEndCommand(
                                () -> pivotSubsystem.moveDown(0.05), () -> pivotSubsystem.moveDown(0.01)));
        // Spin feeder
        joystick
                .x()
                .whileTrue(
                        new StartEndCommand(() -> shooterSubsystem.spinFeeder(0.3), shooterSubsystem::stopNeo));
    }

    public void updateOdometryVision() {
        var visionResult = visionSubsystem.getTargetingResults();

        if (DriverStation.getAlliance()
                .isPresent()) { // this is here because it's sometimes not present during simulation
            if (visionResult != null
                    && visionResult.valid
                    && Math.abs(drivetrain.getPigeon2().getRate()) < 230) {
                Pose2d llPose = visionResult.getBotPose2d_wpiBlue();
                drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            }
        }
    }

    public void buildAutoChooser() {

        var namedCommands =
                new AutoNamedCommands(intakeSubsystem, shooterSubsystem, pivotSubsystem, armSubsystem);
        namedCommands.registerCommands();

        autoChooser = new SendableChooser<>();

        for (var autoMode : AutoConstants.AutoMode.values()) {
            autoChooser.addOption(autoMode.name, autoMode);
        }
        // note: setDefaultOption overwrites the name in the map, so we won't have duplicate options
        autoChooser.setDefaultOption(
                AutoConstants.AutoMode.BUMP_ONLY.name, AutoConstants.AutoMode.BUMP_ONLY);

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
