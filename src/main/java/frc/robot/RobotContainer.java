// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.led.BlinkLimeLightCommand;
import frc.robot.commands.led.SetLEDToRGBCommand;
import frc.robot.commands.pivot.PivotHomeCommand;
import frc.robot.commands.pivot.PivotMoveCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.controllerUtils.ButtonHelper;
import frc.robot.util.controllerUtils.ControllerContainer;
import frc.robot.util.controllerUtils.MultiButton;


public class RobotContainer {

    public EventLoop eventLoop;
    private final BooleanEvent limitSwitchTriggered;
    public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final PivotSubsystem pivotSubsystem = new PivotSubsystem();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private PivotLimitSwitchCommand pivotLimitSwitchCommand;
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public ControllerContainer controllerContainer = new ControllerContainer();
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());

    public final CommandXboxController joystick = new CommandXboxController(1); // My joystick
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
        eventLoop = new EventLoop();
        ledSubsystem.setDefaultCommand(new SetLEDToRGBCommand(ledSubsystem, 255, 0, 0, 1.0, 0));

        NamedCommands.registerCommand("shootBump", new SequentialCommandGroup(
                new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.55)),
                new InstantCommand(() -> shooterSubsystem.bumpFire()),
                new WaitCommand(.75),
                new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop).withTimeout(1))
        ));
        var field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
      
       limitSwitchTriggered = new BooleanEvent(eventLoop, () -> pivotSubsystem.getForwardLimitSwitch() || pivotSubsystem.getReverseLimitSwitch());
       configureBindings();
    }


    private void configureBindings() {


        buttonHelper.createButton(1, 0, new ShooterStateCommand(drivetrain, pivotSubsystem, shooterSubsystem, intakeSubsystem),MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(5, 0, new StartEndCommand(() -> shooterSubsystem.bumpFire(), shooterSubsystem::stopFlywheel).alongWith(new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.53))),MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(10,0,  new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(2), MultiButton.RunCondition.WHEN_PRESSED);
//        buttonHelper.createButton(1, 0, new ConditionalCommand(new RunCommand(() ->intakeSubsystem.roll(.5)), new RunCommand(() -> intakeSubsystem.roll(0)), intakeSubsystem::getBeamBreak), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(2, 0, new StartEndCommand(() -> intakeSubsystem.roll(-.65), intakeSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(3, 0, new StartEndCommand(() -> climberSubsystem.climbUp(), () -> climberSubsystem.stopClimb()), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(7, 0, new StartEndCommand(() -> climberSubsystem.climbDown(), () -> climberSubsystem.stopClimb()).until(() -> climberSubsystem.getRightEncoder() >= .200195 && climberSubsystem.getLeftEncoder() <= -0.242676), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(4, 0,  new StartEndCommand(() -> elevatorSubsystem.goDown(0.2), elevatorSubsystem::stop).withTimeout(0.3).andThen(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.DOWN)).andThen(new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.5)))), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(6, 0, new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopNeo).until(shooterSubsystem::getBeamBreak), MultiButton.RunCondition.WHILE_HELD);
        buttonHelper.createButton(9, 0, new StartEndCommand(() -> shooterSubsystem.spinFeeder(-0.25), shooterSubsystem::stopNeo), MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(9, 0, new StartEndCommand(() -> armSubsystem.moveUp(0.3), armSubsystem::stop), MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(10, 0, new StartEndCommand(() -> armSubsystem.moveUp(.5), armSubsystem::stop).until(() -> armSubsystem.getEnc() >= 0.).andThen(armSubsystem::setMotorsBrake), MultiButton.RunCondition.WHEN_PRESSED);
//        buttonHelper.createButton(5, 0, new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(
//                () -> armSubsystem.getEnc() <= .6 && armSubsystem.getEnc() >= .58).alongWith(new PivotHomeCommand(pivotSubsystem)), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(8,0,new InstantCommand(climberSubsystem::engageBrake),MultiButton.RunCondition.WHEN_PRESSED);
//        buttonHelper.createButton(11, 0,new StartEndCommand(shooterSubsystem::shootAmp, shooterSubsystem::stopFlywheel),MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(12,0,new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.TRAP)), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(
                12,0,new InstantCommand(climberSubsystem::disengageBrake),MultiButton.RunCondition.WHEN_PRESSED);
        limitSwitchTriggered.castTo(Trigger::new).onTrue(new PivotLimitSwitchCommand(pivotSubsystem));
        buttonHelper.createButton(11, 0,new StartEndCommand(shooterSubsystem::shootAmp, shooterSubsystem::stopFlywheel),MultiButton.RunCondition.WHILE_HELD);
//        buttonHelper.createButton(12,0,new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.TRAP)), MultiButton.RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(
                12,0,new InstantCommand(climberSubsystem::disengageBrake),MultiButton.RunCondition.WHEN_PRESSED);
        limitSwitchTriggered.castTo(Trigger::new).onTrue(pivotLimitSwitchCommand);
        //TO TEST




        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.
                                        withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
                                        .withRotationalRate(-joystick.getRightX() * DriveConstants.MaFxAngularRate) // Drive counterclockwise with negative X (left)
                ));

//        intakeSubsystem.setDefaultCommand(
//                new ConditionalCommand(new BlinkLimeLightCommand(), new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME), intakeSubsystem.s))
//        );
//        joystick.a().whileTrue(drivetraixn.applyRequest(() -> brake));
        joystick.x().whileTrue(new AutoAimCommand(drivetrain,joystick::getLeftX,joystick::getLeftY))
        joystick.x().whileTrue(new AutoAimCommand(drivetrain,() -> -joystick.getLeftY(), () -> -joystick.getLeftX()));
        joystick.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        joystick.rightBumper().whileTrue(new StartEndCommand(() -> intakeSubsystem.roll(.70), intakeSubsystem::stop));

        joystick.leftBumper().onTrue(new StartEndCommand(() -> armSubsystem.moveDown(.5), armSubsystem::stop).until(
                () -> armSubsystem.getEnc() <= .54 && armSubsystem.getEnc() >= .52).alongWith(new PivotHomeCommand(pivotSubsystem)));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> elevatorSubsystem.setPosition2(ElevatorConstants.ElevatorPositions.AMP)).andThen(new StartEndCommand(() -> pivotSubsystem.moveDown(0.25), pivotSubsystem::stop).unless(
                () -> pivotSubsystem.getEncAngle() < 0.4).withTimeout(0.6).andThen(new InstantCommand(() -> pivotSubsystem.setPivotPosition(0.26)).unless(() -> !elevatorSubsystem.getmagSwitch()))));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.a().onTrue(
                new StartEndCommand(() -> shooterSubsystem.setVelocity(45), shooterSubsystem::stopFlywheel).withTimeout(0.5)
        );

        joystick.rightTrigger().whileTrue(new StartEndCommand(() -> shooterSubsystem.spinNeo(), shooterSubsystem::stopFlywheel).alongWith(new StartEndCommand(() -> intakeSubsystem.roll(-1), intakeSubsystem::stop)));
//        joystick.leftTrigger().whileTrue(new StartEndCommand(() -> pivotSubsystem.moveUp(.15), pivotSubsystem::stop).until(() -> pivotSubsystem.getEncAngle() <= ShooterConstants.SHOOTER_MAP().get(drivetrain.getState().Pose.getX()).angle));

        joystick.povUp().onTrue(new HandoffCommandGroup(pivotSubsystem, armSubsystem, shooterSubsystem, intakeSubsystem).withTimeout(2));
        joystick.povDown().onTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(ElevatorConstants.ElevatorPositions.DOWN)));
        joystick.povLeft().onTrue(new StartEndCommand(() -> pivotSubsystem.moveUp(0.1), pivotSubsystem::stop));
        joystick.povLeft().onTrue(new StartEndCommand(() -> pivotSubsystem.moveUp(-0.1), pivotSubsystem::stop));
//        joystick.povRight().onTrue(new InstantCommand(() -> pivotSubsystem.setPivotPosition(.5)));


    }




        public Command getAutonomousCommand () {
            return new InstantCommand();
        }

}
