package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TalonFXConstants;
import frc.robot.subsystems.drivetrain.NaivePublisher;
import frc.robot.util.ShooterStateData;

public class ShooterSubsystem extends SubsystemBase {

    public static ShooterModes shooterModes;
    public static ShooterStatus shooterStatus;
    public static double velocity = 0;
    private final TalonFX leftKraken;
    private final TalonFX rightKraken;
    private final TalonFX neo;
    private final DigitalInput beamBreak;
    MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    private final StatusSignal<Double> leftVel;
    private final StatusSignal<Double> rightVel;


    public ShooterSubsystem() {
        leftKraken = new TalonFX(ShooterConstants.SHOOTER_LEFT_MOTOR, TalonFXConstants.CANIVORE_NAME);
        rightKraken = new TalonFX(ShooterConstants.SHOOTER_RIGHT_MOTOR, TalonFXConstants.CANIVORE_NAME);
        var rightMotorConfigurator = rightKraken.getConfigurator();
        var leftMotorConfigurator = leftKraken.getConfigurator();
        var rightMotorConfiguration = new TalonFXConfiguration();

        rightMotorConfiguration.MotorOutput.Inverted = ShooterConstants.SHOOTER_INVERSION;
        rightMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfiguration.CurrentLimits = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        rightMotorConfiguration.Slot0 = ShooterConstants.SLOT_0_CONFIGS;
        leftKraken.setControl(new Follower(rightKraken.getDeviceID(), false));

        shooterStatus = ShooterStatus.OFF;
        shooterModes = ShooterModes.TRAP;

        leftVel = leftKraken.getVelocity();
        rightVel = rightKraken.getVelocity();

        neo = new TalonFX(ShooterConstants.SHOOTER_NEO, "canivoreBus");

        beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

        var motionMagicConfigs = rightMotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;
        rightMotorConfigurator.apply(rightMotorConfiguration);
        leftMotorConfigurator.apply(rightMotorConfiguration);
    }

    @Override
    public void periodic() {
        leftVel.refresh();
        rightVel.refresh();

        SmartDashboard.putData("shooter beam break", beamBreak);
        SmartDashboard.putNumber("left rps:", leftVel.getValue());
        SmartDashboard.putNumber("right rps:", rightVel.getValue());
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    private void stopFeeder() {
        neo.stopMotor();
    }

    private void spinFeeder(double speed) {
        neo.set(speed);
    }

    public void stopShooter() {
        rightKraken.stopMotor();
        leftKraken.stopMotor();
        neo.stopMotor();
        shooterStatus = ShooterStatus.OFF;
    }

    public void setDualVelocity(double leftVel, double rightVel) {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(leftVel));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(rightVel));
    }

    private void setVelocity(double velocity) {
        setDualVelocity(velocity, velocity);
    }

    public double getVelocity() {
        if (!leftVel.hasUpdated() || !rightVel.hasUpdated()) {
            leftVel.waitForUpdate(ShooterConstants.SHOOTER_STATUS_FRAME_SECONDS);
            rightVel.waitForUpdate(ShooterConstants.SHOOTER_STATUS_FRAME_SECONDS);
        }

        return (leftVel.getValue() + rightVel.getValue()) / 2;
    }

    public Command updateVelocityWith(NaivePublisher<ShooterStateData> publisher) {
        NaivePublisher.NaiveSubscriber<ShooterStateData> shooterDataSubscriber = new NaivePublisher.NaiveSubscriber<ShooterStateData>(shooterStateData -> setVelocity(shooterStateData.rps));
        return startEnd(() -> publisher.subscribe(shooterDataSubscriber), () -> publisher.cancel(shooterDataSubscriber));
    }

    public Command shooterBumpFire() {
        return runOnce(() -> setDualVelocity(60, 100));
    }

    public Command shooterTrap() {
        return startEnd(() -> setDualVelocity(35, 80), this::stopShooter);
    }

    public Command setVelocityAndStop(double vel) {
        return startEnd(() -> setVelocity(vel), this::stopShooter);
    }

    public Command setVelocityContinuous(double vel) {
        return runOnce(() -> setVelocity(vel));
    }

    public Command spinFeederAndStop(double vel) {
        return Commands.startEnd(() -> spinFeeder(vel), this::stopFeeder);
    }

    public Command spinFeederMaxAndStop() {
        return spinFeederAndStop(-1);
    }

    public enum ShooterModes {
        DEFAULT,
        SPEAKER,
        AMP,
        TRAP,
        BUMP
    }

    public enum ShooterStatus {
        FORWARD,
        BACKWARD,
        OFF
    }
}
