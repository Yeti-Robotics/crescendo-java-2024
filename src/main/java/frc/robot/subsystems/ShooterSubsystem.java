package frc.robot.subsystems;

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

public class ShooterSubsystem extends SubsystemBase {

    public static ShooterModes shooterModes;
    public static ShooterStatus shooterStatus;
    public static double velocity = 0;
    private final TalonFX leftKraken;
    private final TalonFX rightKraken;
    private final TalonFX neo;
    private final DigitalInput beamBreak;
    MotionMagicVelocityVoltage motionMagicVelocityVoltage;

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
        SmartDashboard.putData("shooter beam break", beamBreak);
        SmartDashboard.putNumber("left rps:", leftKraken.getVelocity().getValue());
        SmartDashboard.putNumber("right rps:", rightKraken.getVelocity().getValue());
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    public void spinNeo() {
        neo.set(-1);
    }

    public void spinFeeder(double speed) {
        neo.set(speed);
    }

    public void stopNeo() {
        neo.stopMotor();
    }

    public void shootFlywheel(double speed) {
        rightKraken.set(speed);
        leftKraken.set(speed);
        neo.set(0.9); //
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void stopFlywheel() {
        rightKraken.stopMotor();
        leftKraken.stopMotor();
        neo.stopMotor();
        shooterStatus = ShooterStatus.OFF;
    }

    public void setVelocity(double vel) {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(vel));
    }

    public void bumpFire() {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(60));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(100));
    }

    public Command bumpFireCmd() {
        return runOnce(this::bumpFire);
    }

    public Command spinNeoCmd() {
        return Commands.startEnd(this::spinNeo, this::stopFlywheel);
    }

    public Command setVelocityCmd(double vel) {
        return startEnd(() -> setVelocity(vel), this::stopFlywheel);
    }

    public Command setVelocityInstantCommand(double vel) {
        return runOnce(() -> setVelocity(vel));
    }

    public Command spinFeederCmd(double vel) {
        return startEnd(() -> spinFeeder(vel), this::stopNeo);
    }

    public Command shootTrapCmd() {
        return startEnd(this::shootTrap, this::stopFlywheel);
    }

    public void shootTrap() {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(35));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(80));
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
