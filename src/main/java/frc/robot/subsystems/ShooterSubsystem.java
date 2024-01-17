package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TalonFXConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftKraken;
    private final TalonFX rightKraken;
    public enum ShooterModes {
        DEFAULT,
        SPEAKER,
        AMP,
        TRAP,
        BUMP
    }

    public static ShooterModes shooterModes;

    public enum ShooterStatus {
        FORWARD,
        BACKWARD,
        OFF
    }

    public static ShooterStatus shooterStatus;
    public static double setpoint = 0;
    public static boolean atSetpoint = false;
    public static boolean isShooting = false;
    public static double velocity = 0;



    public ShooterSubsystem() {
        leftKraken = new TalonFX(ShooterConstants.SHOOTER_LEFT_MOTOR);
        rightKraken = new TalonFX(ShooterConstants.SHOOTER_RIGHT_MOTOR);
        leftKraken.setControl(new DifferentialFollower(rightKraken.getDeviceID(), true));
        var rightMotorConfigurator = rightKraken.getConfigurator();
        var rightMotorConfiguration = new TalonFXConfiguration();

        rightMotorConfiguration.MotorOutput.Inverted = ShooterConstants.SHOOTER_INVERSION;
        rightMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfiguration.CurrentLimits = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        rightMotorConfiguration.Slot0 = ShooterConstants.SLOT_0_CONFIGS;
         shooterStatus = ShooterStatus.OFF;
         shooterModes = ShooterModes.SPEAKER;



                rightMotorConfigurator.apply(rightMotorConfiguration);
    }

    @Override
    public void periodic() {

        MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(
                velocity, 0, true, ShooterConstants.SHOOTER_F, 0, false, false, false);

        switch (shooterModes) {
            case SPEAKER:
             //   if() Pose get y is above a certain value {
                //   Our main linear regression for RPM}
            break;

            case BUMP:
                velocity = ShooterConstants.BUMP_FIRE_VEL;
                rightKraken.setControl(motionMagicVelocityVoltage);
             break;
            case AMP:
                velocity = ShooterConstants.AMP_VEL;
                rightKraken.setControl(motionMagicVelocityVoltage);
                break;
            case TRAP:
                velocity = ShooterConstants.TRAP_VEL;
                 rightKraken.setControl(motionMagicVelocityVoltage);
            case DEFAULT:
                velocity = ShooterConstants.DEFAULT_VEL;
                rightKraken.setControl(motionMagicVelocityVoltage);
                break;
        }



    }

    public void shootFlywheel(double speed) {
        rightKraken.set(speed);
        shooterStatus = ShooterStatus.FORWARD;
    }


    public void stopFlywheel() {
        rightKraken.stopMotor();
        shooterStatus = ShooterStatus.OFF;
    }

    public double getRightEncoder() {
        return rightKraken.getRotorVelocity().getValue();
    }

    public  double getLeftEncoder() {
        return leftKraken.getRotorVelocity().getValue();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getFlywheelRPM() {
        return getAverageEncoder() * ShooterConstants.PULLEY_RATIO * (600.0 / TalonFXConstants.COUNTS_PER_REV);
    }

    public double flywheelMPS(double rpm) {
        return (ShooterConstants.FLYWHEEL_DIAMETER_M * Math.PI) * (rpm * 60.0);
    }

    public void setSetpoint(double setpoint) {
        ShooterSubsystem.setpoint = setpoint;
    }

}

