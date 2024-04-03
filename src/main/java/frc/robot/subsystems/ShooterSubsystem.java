package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TalonFXConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftKraken;
    private final TalonFX rightKraken;
    private final TalonFX neo;
    private final DigitalInput beamBreak;
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
//                velocity, MOTION_MAGIC_ACCELERATION, false, SHOOTER_F, 0, false, false, false);

        var motionMagicConfigs = rightMotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;
        rightMotorConfigurator.apply(rightMotorConfiguration);
        leftMotorConfigurator.apply(rightMotorConfiguration);


    }

    @Override
    public void periodic() {
//        SmartDashboard.putData("left shooter kraken", leftKraken);
//        SmartDashboard.putData("right shooter kraken", rightKraken);
//        SmartDashboard.putData("feeder shooter kraken", neo);
        SmartDashboard.putData("shooter beam break", beamBreak);
//
//        MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(
//                velocity, 0, true, ShooterConstants.SHOOTER_F, 0, false, false, false);
//
//        switch (shooterModes) {
//            case SPEAKER:
//                shooterStatus = ShooterStatus.FORWARD;
//                //   if() Pose get y is above a certain value {
//                //   Our main linear regression for RPM}
//            break;
//
//            case BUMP:
//                velocity = ShooterConstants.BUMP_FIRE_VEL;
//                shooterStatus = ShooterStatus.FORWARD;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//             break;
//            case AMP:
//                velocity = ShooterConstants.AMP_VEL;
//                shooterStatus = ShooterStatus.FORWARD;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//                break;
//            case TRAP:
//                velocity = ShooterConstants.TRAP_VEL;
//                shooterStatus = ShooterStatus.FORWARD;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//            case DEFAULT:
//                velocity = ShooterConstants.DEFAULT_VEL;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//                break;
//        }
//
//
//
    }
    public boolean getBeamBreak(){
        return !beamBreak.get();
    }

    public void spinNeo() {
        neo.set(ShooterConstants.STAGE_SPEED);
    }
    public void spinFeeder(double speed) {
        neo.set(speed);
    }

    public void stageNeo() {
        neo.set(-.2);
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

    public void setMode(ShooterModes mode) {
        shooterModes = mode;
    }


    public void stopFlywheel() {
        rightKraken.stopMotor();
        leftKraken.stopMotor();
        neo.stopMotor();
        shooterStatus = ShooterStatus.OFF;
    }


    public void setVelocity(double vel){

        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(vel));
//        neo.set(1);


    }

    public void bumpFire() {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(60));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(100));
    }

    public void shootAmp() {
        leftKraken.setControl(motionMagicVelocityVoltage.withVelocity(40));
        rightKraken.setControl(motionMagicVelocityVoltage.withVelocity(15));
    }


    //    public void testMotionMagic(double vel) {
//        MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(
//                vel, 0, false, SHOOTER_F, 0, false, false, false);
//        rightKraken.setControl(motionMagicVelocityVoltage);
//        leftKraken.setControl(motionMagicVelocityVoltage);
//    }
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
