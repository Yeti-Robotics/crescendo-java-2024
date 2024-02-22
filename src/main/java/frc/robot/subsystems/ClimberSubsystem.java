package frc.robot.subsystems;


//import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberFalcon1;
    private final TalonFX climberFalcon2;
    private final Servo climberBrake1;
    private final Servo climberBrake2;

    public static final double WPILIB_MIN_SERVO_ANGLE = 0.0; //degrees
    public static final double WPILIB_MAX_SERVO_ANGLE = 360.0; //degrees
    private static final double TIME_TO_SERVO_FULL_EXTENSION = 3.48; //Avg time to move from retract to extend
    private static final double PERCENT_PER_SECOND = 1.00 / TIME_TO_SERVO_FULL_EXTENSION;
    private static final double DEGREES_PER_SECOND = (WPILIB_MAX_SERVO_ANGLE - WPILIB_MIN_SERVO_ANGLE)
            * PERCENT_PER_SECOND;

    private static final double MAX_POSITION = 1.0; //percent servo travel to max hood position
    private static final double MIN_POSITION = 0.0; //percent servo travel to min hood position

    private static final double MAX_SERVO_PWM = 2.0; //ms
    private static final double MIN_SERVO_PWM = 1.0; //ms
    private static final double SERVO_RANGE = MAX_SERVO_PWM - MIN_SERVO_PWM;
    private static final double CENTER_SERVO_PWM = 1.5; //ms
    private static final double SERVO_DEADBAND = 0.0; //ms - no deadband

    // pwm values in ms for the max and min angles of the shooter hood
    private static final double HOOD_MAX_PWM = MIN_SERVO_PWM + (SERVO_RANGE * MAX_POSITION);
    private static final double HOOD_MIN_PWM = MIN_SERVO_PWM + (SERVO_RANGE * MIN_POSITION);



    public ClimberSubsystem() {
        climberFalcon1 = new TalonFX(ClimberConstants.CLIMBER_TALON_1, "canivoreBus");
        climberFalcon2 = new TalonFX(ClimberConstants.CLIMBER_TALON_2, "canivoreBus");

        climberBrake1 = new Servo(ClimberConstants.SERVO_CHANNEL);
        climberBrake2 = new Servo(ClimberConstants.SERVO_CHANNEL_2);
//        climberFalcon2.setControl(new Follower(climberFalcon1.getDeviceID(), false));

        var talon1Config = climberFalcon1.getConfigurator();
        var talon2Config = climberFalcon2.getConfigurator();

        var motorConfig1 = new TalonFXConfiguration();
        var motorConfig2 = new TalonFXConfiguration();

        motorConfig1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // motorConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
//        motorConfig1.SoftwareLimitSwitch = ClimberConstants.CLIMBER_SOFT_LIMIT_1;
//        motorConfig2.SoftwareLimitSwitch = ClimberConstants.CLIMBER_SOFT_LIMIT_2;
        motorConfig1.FutureProofConfigs = true;
        motorConfig2.FutureProofConfigs = true;


        // Fix sammy perlmen's issue :)
        talon1Config.apply(motorConfig1);
        talon2Config.apply(motorConfig2);
    }

    public void setClimberBrake() {
        climberBrake1.setSpeed(1);
        climberBrake2.setSpeed(1);
        climberBrake1.setAngle(360);
        climberBrake2.setAngle(360);
    }

    @Override
    public void periodic() {
    }

    public void climbUp() {
        climberFalcon2.set(-ClimberConstants.CLIMB_SPEED);
        climberFalcon1.set(ClimberConstants.CLIMB_SPEED);
    }

    public void climbDown() {
        climberFalcon2.set(ClimberConstants.CLIMB_SPEED);
        climberFalcon1.set(-ClimberConstants.CLIMB_SPEED);
    }

    public void stopClimb() {
        climberFalcon1.set(0);
        climberFalcon2.set(0);
    }

    public double getServo() {
        return climberBrake1.getAngle();
    }

    public double getLeftEncoder() {
        return climberFalcon2.getRotorPosition().getValue();
    }

    public double getRightEncoder() {
        return climberFalcon1.getRotorPosition().getValue();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public boolean atEncoderLimit() {
        return getAverageEncoder() <= ClimberConstants.CLIMBER_TALON_1;
    }

}

