package frc.robot.subsystems;


//import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
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

    private final CANcoder canCoder1;
    private final CANcoder canCoder2;



    public ClimberSubsystem() {
        climberFalcon1 = new TalonFX(ClimberConstants.CLIMBER_TALON_1, "canivoreBus");
        climberFalcon2 = new TalonFX(ClimberConstants.CLIMBER_TALON_2, "canivoreBus");

        canCoder1 = new CANcoder(ClimberConstants.CLIMBER_ENCODER_1, "canivoreBus");
        canCoder2 = new CANcoder(ClimberConstants.CLIMBER_ENCODER_2, "canivoreBus");

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

        motorConfig1.Feedback.withFeedbackRemoteSensorID(canCoder1.getDeviceID());
        motorConfig2.Feedback.withFeedbackRemoteSensorID(canCoder2.getDeviceID());

        var encoderConfig = new CANcoderConfiguration();
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

    public void engageBrake() {
        climberBrake1.set(0.2);
        climberBrake2.set(0.2);
    }

    public void disengageBrake() {
        climberBrake1.set(0.4);
        climberBrake2.set(0.4);
    }

    public double getServo() {
        return climberBrake1.getAngle();
    }

    public double getRightEncoder() {
        return canCoder1.getAbsolutePosition().getValue();
    }

    public double getLeftEncoder() {
        return canCoder2.getAbsolutePosition().getValue();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public boolean atEncoderLimit() {
        return getAverageEncoder() <= ClimberConstants.CLIMBER_TALON_1;
    }


}

