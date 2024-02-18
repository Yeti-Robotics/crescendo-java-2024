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
        motorConfig1.SoftwareLimitSwitch = ClimberConstants.CLIMBER_SOFT_LIMIT_1;
        motorConfig2.SoftwareLimitSwitch = ClimberConstants.CLIMBER_SOFT_LIMIT_2;
        motorConfig1.FutureProofConfigs = true;
        motorConfig2.FutureProofConfigs = true;

        // Fix sammy perlmen's issue :)
        talon1Config.apply(motorConfig1);
        talon2Config.apply(motorConfig2);
    }

    public void setClimberBrake(double servoPosition) {
        climberBrake1.set(servoPosition);
        climberBrake2.set(servoPosition);
    }

    @Override
    public void periodic() {
    }

    public void climbUp() {
        climberFalcon2.set(ClimberConstants.CLIMB_SPEED);
        climberFalcon1.set(ClimberConstants.CLIMB_SPEED);
    }

    public void climbDown() {
        climberFalcon2.set(-ClimberConstants.CLIMB_SPEED);
        climberFalcon1.set(-ClimberConstants.CLIMB_SPEED);
    }

    public void stopClimb() {
        climberFalcon1.set(0);
        climberFalcon2.set(0);
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

