package frc.robot.subsystems;


//import com.ctre.phoenix6.StatusSignal;

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
    private final Servo climberBrake;

    public ClimberSubsystem() {
        climberFalcon1 = new TalonFX(ClimberConstants.CLIMBER_TALON_1);
        climberFalcon2 = new TalonFX(ClimberConstants.CLIMBER_TALON_2);

        climberBrake = new Servo(ClimberConstants.SERVO_CHANNEL);
        climberFalcon1.setInverted(true);
        climberFalcon2.setControl(new Follower(climberFalcon1.getDeviceID(), false));

        var talon1Config = climberFalcon1.getConfigurator();
        var talon2Config = climberFalcon2.getConfigurator();

        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // motorConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        motorConfig.FutureProofConfigs = true;

        // Fix sammy perlmen's issue :)
        talon1Config.apply(motorConfig);
        talon2Config.apply(motorConfig);
    }

    public void setClimberBrake(double servoPosition) {
        climberBrake.set(servoPosition);
    }

    @Override
    public void periodic() {
    }

    public void climbUp() {
        climberFalcon1.set(ClimberConstants.CLIMB_SPEED);
    }

    public void climbDown() {
        climberFalcon1.set(-ClimberConstants.CLIMB_SPEED);
    }

    public void stopClimb() {
        climberFalcon1.set(0);
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


