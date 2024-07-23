package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConfiguratorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeKraken;

    private final DigitalInput beamBreak;

    public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double INTAKE_POSITION_STATUS_FRAME = 0.05;
    public static final double INTAKE_VELOCITY_STATUS_FRAME = 0.01;

    private boolean prevBreak = false;

    public IntakeSubsystem() {
        intakeKraken = new TalonFX(ConfiguratorConstants.INTAKE_KRAKEN_ID, "canivoreBus");
        var intakeConfigurator = intakeKraken.getConfigurator();
        var configs = new TalonFXConfiguration();

        beamBreak = new DigitalInput(2);
        configs.MotorOutput.Inverted = INTAKE_INVERSION;
        configs.MotorOutput.NeutralMode = INTAKE_NEUTRAL_MODE;
        configs.FutureProofConfigs = ConfiguratorConstants.TALON_FUTURE_PROOF;
        intakeKraken.getRotorVelocity().waitForUpdate(INTAKE_VELOCITY_STATUS_FRAME);
        intakeKraken.getRotorPosition().waitForUpdate(INTAKE_POSITION_STATUS_FRAME);
        intakeConfigurator.apply(configs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("intake kraken", intakeKraken);
        SmartDashboard.putData("intake beam break", beamBreak);
    }

    public void rollIn(double speed) {
        intakeKraken.set(Math.abs(speed));
    }

    public void rollOut(double speed) {
        intakeKraken.set(-Math.abs(speed));
    }

    public void roll(double speed) {
        intakeKraken.set(speed);
    }

    public void stop() {
        intakeKraken.stopMotor();
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    public Command rollCmd(double val) {
        return startEnd(() -> roll(val), this::stop);
    }
}
