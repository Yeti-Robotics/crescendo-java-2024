package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    public static final int SHOOTER_NEO = 16;
    public static final int BEAM_BREAK = 0;

    private final TalonFX feederMotor;
    private final DigitalInput beamBreak;

    public FeederSubsystem() {
        feederMotor = new TalonFX(SHOOTER_NEO, "canivoreBus");
        beamBreak = new DigitalInput(BEAM_BREAK);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("shooter beam break", beamBreak);
    }

    private boolean isBeamBroken() {
        return !beamBreak.get();
    }

    private Command setSpeed(double speed) {
        return startEnd(() -> feederMotor.set(speed), feederMotor::stopMotor);
    }

    public Command ingest(double speed) {
        return setSpeed(speed).until(this::isBeamBroken);
    }

    public Command feed(double speed) {
        return setSpeed(speed).until(() -> !isBeamBroken());
    }
}
