package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    TalonFX leftTalon;
    TalonFX rightTalon;
    public ShooterSubsystem() {

        leftTalon = new TalonFX(15, "canivoreBus");
        rightTalon = new TalonFX(16, "canivoreBus");

//        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
//        TalonFXConfigurator talonFXConfigurator = leftTalon.getConfigurator();
//
//
//        leftTalon.setControl(new Follower(rightTalon.getDeviceID(),true));
//        talonFXConfigurator.apply(talonFXConfiguration);
    }

    public void spinShooter(double speed) {
        rightTalon.set(speed);
        leftTalon.set(speed);
    }

    public void stopShooter() {
        rightTalon.stopMotor();
    }
}

