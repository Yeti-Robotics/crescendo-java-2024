package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final DigitalInput magSwitch;

    public class ElevatorConstants {
        public static ElevatorConstants.ElevatorPositions elevatorPositions = ElevatorConstants.ElevatorPositions.DOWN;

        public static int STAGES = 3;
        public static final double ELEVATOR_DISTANCE_PER_PULSE = 1; //PLACEHOLDER
        public static int ELEVATOR_ID = 10;
        public static final double ELEVATOR_P = 45;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0.;
        public static final double ELEVATOR_G = 0.0;
        public static final double ELEVATOR_A = 0.05;
        public static final double ELEVATOR_V = 8.0;
        public static final double PROFILE_V = 0.2;
        public static final double PROFILE_A = .5;

        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs().withKP(ELEVATOR_P).withKI(ELEVATOR_I).withKD(ELEVATOR_D)
                .withKA(ELEVATOR_A).withKV(ELEVATOR_V).withKG(ELEVATOR_G).withGravityType(GravityTypeValue.Elevator_Static);

        public static final double ELEVATOR_POSITION_STATUS_FRAME = 0.05;
        public static final double ELEVATOR_VELOCITY_STATUS_FRAME = 0.01;

        public static final CurrentLimitsConfigs ELEVATOR_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(65).withSupplyCurrentLimit(75).withSupplyTimeThreshold(1).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(75);
        public static final SoftwareLimitSwitchConfigs ELEVATOR_SOFT_LIMIT = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false).withForwardSoftLimitThreshold(0).withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(0);

        public enum ElevatorPositions {
            DOWN(0),
            AMP(18);
            public final double distanceEl;
            public final double sensorUnitsEl;

            ElevatorPositions(double distance) {
                this.distanceEl = distance;
                this.sensorUnitsEl = (distance / STAGES) / ELEVATOR_DISTANCE_PER_PULSE;
            }
        }
    }

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID, Constants.TalonFXConstants.CANIVORE_NAME);
        magSwitch = new DigitalInput(9);
        var ElConfigurator = elevatorMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.CurrentLimits = ElevatorConstants.ELEVATOR_CURRENT_LIMIT;
        talonFXConfiguration.SoftwareLimitSwitch = ElevatorConstants.ELEVATOR_SOFT_LIMIT;
        talonFXConfiguration.Slot0 = ElevatorConstants.SLOT_0_CONFIGS;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.PROFILE_V;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.PROFILE_A;


        elevatorMotor.getRotorVelocity().waitForUpdate(ElevatorConstants.ELEVATOR_VELOCITY_STATUS_FRAME);
        elevatorMotor.getRotorPosition().waitForUpdate(ElevatorConstants.ELEVATOR_POSITION_STATUS_FRAME);

        ElConfigurator.apply(talonFXConfiguration);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);


    }

    public void goUp(double speed) {
        elevatorMotor.set(Math.abs(speed));
    }

    public void goDown(double speed) {
        elevatorMotor.set(-Math.abs(speed));
   }

    public void stop() {
        elevatorMotor.stopMotor();
    }

    public boolean getmagSwitch() {
        return magSwitch.get();
    }

    public double getEncoder() {
        return elevatorMotor.getRotorPosition().getValue();
    }


    public void setRotations(double rotations) {
        elevatorMotor.setPosition(rotations);
    }

    // why do we have two of these methods????

    private void setPosition(ElevatorConstants.ElevatorPositions position) {
        ElevatorConstants.elevatorPositions = position;
        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(
                position.distanceEl, true, 0.0, 0,
                true, false, false);
        if (position == ElevatorSubsystem.ElevatorConstants.ElevatorPositions.AMP) {
            motionMagicVoltage.withFeedForward(10);
        }
        elevatorMotor.setControl(motionMagicVoltage);
    }

    public Command setPositionDown(){
        //setPosition(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN)
      return run(() -> setPosition(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN));
    }

    public Command setPositionAmp(){
        //setPosition(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.AMP)
        return run(() -> setPosition(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.AMP));
    }

    public Command goDownAndStop(double speed){
        return startEnd(() -> goDown(speed), this::stop);
    }


    @Override
    public void periodic() {
        if (getmagSwitch() && ElevatorConstants.elevatorPositions == ElevatorConstants.ElevatorPositions.DOWN) {
            setRotations(0);
        }
        SmartDashboard.putData("Elevator motor", elevatorMotor);
        SmartDashboard.putData("Elevator magswitch", magSwitch);
    }


    public Command positionDownCmd() {
        return runOnce(() -> setPosition(ElevatorConstants.ElevatorPositions.DOWN));
    }
}
