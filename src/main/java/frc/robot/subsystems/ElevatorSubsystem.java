package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.CANCoderConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.ElevatorPositions;
import frc.robot.constants.TalonFXConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX elevatorMotor;

    private final DigitalInput magSwitch;
    private ElevatorConstants.ElevatorPositions elevatorPositions = ElevatorConstants.ElevatorPositions.DOWN;

    public ElevatorSubsystem(){
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID, TalonFXConstants.CANIVORE_NAME);
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

    public void goUp(double speed){
        elevatorMotor.set(Math.abs(speed));
    }

    public void goDown(double speed){
        elevatorMotor.set(-Math.abs(speed));
    }

    public void stop(){
        elevatorMotor.stopMotor();
    }

    public boolean getmagSwitch(){
        return magSwitch.get();
    }

    public double getEncoder() {
        return elevatorMotor.getRotorPosition().getValue();
    }


    public void setRotations(double rotations) {
        elevatorMotor.setPosition(rotations);
    }
    public void setPosition(ElevatorConstants.ElevatorPositions position){
        elevatorPositions = position;

        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(
                position.distanceEl, true, 0.0 , 0,
                true, false, false);

        elevatorMotor.setControl(motionMagicVoltage.withPosition(position.distanceEl));
    }


    @Override
    public void periodic() {
        if(getmagSwitch() && elevatorPositions == ElevatorPositions.DOWN) {
            setRotations(0);
        }
    }
}