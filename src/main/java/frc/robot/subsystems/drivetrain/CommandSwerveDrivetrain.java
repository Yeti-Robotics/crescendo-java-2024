package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    StructArrayPublisher<SwerveModuleState> publisher;

    private final Rotation2d BluePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedPerspectiveRotation = Rotation2d.fromDegrees(0);
    private final SwerveRequest.ApplyChassisSpeeds AutoReq = new SwerveRequest.ApplyChassisSpeeds();

    private boolean hasAppliedPerspective = false;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();

        setDriveCurrentLimits();
        setDriveVoltageLimits();
        setAzimuthCurrentLimits();
        setAzimuthVoltageLimits();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();

        setDriveCurrentLimits();
        setDriveVoltageLimits();
        setAzimuthCurrentLimits();
        setAzimuthVoltageLimits();

        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveRad = 0;
        for(var moduleLocation : m_moduleLocations) {
            driveRad = Math.max(driveRad, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getChassisSpeeds,
                (speeds) -> this.setControl(AutoReq.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10,0,0),
                        new PIDConstants(10,0,0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveRad,
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if(alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },

                this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setDriveCurrentLimits() {
        var currentLimitConfigs = new CurrentLimitsConfigs();

        for(var module : Modules) {
            var currentConfig = module.getDriveMotor().getConfigurator();
            currentConfig.refresh(currentLimitConfigs);

            currentLimitConfigs.SupplyCurrentLimit = DriveConstants.SUPPLY_CURRENT_LIMIT;
            currentLimitConfigs.SupplyCurrentLimitEnable = DriveConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
            currentLimitConfigs.SupplyCurrentThreshold = DriveConstants.SUPPLY_CURRENT_LIMIT_CURRENT_THRESHOLD;
            currentLimitConfigs.SupplyTimeThreshold = DriveConstants.SUPPLY_CURRENT_LIMIT_TIME_THRESHOLD;

            currentConfig.apply(currentLimitConfigs);
        }
    }

    public void setAzimuthCurrentLimits() {
        var currentLimitConfigs = new CurrentLimitsConfigs();

        for (var module : Modules) {
            var currentConfig = module.getSteerMotor().getConfigurator();
            currentConfig.refresh(currentLimitConfigs);

            currentLimitConfigs.SupplyCurrentLimit = DriveConstants.SUPPLY_CURRENT_LIMIT;
            currentLimitConfigs.SupplyCurrentLimitEnable = DriveConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
            currentLimitConfigs.SupplyCurrentThreshold = DriveConstants.SUPPLY_CURRENT_LIMIT_CURRENT_THRESHOLD;
            currentLimitConfigs.SupplyTimeThreshold = DriveConstants.SUPPLY_CURRENT_LIMIT_TIME_THRESHOLD;

            currentConfig.apply(currentLimitConfigs);
        }
    }

    public void setDriveVoltageLimits() {
            var voltageLimitConfigs = new VoltageConfigs();

            for(var module : Modules) {
                var currentConfig = module.getDriveMotor().getConfigurator();

                currentConfig.refresh(voltageLimitConfigs);

                voltageLimitConfigs.PeakForwardVoltage = DriveConstants.PEAK_FORWARD_VOLTAGE;
                voltageLimitConfigs.PeakReverseVoltage = DriveConstants.PEAK_REVERSE_VOLTAGE;

                currentConfig.apply(voltageLimitConfigs);
            }
        }

    public void setAzimuthVoltageLimits() {
        var voltageLimitConfigs = new VoltageConfigs();

        for(var module : Modules) {
            var currentConfig = module.getSteerMotor().getConfigurator();

            currentConfig.refresh(voltageLimitConfigs);

            voltageLimitConfigs.PeakForwardVoltage = DriveConstants.PEAK_FORWARD_VOLTAGE;
            voltageLimitConfigs.PeakReverseVoltage = DriveConstants.PEAK_REVERSE_VOLTAGE;

            currentConfig.apply(voltageLimitConfigs);
        }
    }

    @Override
    public void periodic() {
        publisher.set(super.getState().ModuleStates);
        if(!hasAppliedPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == DriverStation.Alliance.Red ? RedPerspectiveRotation
                                : BluePerspectiveRotation);
                hasAppliedPerspective = true;
            });
        }
    }
}



