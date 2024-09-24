package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterValues {
    public static final Interpolator<ShooterTarget> interpolator = (initialValue, finalValue, t) -> {
        double initialAngle = initialValue.angle;
        double finalAngle = finalValue.angle;
        double initialRPS = initialValue.rps;
        double finalRPS = finalValue.rps;

        return new ShooterTarget(
                initialAngle + t * (finalAngle - initialAngle), initialRPS + t * (finalRPS - initialRPS));
    };

    public static InterpolatingTreeMap<Double, ShooterTarget> SHOOTER_MAP = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), interpolator);
    public static InterpolatingTreeMap<Double, ShooterTarget> SHUTTLE_MAP = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), interpolator);

    static {
        // TODO: decrease angles by around 0.05 to tune
        SHOOTER_MAP.put(1.375, new ShooterTarget(0.5, 125));
        SHOOTER_MAP.put(1.7, new ShooterTarget(.485, 125));
        SHOOTER_MAP.put(2.0, new ShooterTarget(0.478, 125));
        SHOOTER_MAP.put(2.3, new ShooterTarget(0.47, 125));
        SHOOTER_MAP.put(2.65, new ShooterTarget(.465, 125));
        SHOOTER_MAP.put(2.8, new ShooterTarget(.4625, 125));
        SHOOTER_MAP.put(3.0, new ShooterTarget(0.46, 125));
        SHOOTER_MAP.put(3.8, new ShooterTarget(0.443, 125));

        SHUTTLE_MAP.put(8.0, new ShooterTarget(0.5, 125));
    }
}
