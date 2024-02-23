package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolator;

public class ShooterStateData {

    public static final Interpolator<ShooterStateData> interpolator = new Interpolator<ShooterStateData>() {
        @Override
        public ShooterStateData interpolate(
                ShooterStateData initialValue, ShooterStateData finalValue, double t) {

            double initialAngle = initialValue.angle;
            double finalAngle = finalValue.angle;
            double initialRPS = initialValue.rps;
            double finalRPS = finalValue.rps;

            return new ShooterStateData(
                    initialAngle + t * (finalAngle - initialAngle), initialRPS + t * (finalRPS - initialRPS));

        }
    };

    public final double angle;
    public final double rps;

    public ShooterStateData(double angle, double rps) {
        this.angle = angle;
        this.rps = rps;
    }
}