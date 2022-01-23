package frc.robot.libraries;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Deadzone {
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double invlerp(double a, double b, double v) {
        return (v - a) / (b - a);
    }

    // Fixed, previous version didnt work with negative values, stupid me
    public static double deadZone(double value, double deadzone) {
        if (Math.abs(value) > deadzone)
            return Math.signum(value) * invlerp(deadzone, 1.0, Math.abs(value));
        else
            return 0.;
    }

    public static double cutOff(double value, double cutoff) {
        if (Math.abs(value) < cutoff)
            return 0;
        else
            return value;
    }

    // TODO: test this
    public static Vector2d deadBand(Vector2d v, double deadzone) {
        double l = v.magnitude();
        if (l > deadzone) {
            double f = invlerp(deadzone, 1., l);
            return new Vector2d(v.x / l * f, v.y / l * f);
        } else
            return new Vector2d(0., 0.);
    }
}
