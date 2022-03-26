package frc.robot.libraries;

public class AxisConverter {
    /**
     * STANDARD COORDINATE SYSTEM:
     * 1st number is front-back (positive is forwards, ranges from -1 to 1)
     * 2nd number is left-right (positive is right, ranges from -1 to 1)
     * 3rd number is rotation (positive is CW, ranges from -1 to 1)
     */
    public static double[] convertFromNAVX(double[] inp) {
        if (inp.length == 3) {
            double[] rtn = {inp[0], -1 * inp[1], -1 * inp[2]};
            return rtn;
        } else {
            return null;
        }
        /**
         * NOT CONFIDENT
         * forward = +x
         * backward = -x
         * left = +y
         * right = -y
         * CCW = +z
         * CW = -z
         */
    }

    public static double[] convertFromJoystick(double[] inp) {
        if (inp.length == 3) {
            double[] rtn = {-1 * inp[1], inp[0], inp[2]};
            return rtn;
        } else {
            return null;
        }
        /**
         * forward = -y
         * backward = +y
         * left = -x
         * right = +x
         * CCW = -z
         * CW = +z
         */
    }

    public static double[] convertToCartesian(double[] inp) {
        if (inp.length == 3) {
            double[] rtn = {inp[1], -1 * inp[0], -1 * inp[2]};
            return rtn;
        } else {
            return null;
        }
        /**
         * NOT CONFIDENT
         * forward = +y
         * backward = -y
         * left = +x
         * right = -x
         * CCW = +z
         * CW = -z
         */
    }
}