/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.libraries;

import edu.wpi.first.math.controller.PIDController;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;

	public Gains(double _kP, double _kI, double _kD, double _kF) {
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
	}

	public Gains(double _kP, double _kI, double _kD) {
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = 0.0;
	}

	public PIDController getPID() {
		return new PIDController(kP, kI, kD);
	}
}