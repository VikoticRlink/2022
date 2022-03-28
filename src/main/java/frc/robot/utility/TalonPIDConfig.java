// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Container for CTRE Talon motor controller parameters */
public class TalonPIDConfig extends Object {
    public double kP;   /** Proportional gain */
    public double kI;   /** Integral gain */
    public double kD;   /** Derivative gain */
    public double kF;   /** Arbitrary feed-forward gain */

    public int pidIndex;    /** PID gain index */
    public int timeoutMs;   /** Timeout used for configuring gains */
    
    /** Default-constructs an object with all gains set to zero */
    public TalonPIDConfig() {
        kP = kI = kD = kF = 0.0; 
        pidIndex = 0;
        timeoutMs = 0;
    }

    /** Creates an instance of the object 
     * @param p     Proportional gain
     * @param i     Integral gain
     * @param d     Derivative gain
     * @param f     Arbitrary feed-forward gain
     * @param pidIndex  PID index gains will be applied to
     * @param timeoutMs Timeout used when applying gains
    */
    public TalonPIDConfig(double p, double i, double d, double f,
                          int pidIndex, int timeoutMs) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        this.pidIndex = pidIndex;
        this.timeoutMs = timeoutMs;
    }
}
