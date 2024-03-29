/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Class that implements a Lidar sensor
 */
public class Lidar extends BaseSensor {
    public double LIDAR_TOLERANCE = 0; // tune
    private Counter counter;
    private DigitalInput _source = null;
	
	public Lidar(int dioPortNumber) {
		super();

        _source = new DigitalInput(dioPortNumber);

		System.out.println("Source: " + _source);
		counter = new Counter(_source);

	    counter.setMaxPeriod(1.0);
	    // Configure for measuring rising to falling pulses
	    counter.setSemiPeriodMode(true);
	    counter.reset();
		// System.out.println("Counter: " + counter);
    }
    
	/**
	 * Take a measurement and return the distance in cm
	 * 
	 * @return Distance in cm
	 */
	public double getDistance() {
		double cm;

		// SmartDashboard.putNumber("Period", counter.getPeriod());

		// if (counter.get() < 1)
			// return 0;

		// while (counter.get() < 1) {
		// 	System.out.println("Lidar: waiting for distance measurement");
		// }
		/* getPeriod returns time in seconds. The hardware resolution is microseconds.
		 * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		 */
		cm = (counter.getPeriod() * 1000000.0 / 10.0);
		return cm;
	}

	/**
	 * Converts getDistance() of cm to inches
	 * @return Distance in inches
	 */
	public double getInches() {
		return getDistance() / 2.54;
    }
    
    /**
     * Gets distance in Feet
     * @return Distance in Feet
     */
    public double getFeet(){
        return getInches() / 12;
    }
}
