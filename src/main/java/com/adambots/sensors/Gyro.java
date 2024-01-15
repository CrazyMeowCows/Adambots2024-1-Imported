/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Class that represents a Gyroscope sensor
 */
public class Gyro extends BaseSensor implements edu.wpi.first.wpilibj.interfaces.Gyro {

    private static Gyro _instance = null;
    private static edu.wpi.first.wpilibj.interfaces.Gyro _navx = null;
    private static boolean _isDisabled = false; // Sometimes NAVX breaks everything and if so, disable it

    private static class InstanceHolder {
        public static final Gyro instance = new Gyro();
    }

    private Gyro(){
        try {
            if (_navx == null){
                if (!_isDisabled){
                    _navx = new AHRS(Port.kMXP); // although this brings in depency, using setDevice this can be overwritten before calling getInstance
                    ((AHRS)_navx).enableBoardlevelYawReset(true);
                } else {
                    _navx = new edu.wpi.first.wpilibj.interfaces.Gyro() {

                        @Override
                        public void close() throws Exception {
                            // TODO Auto-generated method stub
                            
                        }

                        @Override
                        public void calibrate() {
                            // TODO Auto-generated method stub
                            
                        }

                        @Override
                        public void reset() {
                            // TODO Auto-generated method stub
                            
                        }

                        @Override
                        public double getAngle() {
                            // TODO Auto-generated method stub
                            return 0;
                        }

                        @Override
                        public double getRate() {
                            // TODO Auto-generated method stub
                            return 0;
                        }
                        
                    };
                }
            }

            _navx.reset();
        } catch (Exception e) {
            System.out.println("NavX Initialization Failed");
        }
    }

    public static void setDevice(AHRS navx){
        
        if (_navx == null)
            _navx = navx; // It should only be set once.
    }

    public static Gyro getInstance(){
        // if (_instance == null){
        //     _instance = new Gyro();
        // }
        _instance = InstanceHolder.instance;
        return _instance;
    }

    public void reset(){
        _navx.reset();
        // _navx.enableBoardlevelYawReset(true);
    }

    public void lowLevelReset(){
        if (!_isDisabled)
            ((AHRS)_navx).enableBoardlevelYawReset(true);
        
        _navx.reset();
    }

    /**
     * Use only in Robot Init
     */
    public void calibrationCheck() {

        if (!_isDisabled){

            boolean isCalibrating = ((AHRS)_navx).isCalibrating();
            
            if (isCalibrating) {
                System.out.println("In Calibration Check - waiting 2 seconds to complete Gyro Calibration");
                Timer.delay(2); // wait 2 seconds to let it complete calibration
            }
        }
    }

    public float getRoll() {
        if (!_isDisabled)
            return ((AHRS)_navx).getRoll();
        else
            return 0f;
    }

    public float getPitch() {
        if (!_isDisabled){
            return ((AHRS)_navx).getPitch();
        } else {
            return 0f;
        }
    }

    /**
     * Returns current Yaw value between a range of -180 to 180 degrees. 
     * Note that this Yaw value can accumulate errors over a period of time.
     * @return
     */
    public float getYaw() {
        if (!_isDisabled){

            return ((AHRS)_navx).getYaw();
        } else {
            return 0f;
        }
    }

    @Override
    public void close() throws Exception {
        _navx.close();

    }

    /**
     * Returns accumulated Yaw angles and can be > than 360 degrees. This is in contrast to getYaw that returns -180 to 180 degree values.
     */
    @Override
    public double getAngle() {
        return _navx.getAngle();
    }

    @Override
    public double getRate() {
        return _navx.getRate();
    }

    @Override
    public void calibrate() {
        // do nothing - NavX automatically calibrates at startup
        
    }
}
