/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */

    Victor LeftMotor_1;
    Victor LeftMotor_2;
    Victor RightMotor_1;
    Victor RightMotor_2;
    Joystick stick;
    Encoder LeftEncoder;
    Encoder RightEncoder;
    SmartDashboardData SDD;
    
    boolean IsTargetDistance;
    double TargetRateL;
    double TargetRateR;
    double LeftCmd;
    double RightCmd;
    int AutonMode;
    
    
    public void robotInit() {
        LeftMotor_1 = new Victor(1);
        LeftMotor_2 = new Victor(2);
        RightMotor_1 = new Victor(3);
        RightMotor_2 = new Victor(4);
        stick = new Joystick(1);
        LeftEncoder = new Encoder(10, 11);
        RightEncoder = new Encoder(3, 4);
        SDD = new SmartDashboardData();
        
        LeftEncoder.setDistancePerPulse(0.0357142857142857);
        RightEncoder.setDistancePerPulse(0.0357142857142857);
        LeftEncoder.start();
        RightEncoder.start();
    }

    public void autonomousInit() {
        LeftEncoder.reset();
        RightEncoder.reset();
        IsTargetDistance = false;
        TargetRateL = TargetRateR = 47.46428571428571;
        AutonMode = 1;
        LeftCmd = 0;
        RightCmd = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        
        double PrateL = (TargetRateL - LeftEncoder.getRate()) / TargetRateL;
        double PrateR = (TargetRateR - RightEncoder.getRate()) / TargetRateR;
        double DeltaDistance = LeftEncoder.getDistance()- RightEncoder.getDistance();
        
        
        switch(AutonMode){
            case 1: {
            if (LeftEncoder.getDistance() < 12 || RightEncoder.getDistance() < 12) {
                if ((PrateL + 0.3) > 0.99) {
                    LeftCmd = 0.99;
                } else {
                    LeftCmd = (0.15 * PrateL + 0.3);
                }
                if ((PrateR + 0.3) > 0.99) {
                    RightCmd = 0.99;
                } else {
                    RightCmd = -(0.15 * PrateR + 0.3);
                }
                
                LeftMotor_1.set(LeftCmd);
                LeftMotor_2.set(LeftCmd);
                RightMotor_1.set(RightCmd);
                RightMotor_2.set(RightCmd);
                break;
            } else {
                LeftCmd = 0;
                RightCmd = 0;
                
                LeftEncoder.reset();
                RightEncoder.reset();
                
                TargetRateL = 47.46428571428571;
                TargetRateR = -47.46428571428571;
                
                AutonMode = 2;
                break;
                } 
            }
            case 2: {
                if(LeftEncoder.getDistance() < 16.10066235){
                    LeftCmd = (0.15 * PrateL + 0.3);
                } else {
                    LeftCmd = 0;
                }
                if(RightEncoder.getDistance() > -16.10066235) {
                    RightCmd = (0.15 * PrateR + 0.3);
                } else {
                    RightCmd = 0;
                } 
                break;
            }
            default: {
                LeftCmd = 0;
                RightCmd = 0;
                break;
            }
        }
        
        drive(LeftCmd, RightCmd);
        
        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        if(stick.getRawAxis(2) < 0.03 && stick.getRawAxis(2) > -0.03){
            LeftCmd = 0;
        } else LeftCmd = -stick.getRawAxis(2);
        if(stick.getRawAxis(4) < 0.03 && stick.getRawAxis(4) > -0.03) {
            RightCmd = 0;
        } else RightCmd = stick.getRawAxis(4);
        
        drive(LeftCmd, RightCmd);
        
        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {

    }

    public void drive(double leftspeed, double rightspeed){
        LeftMotor_1.set(leftspeed);
        LeftMotor_2.set(leftspeed);
        RightMotor_1.set(rightspeed);
        RightMotor_2.set(rightspeed);
    }

    /* public void putTeleopData() {
     SmartDashboard.putNumber("Left Motor 1", LeftMotor_1.get());
     SmartDashboard.putNumber("Left Motor 2", LeftMotor_2.get());
     SmartDashboard.putNumber("Right Motor 1", RightMotor_1.get());
     SmartDashboard.putNumber("Right Motor 2", RightMotor_2.get());
     SmartDashboard.putNumber("Left Encoder", LeftEncoder.getDistance());
     SmartDashboard.putNumber("Right Encoder", RightEncoder.getDistance());
     }
     */
    
}
