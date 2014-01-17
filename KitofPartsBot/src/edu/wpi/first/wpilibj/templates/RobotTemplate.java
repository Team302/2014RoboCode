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
    SmartDashboardData SDD = new SmartDashboardData();
    PIDController AutonomousMobility;
    
    public void robotInit() {
       LeftMotor_1 = new Victor(1);
       LeftMotor_2 = new Victor(2);
       RightMotor_1 = new Victor(3);
       RightMotor_2 = new Victor(4);
       stick = new Joystick(1);
       LeftEncoder = new Encoder(10,11);
       RightEncoder = new Encoder(1,2);
       LeftEncoder.start();
       RightEncoder.start();
       LeftEncoder.setReverseDirection(true);
       RightEncoder.setReverseDirection(true);
       LeftEncoder.setDistancePerPulse(28);
       RightEncoder.setDistancePerPulse(28);
    }

    
    public void autonomousInit() {
        AutonomousMobility = new PIDController(1, 0, 0, LeftEncoder, LeftMotor_1);
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        AutonomousMobility.setSetpoint(120);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        LeftMotor_1.set(-stick.getRawAxis(2));
        LeftMotor_2.set(-stick.getRawAxis(2));
        RightMotor_1.set(stick.getRawAxis(2));
        RightMotor_2.set(stick.getRawAxis(2));
        SDD.putTeleopData(LeftMotor_1, LeftMotor_2, RightMotor_1,RightMotor_2, LeftEncoder, RightEncoder);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
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
