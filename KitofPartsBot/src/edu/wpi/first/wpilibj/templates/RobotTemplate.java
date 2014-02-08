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
import edu.wpi.first.wpilibj.Victor;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot implements RobotMap {

    /*
     * Only the cRIO can access the variables in this class. In order to 
     * interface with the robot through the Driver Station, use Network Tables.
     */    
    
    Victor LeftMotor_1;
    Victor LeftMotor_2;
    Victor RightMotor_1;
    Victor RightMotor_2;
    Joystick stick;
    Encoder LeftEncoder;
    Encoder RightEncoder;
    SmartDashboardData SDD;
    
    double TargetDistanceL;
    double TargetDistanceR;
    double TargetRateL;
    double TargetRateR;
    double LeftCmd;
    double RightCmd;
    double TemporaryTargetL;
    double TemporaryTargetR;
    
    /*
     * Enumerated Constants don't work with this version of Java (1.4) so these
     * serve as the enumerated constants for the Autonomous state machine.
     */
    
    int AutonMode;
    final int FORWARD_1 = 1;
    final int SHOOT_1 = 2;
    final int BACKWARD_1 = 3;
    final int TURN_RIGHT_90 = 4;
    final int COLLECT_BALL = 5;
    final int TURN_LEFT_90 = 6;
    final int FORWARD_2 = 7;
    final int SHOOT_2 = 8;
    
    boolean initloop;
    
    int TimerCount;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    public void robotInit() {
        LeftMotor_1 = new Victor(PWM_LEFT_MOTOR_1);
        LeftMotor_2 = new Victor(PWM_LEFT_MOTOR_2);
        RightMotor_1 = new Victor(PWM_RIGHT_MOTOR_1);
        RightMotor_2 = new Victor(PWM_RIGHT_MOTOR_2);
        stick = new Joystick(1);
        LeftEncoder = new Encoder(DIO_LEFT_ENCODER_ACHANNEL, DIO_LEFT_ENCODER_BCHANNEL);
        RightEncoder = new Encoder(DIO_RIGHT_ENCODER_ACHANNEL, DIO_RIGHT_ENCODER_BCHANNEL);
        SDD = new SmartDashboardData();
        
        /*
         * One Encoder pulse is approximately 1/28 inches.
         */
        
        LeftEncoder.setDistancePerPulse(0.0357142857142857);
        RightEncoder.setDistancePerPulse(0.0357142857142857);
        LeftEncoder.start();
        RightEncoder.start();
    }

    /**
     * This function is called once before autonomous and should be used for any
     * initialization code.
     */
    public void autonomousInit() {
        LeftEncoder.reset();
        RightEncoder.reset();
        
        TargetRateL = TargetRateR = 47.46428571428571; //Approximately 30% motor power
        TargetDistanceL = TargetDistanceR = 12; //12 inches
        
        AutonMode = FORWARD_1;
        LeftCmd = 0;
        RightCmd = 0;
        
        TimerCount = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        
        /*
         * Percent error to be used for Proportional control of the motors.
         */
        
        double PrateL = (TargetRateL - LeftEncoder.getRate()) / TargetRateL;
        double PrateR = (TargetRateR - RightEncoder.getRate()) / TargetRateR;
        double DeltaDistance = LeftEncoder.getDistance()- RightEncoder.getDistance();
        
        double PdistanceL = ((TargetDistanceL - LeftEncoder.getDistance()) / TargetDistanceL);
        double PdistanceR = ((TargetDistanceR - RightEncoder.getDistance()) / TargetDistanceR);
        
        /*
        TODO: reduce to 5 cases
        */
        switch(AutonMode){
            //Drive forward
            case FORWARD_1: {
            if (LeftEncoder.getDistance() < 12 || RightEncoder.getDistance() < 12) {
                
                LeftCmd = (0.15 * PrateL + 0.3);
                RightCmd = (0.15 * PrateR + 0.3);
            
            } else {
                LeftCmd = 0;
                RightCmd = 0;
                
                LeftEncoder.reset();
                RightEncoder.reset();
                
                TimerCount = 0;
                AutonMode = SHOOT_1;
                }
            break;
            } 
            //Shoot the first ball (no shooter, waits for 50 loops)
            case SHOOT_1: {
                if (TimerCount < 50){
                    TimerCount++;
                } else {
                    TimerCount = 0;
                    LeftEncoder.reset();
                    RightEncoder.reset();
                    
                    TargetRateL = -47.46428571428571;
                    TargetRateR = -47.46428571428571;
                    
                    AutonMode = BACKWARD_1;
                }
                break;
            } 
            //Drive backward
            case BACKWARD_1: {
                if (LeftEncoder.getDistance() > -12 || RightEncoder.getDistance() > -12){
                    LeftCmd = -(0.15 * PrateL + 0.3);
                    RightCmd = -(0.15 * PrateR + 0.3);
                } else {
                    TimerCount = 0;
                    AutonMode = TURN_RIGHT_90;
                    
                    LeftEncoder.reset();
                    RightEncoder.reset();
                    
                    TargetDistanceL = 16.10066235;
                    TargetDistanceR = -16.10066235;
                    //LoL is fun
                    //^Message from Kyle
                }
                break;
            } 
            //Turn right 90 degrees
            case TURN_RIGHT_90: {
                boolean IsTargetDistanceL = false;
                boolean IsTargetDistanceR = false;
                
                if(Math.abs(TargetDistanceL - LeftEncoder.getDistance()) > 0.1){
                    LeftCmd = (0.3 * PdistanceL) + 0.15;
                } else {
                    IsTargetDistanceL = true;
                    LeftCmd = 0;
                }
                
                if(Math.abs(TargetDistanceR - RightEncoder.getDistance()) > 0.1){
                    RightCmd = (0.3 * PdistanceR) - 0.15;
                } else {
                    IsTargetDistanceR = true;
                    RightCmd = 0;
                }
                
                if (IsTargetDistanceL && IsTargetDistanceR){
                    TimerCount = 0;
                    LeftEncoder.reset();
                    RightEncoder.reset();
                    AutonMode = COLLECT_BALL;
                }
                break;
            } 
            //Grab a ball using whatever mechanism collects balls (no mechanism, so wait for 50 loops)
            case COLLECT_BALL: {
                if (TimerCount < 50){
                    TimerCount++;
                } else {
                    TimerCount = 0;
                    LeftEncoder.reset();
                    RightEncoder.reset();
                    
                    TargetDistanceL = -16.10066235;
                    TargetDistanceR = 16.10066235;
                    
                    TargetRateL = -47.46428571428571;
                    TargetRateR = 47.46428571428571;
                    
                    AutonMode = TURN_LEFT_90;
                }
                break;
            } 
            //Turn left after collecting the ball
            case TURN_LEFT_90: {
                
                boolean IsTargetDistanceL = false;
                boolean IsTargetDistanceR = false;
                
                if(Math.abs(TargetDistanceL - LeftEncoder.getDistance()) > 0.1){
                    LeftCmd = (0.3 * PdistanceL) - 0.15;
                } else {
                    IsTargetDistanceL = true;
                    LeftCmd = 0;
                }
                
                if(Math.abs(TargetDistanceR - RightEncoder.getDistance()) > 0.1){
                    RightCmd = (0.3 * PdistanceR) + 0.15;
                } else {
                    IsTargetDistanceR = true;
                    RightCmd = 0;
                }
                
                if (IsTargetDistanceL && IsTargetDistanceR){
                    TimerCount = 0;
                    
                    LeftCmd = 0;
                    RightCmd = 0;
                    
                    LeftEncoder.reset();
                    RightEncoder.reset();
                    
                    TargetRateL = 47.46428571428571;
                    TargetRateR = 47.46428571428571;
                    
                    AutonMode = FORWARD_2;
                } break;
            }
            case FORWARD_2: {
                if (LeftEncoder.getDistance() < 12 || RightEncoder.getDistance() < 12) {
                
                LeftCmd = (0.15 * PrateL + 0.3);
                RightCmd = (0.15 * PrateR + 0.3);
            
            } else {
                LeftCmd = 0;
                RightCmd = 0;
                
                LeftEncoder.reset();
                RightEncoder.reset();
                
                TimerCount = 0;
                AutonMode = SHOOT_2;
                } break;
            } 
            //Shoot with the second ball (no shooter, so wait for 50 loops)
            case SHOOT_2: {
                if (TimerCount < 50){
                    TimerCount++;
                } else {
                    TimerCount = 0;
                    LeftEncoder.reset();
                    RightEncoder.reset();
                    LeftCmd = 0;
                    RightCmd = 0;
                }
                break;
            }
            //don't do anything--should never be called
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
    
    // Hello John, it is I Tucker, I've come to say hello.
    public void teleopPeriodic() {
        if(stick.getRawAxis(2) < 0.03 && stick.getRawAxis(2) > -0.03){
            LeftCmd = 0;
        } else LeftCmd = -stick.getRawAxis(2);
        if(stick.getRawAxis(4) < 0.03 && stick.getRawAxis(4) > -0.03) {
            RightCmd = 0;
        } else RightCmd = -stick.getRawAxis(4);
        
        drive(LeftCmd, RightCmd);
        
        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder);
    }
    
    public void testInit() {
        initloop = true;
    }
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        driveDistance(12, 12, initloop);
        initloop = false;
        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder);
    }
    
    /**
     * Set the output for both motors
     * 
     * @param leftspeed double ranging from -1.0 to 1.0, adding anything higher 
     * or lower will not cause errors and will automatically be adjusted
     * 
     * @param rightspeed double ranging from -1.0 to 1.0, adding anything higher
     * or lower will not cause errors and will automatically be adjusted. Right 
     * Motor is inverted and the Victor can only be inverted directly in the 
     * output.
     */
    
    public void drive(double leftspeed, double rightspeed){
        LeftMotor_1.set(leftspeed);
        LeftMotor_2.set(leftspeed);
        RightMotor_1.set(-rightspeed);
        RightMotor_2.set(-rightspeed);
    }
    
    //TODO: test this maybe
    
    public boolean driveDistance(double leftdistance, double rightdistance, boolean initloop){
        boolean IsTargetDistance;
        double LeftError = TemporaryTargetL - LeftEncoder.getDistance();
        double RightError = TemporaryTargetR - RightEncoder.getDistance();
        
        if(initloop) {
            TemporaryTargetL = 0.96;
            TemporaryTargetR = 0.96;
            LeftEncoder.reset();
            RightEncoder.reset();
            IsTargetDistance = false;
        } else if(Math.abs(LeftError) < 1 && Math.abs(RightError) < 1){
            LeftCmd = 0;
            RightCmd = 0;
            IsTargetDistance = true;
        } else {
            LeftCmd = 0.03 * LeftError + 0.25 * Math.abs(LeftError) / LeftError;
            RightCmd = 0.03 * RightError + 0.25 * Math.abs(RightError) / RightError;
            
            if(Math.abs(TemporaryTargetL + 0.96 * Math.abs(leftdistance) / leftdistance) > Math.abs(leftdistance)) {
                TemporaryTargetL = leftdistance;
            } else TemporaryTargetL = TemporaryTargetL + 0.96 * Math.abs(leftdistance) / leftdistance;
            if(Math.abs(TemporaryTargetR + 0.96 * Math.abs(rightdistance) / rightdistance) > Math.abs(rightdistance)) {
                TemporaryTargetR = rightdistance;
            } else TemporaryTargetR = TemporaryTargetR + 0.96 * Math.abs(rightdistance) / rightdistance;
            IsTargetDistance = false;
        }
        SmartDashboardData.putNumber("Left Error", LeftError);
        SmartDashboardData.putNumber("Right Error", RightError);
        SmartDashboardData.putNumber("Temporary Target Left", TemporaryTargetL);
        SmartDashboardData.putNumber("Temporary Target Right", TemporaryTargetR);
        return IsTargetDistance;
    }
}
