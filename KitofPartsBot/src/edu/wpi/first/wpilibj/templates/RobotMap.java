/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author John
 */
public interface RobotMap {
    
    static final int PWM_LEFT_MOTOR_1 = 1;
    static final int PWM_LEFT_MOTOR_2 = 2;
    static final int PWM_RIGHT_MOTOR_1 = 3;
    static final int PWM_RIGHT_MOTOR_2 = 4;
    static final int PWM_COLLECTOR_MOTOR = 5;
    static final int PWM_SHOOTER_MOTOR = 6;
    
    static final int DIO_AUTON_SWITCH = 1;
    static final int DIO_PRESSURE_SWITCH = 6;
    static final int DIO_SHOOTER_ENCODER_ACHANNEL = 7;
    static final int DIO_SHOOTER_ENCODER_BCHANNEL = 8;
    static final int DIO_LEFT_ENCODER_ACHANNEL = 10;
    static final int DIO_LEFT_ENCODER_BCHANNEL = 11;
    static final int DIO_RIGHT_ENCODER_ACHANNEL = 13;
    static final int DIO_RIGHT_ENCODER_BCHANNEL = 14;
    
    static final int RELAY_COMPRESSOR = 2;
    
    static final int SOLENOID_ROTATOR_UP = 2;
    static final int SOLENOID_ROTATOR_DOWN= 7;
    static final int SOLENOID_JAWS_CLOSE = 3;
    static final int SOLENOID_JAWS_OPEN = 5;
    static final int SOLENOID_SHIFTERS = 6;
    static final int SOLENOID_SHOOTER_HOLDER = 1;
    
    
    //Buttons/Switches Definition
    
    static final int JAWS_OPEN = 4;
    static final int JAWS_ClOSE = 3;
    
    static final int COLLECTOR_MOTOR_IN = 1;
    static final int COLLECTOR_MOTOR_OUT = 2;
    
    static final int ROTATOR_SWITCH = 6;
    
    static final int SHOOTER_BUTTON = 5;
    static final int SHOOTER_OVERRIDE = 7;
    
    static final int AUTON_SHOOT_HIGH = 8;
}
