/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class KitofPartsBot extends IterativeRobot implements RobotMap {

    /*
     * Only the cRIO can access the variables in this class. In order to 
     * interface with the robot through the Driver Station, use Network Tables.
     */
    Talon LeftMotor_1;
    Talon LeftMotor_2;
    Talon RightMotor_1;
    Talon RightMotor_2;
    Victor CollectorMotor;
    DoubleSolenoid Jaws;
    DoubleSolenoid Rotator;
    Joystick stick;
    Joystick CoOpstick;
    Encoder LeftEncoder;
    Encoder RightEncoder;
    SmartDashboardData SDD;
    NetworkTable table;
    Compressor Compressor;

    boolean IsTargetDistance;
    double TargetDistanceL;
    double TargetDistanceR;
    double TargetRateL;
    double TargetRateR;
    double LeftCmd;
    double RightCmd;
    double Speed;
    double Turn;
    boolean RCMode;
    boolean PRCMode;
    boolean Pbutton11;
    boolean AutonSwitch = true;
    /*
     * Enumerated Constants don't work with this version of Java (1.4) so these
     * serve as the enumerated constants for the Autonomous state machine.
     */

    int AutonMode;
    final int CLAMP_1 = 9;
    final int FORWARD_1 = 1;
    final int SHOOT_1 = 2;
    final int BACKWARD_1 = 3;
    final int TURN_RIGHT_90 = 4;
    final int COLLECT_BALL = 5;
    final int TURN_LEFT_90 = 6;
    final int FORWARD_2 = 7;
    final int SHOOT_2 = 8;

    int TimerCount;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        LeftMotor_1 = new Talon(PWM_LEFT_MOTOR_1);
        LeftMotor_2 = new Talon(PWM_LEFT_MOTOR_2);
        RightMotor_1 = new Talon(PWM_RIGHT_MOTOR_1);
        RightMotor_2 = new Talon(PWM_RIGHT_MOTOR_2);
        CollectorMotor = new Victor(PWM_COLLECTOR_MOTOR);
        Jaws = new DoubleSolenoid(SOLENOID_JAWS_CLOSE, SOLENOID_JAWS_OPEN);
        Rotator = new DoubleSolenoid(SOLENOID_ROTATOR_UP, SOLENOID_ROTATOR_DOWN);
        stick = new Joystick(1);
        CoOpstick = new Joystick(2);
        LeftEncoder = new Encoder(DIO_LEFT_ENCODER_ACHANNEL, DIO_LEFT_ENCODER_BCHANNEL);
        RightEncoder = new Encoder(DIO_RIGHT_ENCODER_ACHANNEL, DIO_RIGHT_ENCODER_BCHANNEL);
        SDD = new SmartDashboardData();
        table = NetworkTable.getTable("datatable");
        Compressor = new Compressor(DIO_PRESSURE_SWITCH, RELAY_COMPRESSOR);
        Compressor.start();
        jawsRelax();

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

        IsTargetDistance = false; //not used yet
        TargetRateL = TargetRateR = 47.46428571428571; //Approximately 30% motor power
        TargetDistanceL = TargetDistanceR = 12; //12 inches

        AutonMode = CLAMP_1;
        LeftCmd = 0;
        RightCmd = 0;
        Jaws.set(DoubleSolenoid.Value.kReverse);

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
        double DeltaDistance = LeftEncoder.getDistance() - RightEncoder.getDistance();

        double PdistanceL = ((TargetDistanceL - LeftEncoder.getDistance()) / TargetDistanceL);
        double PdistanceR = ((TargetDistanceR - RightEncoder.getDistance()) / TargetDistanceR);

        /*
         TODO: reduce to 5 cases
         */
        /*
         jaw close
         auton foward
         rotator lower
         timer
         jaw relax
         auton shoot
         */
        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder, stick, CoOpstick, RCMode);
    }

    /**
     * This function is called before operator control and should be used for
     * any initialization code.
     */
    public void teleopInit() {
        RCMode = true;
        rotateDown();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //Toggle between drive modes RC and Tank
        if (!Pbutton11 && stick.getRawButton(7)) {
            RCMode = !RCMode;
        }
        if (!RCMode) {
            LeftCmd = deadband(2, .03);
            RightCmd = deadband(5, .03);

            drive(LeftCmd, RightCmd);
        } else {
            Speed = deadband(2, .03);
            Turn = deadband(4, .03);

            RightCmd = Speed + Turn;
            LeftCmd = Speed - Turn;

            drive(LeftCmd, RightCmd);
        }
        //Set the solenoid value for the Jaws, which is controlled by two
        //seperate solenoids
        if (CoOpstick.getRawButton(JAWS_ClOSE)) {
            jawsClose();
        } else if (CoOpstick.getRawButton(JAWS_OPEN)) {
            jawsOpen();
        } else if (CoOpstick.getRawButton(JAWS_ClOSE) == false && CoOpstick.getRawButton(JAWS_OPEN) == false) {
            jawsRelax();
        } else {
            jawsClose();
        }
        if (CoOpstick.getRawButton(COLLECTOR_MOTOR_IN)) {
            collectorMCollect();
        } else if (CoOpstick.getRawButton(COLLECTOR_MOTOR_OUT)) {
            collectorMSpit();
        } else {
            collectorMStop();
        }

        if (CoOpstick.getRawButton(ROTATOR_SWITCH)) {
            rotateUp();
        } else {
            rotateDown();
        }

        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder, stick, CoOpstick, RCMode);
        SmartDashboardData.putNumber(Jaws.getSmartDashboardType(), Jaws.get().value);
        Pbutton11 = stick.getRawButton(7);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {

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
    public void drive(double leftspeed, double rightspeed) {
        LeftMotor_1.set(leftspeed);
        LeftMotor_2.set(leftspeed);
        RightMotor_1.set(-rightspeed);
        RightMotor_2.set(-rightspeed);
    }

    public double deadband(int axis, double deadband) {
        if (stick.getRawAxis(axis) > -deadband && stick.getRawAxis(axis) < deadband) {
            return 0;
        } else {
            return stick.getRawAxis(axis);
        }
    }

    public void rotateUp() {
        Rotator.set(DoubleSolenoid.Value.kForward);
    }

    public void rotateDown() {
        Rotator.set(DoubleSolenoid.Value.kReverse);
    }

    public void jawsClose() {
        Jaws.set(DoubleSolenoid.Value.kReverse);
    }

    public void jawsOpen() {
        Jaws.set(DoubleSolenoid.Value.kForward);
    }

    public void jawsRelax() {
        Jaws.set(DoubleSolenoid.Value.kOff);
    }

    public void collectorMCollect() {
        CollectorMotor.set(-1);
    }

    public void collectorMSpit() {
        CollectorMotor.set(1);
    }

    public void collectorMStop() {
        CollectorMotor.set(0);
    }
}
