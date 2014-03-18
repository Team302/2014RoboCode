/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
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
    Talon CollectorMotor;
    Talon ShooterMotor;

    Solenoid Shifter;
    Solenoid ShooterHolder;
    DoubleSolenoid Jaws;
    DoubleSolenoid Rotator;

    Joystick stick;
    Joystick CoOpstick;

    Encoder LeftEncoder;
    Encoder RightEncoder;
    Encoder ShooterEncoder;

    DigitalInput AutonOverride;

    SmartDashboardData SDD;

    NetworkTable table;

    Compressor Compressor;

    double TargetDistanceL;
    double TargetDistanceR;
    double TargetRateL;
    double TargetRateR;
    double LeftCmd;
    double RightCmd;
    double Speed;
    double Turn;
    double PrevErrorL;
    double PrevErrorR;

    boolean RCMode;
    boolean PRCMode;
    boolean PrevButton7;
    boolean AutonSwitch = false;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        LeftMotor_1 = new Talon(PWM_LEFT_MOTOR_1);
        LeftMotor_2 = new Talon(PWM_LEFT_MOTOR_2);
        RightMotor_1 = new Talon(PWM_RIGHT_MOTOR_1);
        RightMotor_2 = new Talon(PWM_RIGHT_MOTOR_2);
        ShooterMotor = new Talon(PWM_SHOOTER_MOTOR);
        CollectorMotor = new Talon(PWM_COLLECTOR_MOTOR);

        Shifter = new Solenoid(SOLENOID_SHIFTERS);
        ShooterHolder = new Solenoid(SOLENOID_SHOOTER_HOLDER);
        Jaws = new DoubleSolenoid(SOLENOID_JAWS_CLOSE, SOLENOID_JAWS_OPEN);
        Rotator = new DoubleSolenoid(SOLENOID_ROTATOR_UP, SOLENOID_ROTATOR_DOWN);

        stick = new Joystick(1);
        CoOpstick = new Joystick(2);

        LeftEncoder = new Encoder(DIO_LEFT_ENCODER_ACHANNEL, DIO_LEFT_ENCODER_BCHANNEL);
        RightEncoder = new Encoder(DIO_RIGHT_ENCODER_ACHANNEL, DIO_RIGHT_ENCODER_BCHANNEL);
        ShooterEncoder = new Encoder(DIO_SHOOTER_ENCODER_ACHANNEL, DIO_SHOOTER_ENCODER_BCHANNEL);
        AutonOverride = new DigitalInput(DIO_AUTON_SWITCH);

        SDD = new SmartDashboardData();

        table = NetworkTable.getTable("datatable");

        Compressor = new Compressor(DIO_PRESSURE_SWITCH, RELAY_COMPRESSOR);
        Compressor.start();

        /*
         * One Encoder pulse is approximately 1/20 inches.
         */
        LeftEncoder.setDistancePerPulse(0.0514285714285714);
        RightEncoder.setDistancePerPulse(0.0514285714285714);
        ShooterEncoder.setDistancePerPulse(0.9);

        LeftEncoder.start();
        RightEncoder.start();
        ShooterEncoder.start();

        jawsRelax();
        shooterInit();
        driveInit();
    }

    /*
     * Enumerated Constants don't work with this version of Java (ME 1.4) so these
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
     * This function is called once before autonomous and should be used for any
     * initialization code.
     */
    public void autonomousInit() {
        LeftEncoder.reset();
        RightEncoder.reset();

        if (CoOpstick.getRawButton(AUTON_SHOOT_HIGH)) {
            TargetDistanceL = 180;
            TargetDistanceR = 180;
        } else {
            TargetDistanceL = 180;
            TargetDistanceR = 180; //180 is 15 ft
        }

        AutonMode = CLAMP_1;
        LeftCmd = 0;
        RightCmd = 0;
        jawsClose();
        rotateUp();

        PrevErrorL = 0;
        PrevErrorR = 0;
        TimerCount = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

        /*
         * Percent error to be used for Proportional rate control of the motors.
         */
        double PrateL = (TargetRateL - LeftEncoder.getRate()) / TargetRateL;
        double PrateR = (TargetRateR - RightEncoder.getRate()) / TargetRateR;
        /*
         * Difference in distance used to drive straight
         */
        double DeltaDistance = LeftEncoder.getDistance() - RightEncoder.getDistance();
        /*
         * Percent error to be used for Proportional distance control of the motors
         */
        double PdistanceL = ((TargetDistanceL - LeftEncoder.getDistance()) / Math.abs(TargetDistanceL));
        double PdistanceR = ((TargetDistanceR - RightEncoder.getDistance()) / Math.abs(TargetDistanceR));
        /*
         * Accumulated error to be used for Integral distance control of the motors
         */
        double IdistanceL = PdistanceL + PrevErrorL;
        double IdistanceR = PdistanceR + PrevErrorR;

        shooterPeriodic();

        /*
         *TODO: Fill the empty cases.
         */
        switch (AutonMode) {
            
            case CLAMP_1: {
                if (TimerCount < 50) {
                    jawsClose();
                    if (CoOpstick.getRawButton(AUTON_SHOOT_HIGH)) {
                        rotateUp();
                    } else {
                        rotateDown();
                    }
                    collectorMStop();
                    TimerCount++;
                } else {
                    TimerCount = 0;
                    AutonMode = FORWARD_1;
                }
                break;
            }
            
            //Drive forward
            case FORWARD_1: {
                if (LeftEncoder.getDistance() < TargetDistanceL && RightEncoder.getDistance() < TargetDistanceR) {

                    LeftCmd = 0.75 - 0.15 * DeltaDistance;
                    RightCmd = 0.75 + 0.15 * DeltaDistance;

                } else {
                    LeftCmd = 0;
                    RightCmd = 0;

                    if (CoOpstick.getRawButton(AUTON_SHOOT_HIGH)) {
                        jawsOpen();
                    } else {
                        jawsRelax();
                    }
                    TimerCount = 0;
                    AutonMode = SHOOT_1;
                }
                break;
            }
            
            //Shoot the first ball.
            case SHOOT_1: {
                if (TimerCount < 100) {
                    if (CoOpstick.getRawButton(AUTON_SHOOT_HIGH)) {
                        if (TimerCount == 25) {
                            shooterTrigger();
                        }
                    } else {
                        collectorMSpit();
                        LeftCmd = 0.15;
                        RightCmd = 0.15;
                    }
                    TimerCount++;
                } else {
                    TimerCount = 0;
                    collectorMStop();
                    LeftCmd = 0;
                    RightCmd = 0;

                    IdistanceL = PrevErrorL = 0;
                    IdistanceR = PrevErrorR = 0;

                    TargetDistanceL = -156;
                    TargetDistanceR = -156;

                    LeftEncoder.reset();
                    RightEncoder.reset();

                    AutonMode = BACKWARD_1;
                }
                break;
            }
            
            //Drive backward
            case BACKWARD_1: {
                if (LeftEncoder.getDistance() >= TargetDistanceL || RightEncoder.getDistance() >= TargetDistanceR) {
                    LeftCmd = 1 * PdistanceL + 0.001 * IdistanceL;
                    RightCmd = 1 * PdistanceR + 0.001 * IdistanceR;
                } else {
                    TimerCount = 0;

                    IdistanceL = PrevErrorL = 0;
                    IdistanceR = PrevErrorR = 0;

                    if (AutonSwitch) {
                        LeftEncoder.reset();
                        RightEncoder.reset();

                        AutonMode = TURN_RIGHT_90;
                    } else {
                        AutonMode = 0;
                    }
                }
                break;
            }
            
            //Turn right 90 degrees
            case TURN_RIGHT_90: {
                //Eldon
                break;
            }
            
            //Grab a ball using whatever mechanism collects balls
            case COLLECT_BALL: {

                break;
            }
            
            //Turn left after collecting the ball
            case TURN_LEFT_90: {

                break;
            }
            
            //Shoot with the second ball
            case SHOOT_2: {

                break;
            }
            
            //don't do anything
            default: {
                collectorMStop();
                LeftCmd = 0;
                RightCmd = 0;
                break;
            }
        }

        drive(LeftCmd, RightCmd);
        PrevErrorL = IdistanceL;
        PrevErrorR = IdistanceR;

        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder, stick, CoOpstick, RCMode);
    }

    /**
     * This function is called before operator control and should be used for
     * any initialization code.
     */
    public void teleopInit() {
        RCMode = true;

        rotateDown();

        LeftEncoder.reset();
        RightEncoder.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        shooterPeriodic();
        //Toggle between drive modes RC and Tank
        if (!PrevButton7 && stick.getRawButton(7)) {
            RCMode = !RCMode;
        }

        /*if (ShooterState != SHOOTER_STOP) {
            LeftCmd = 0.5 * (leftencodercount - LeftEncoder.getDistance());
            RightCmd = 0.5 * (rightencodercount - LeftEncoder.getDistance());
        } else {
            //RCMode uses left joystick for speed, right joystick for turn*/
            if (RCMode) {
                Speed = deadband(2, .03);
                Turn = deadband(4, .03);

                RightCmd = Speed + Turn;
                LeftCmd = Speed - Turn;
                //Tank Drive uses left joystick for left motor speed control
                //and right joystick for right motor speed control
            } else {
                LeftCmd = deadband(2, .03);
                RightCmd = deadband(5, .03);
            }
        //}
        //Shift
        if (stick.getRawButton(5)) {
            Shifter.set(false);
        } else if (stick.getRawButton(6)) {
            Shifter.set(true);
        }
        //Set the solenoid value for the Jaws, which is controlled by two
        //seperate solenoids
        if (CoOpstick.getRawButton(JAWS_CLOSE)) {
            jawsClose();
        } else if (CoOpstick.getRawButton(JAWS_OPEN)) {
            jawsOpen();
        } else if (CoOpstick.getRawButton(JAWS_RELAX)){
            jawsRelax();
        }
        CollectorMotor.set(-CoOpdeadband(2, 0.03));

        if (CoOpstick.getRawAxis(ROTATOR_AXIS) >= 0.5) {
            rotateDown();
        } else if(CoOpstick.getRawAxis(ROTATOR_AXIS) <= -0.5){
            rotateUp();
        }

        if (CoOpstick.getRawButton(SHOOTER_BUTTON)) {
            shooterTrigger();
        }

        drive(LeftCmd, RightCmd);

        PrevButton7 = stick.getRawButton(7);

        SDD.putSDData(LeftMotor_1, LeftMotor_2, RightMotor_1, RightMotor_2, LeftEncoder, RightEncoder, stick, CoOpstick, RCMode);
        SmartDashboardData.putNumber(Jaws.getSmartDashboardType(), Jaws.get().value);
        SmartDashboardData.putNumber("Shooter Motor", ShooterMotor.get());
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {

    }
    
    double leftoutput;
    double rightoutput;
    double accelerationlimit;
    
    public void driveInit() {
        leftoutput = 0;
        rightoutput = 0;
        accelerationlimit = 0.02;
    }
    /**
     * Set the output for both motors
     *
     * @param leftspeed double ranging from -1.0 to 1.0, adding anything higher
     * or lower will not cause errors and will automatically be adjusted. Left
     * Motor is inverted and the Talon can only be inverted directly in the
     * output.
     *
     * @param rightspeed double ranging from -1.0 to 1.0, adding anything higher
     * or lower will not cause errors and will automatically be adjusted.
     */
    public void drive(double leftspeed, double rightspeed) {
        //Check consistency w/ practice bot
        if(Rotator.get() == DoubleSolenoid.Value.kReverse) {
             if(leftspeed >= leftoutput) {
                 leftoutput = leftoutput + accelerationlimit;
             } else if (leftspeed <= leftoutput) {
                 leftoutput = leftoutput - accelerationlimit;
             }
             if(rightspeed >= rightoutput) {
                 rightoutput = rightoutput + accelerationlimit;
             } else if (rightspeed <= rightoutput) {
                 rightoutput = rightoutput - accelerationlimit;
             }
             LeftMotor_1.set(-leftoutput);
             LeftMotor_2.set(-leftoutput);
             RightMotor_1.set(rightoutput);
             RightMotor_2.set(rightoutput);
        } else {
            LeftMotor_1.set(-leftspeed);
            LeftMotor_2.set(-leftspeed);
            RightMotor_1.set(rightspeed);
            RightMotor_2.set(rightspeed);
        }
    }

    /**
     * Sets a dead band on a joystick axis to prevent constant power drain.
     *
     * @param axis The joystick axis to be used. The joystick is named "stick"
     * and is initialized in robotInit.
     *
     * @param deadband The dead band value. If the joystick is within the dead
     * band, the function returns 0.
     *
     * @return If the joystick is within the dead band, returns 0. Otherwise,
     * the negative of the axis will be returned because the joystick returns 
     * negative in the positive direction.
     */
    public double deadband(int axis, double deadband) {
        if (stick.getRawAxis(axis) > -deadband && stick.getRawAxis(axis) < deadband) {
            return 0;
        } else {
            return -stick.getRawAxis(axis);
        }
    }
    
    public double CoOpdeadband(int axis, double deadband) {
        if (CoOpstick.getRawAxis(axis) > -deadband && CoOpstick.getRawAxis(axis) < deadband) {
            return 0;
        } else {
            return -CoOpstick.getRawAxis(axis);
        }
    }

    /**
     * Sets the value for the rotator to forward.
     */
    public void rotateUp() {
        Rotator.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Sets the value for the rotator to reverse.
     */
    public void rotateDown() {
        Rotator.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the value for the jaws to reverse.
     */
    public void jawsClose() {
        Jaws.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Sets the value for the jaws to forward.
     */
    public void jawsOpen() {
        Jaws.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the value for the jaws to off.
     */
    public void jawsRelax() {
        Jaws.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Sets the collector motor to collect balls.
     */
    public void collectorMCollect() {
        CollectorMotor.set(1);
    }

    /**
     * Sets the collector motor to spit balls.
     */
    public void collectorMSpit() {
        CollectorMotor.set(-1);
    }

    /**
     * Sets the collector motor to 0.
     */
    public void collectorMStop() {
        CollectorMotor.set(0);
    }

    //
    //  control shooter
    //
// later, make this its own class
    double leftencodercount;
    double rightencodercount;
    int ShooterTimer;
    int ShooterState;
    final int SHOOTER_STOP = 0;
    final int SHOOTER_PREPARE = 1;
    final int SHOOTER_RETRACT = 2;
    final int SHOOTER_FIRE = 3;
    final int SHOOTER_RETURN = 4;
    final int SHOOTER_RELAX = 5;
    final int SHOOTER_MANUAL = 6;

    // encoder positions
    final double SHOOTER_BACKSTOP = -60;     // stop retracting when angle is less than this
    final double SHOOTER_SAFEANGLE = -10;     // move backward until the shooter greater than this
    final double SHOOTER_FORWARDSTOP = 70;      // stop forward motion here

    public void shooterInit() {
        ShooterState = SHOOTER_STOP;
    }

    // call this to start the shooter sequence
    public void shooterTrigger() {
        if (ShooterState == SHOOTER_STOP) {
            ShooterState = SHOOTER_PREPARE;
            ShooterTimer = 0;
            ShooterHolder.set(true);
            leftencodercount = LeftEncoder.getDistance();
            rightencodercount = RightEncoder.getDistance();
        }
        // else its already in motion
    }

    // this gets called every 20ms in both Autonomous and Teleop
    public void shooterPeriodic() {
        //  Timeout to stop it in case it gets stuck
        ShooterTimer++;
        if (ShooterTimer > 200) {   // give up after 4 sec.
            ShooterState = SHOOTER_STOP;
        }

        //  Take action depending on the state
        switch (ShooterState) {

            case SHOOTER_STOP:
                ShooterMotor.set(0);
                ShooterHolder.set(false);
                ShooterTimer = 0;
                break;

            case SHOOTER_PREPARE:
                if (CoOpstick.getRawButton(SHOOTER_OVERRIDE)) {
                    ShooterState = SHOOTER_MANUAL;
                } else {
                    ShooterState = SHOOTER_RETRACT;
                }
                break;

            case SHOOTER_RETRACT:
                if (ShooterEncoder.getDistance() > SHOOTER_BACKSTOP) {
                    ShooterMotor.set(-1);   // pull back
                } else {
                    ShooterMotor.set(0);    // stop for 20ms
                    ShooterState = SHOOTER_FIRE;
                }
                break;

            case SHOOTER_FIRE:
                if (ShooterEncoder.getDistance() < SHOOTER_FORWARDSTOP) {
                    ShooterMotor.set(1);    // swing to shoot
                } else {
                    ShooterMotor.set(0);    // stop
                    ShooterState = SHOOTER_RELAX;
                }
                break;

            case SHOOTER_MANUAL:
                if (ShooterTimer < 25) {
                    ShooterMotor.set(-.4);
                } else {
                    if (ShooterTimer < 75) {
                        ShooterMotor.set(1);
                    } else {
                        if (ShooterTimer < 100) {
                        ShooterMotor.set(-.4);
                        } else {
                            ShooterState = SHOOTER_STOP;
                        }
                    }
                }
                break;

            case SHOOTER_RELAX:
                if (ShooterEncoder.getDistance() < SHOOTER_SAFEANGLE) {
                    ShooterState = SHOOTER_STOP;
                } else {
                    ShooterMotor.set(-0.400);
                }
                break;

            default:
                ShooterMotor.set(0);
                ShooterState = SHOOTER_STOP;
                break;
        }
        SmartDashboardData.putNumber("Shooter Encoder", ShooterEncoder.getDistance());
    }
}
