

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 * @author John
 */
public class Shooter {
   /**
    * 
    * @param jaws The jaws double solenoid that will be used in the shooter.
    * @param shooterholder The piston solenoid that will hold the shooter.
    * @param shootermotor The Talon motor controller that will drive the kicker.
    * @param shooterencoder The encoder from which the positions will be acquired.
    * @param coopstick The Joystick that can be used to shoot and to overrider the encoder control.
    * @param shooteroverride The integer value of the button that overrides the encoder control.
    */ 
   public Shooter(DoubleSolenoid jaws, Solenoid shooterholder, Talon shootermotor, Encoder shooterencoder, Joystick coopstick, int shooteroverride) {
       Jaws = jaws;
       ShooterHolder = shooterholder;
       ShooterMotor = shootermotor;
       ShooterEncoder = shooterencoder;
       CoOpstick = coopstick;
       SHOOTER_OVERRIDE = shooteroverride;
   }
   /**
    * This method initializes the shooter and should be called before any control code.
    */
   public void init() {
        ShooterState = SHOOTER_STOP;
   }

    /**
     * This method triggers the shooter.
     */
    public void trigger() {
        if (ShooterState == SHOOTER_STOP && Jaws.get() == DoubleSolenoid.Value.kForward) {
            ShooterState = SHOOTER_PREPARE;
            ShooterTimer = 0;
            ShooterHolder.set(true);
        }
        // else its already in motion
    }

    /**
     * This method handles the shooter logic and should be called in both Autonomous and Teleop.
     */
    public void periodic() {
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
    
    private double leftencodercount;
    private double rightencodercount;
    private int ShooterTimer;
    private int ShooterState;
    private final int SHOOTER_STOP = 0;
    private final int SHOOTER_PREPARE = 1;
    private final int SHOOTER_RETRACT = 2;
    private final int SHOOTER_FIRE = 3;
    private final int SHOOTER_RETURN = 4;
    private final int SHOOTER_RELAX = 5;
    private final int SHOOTER_MANUAL = 6;

    // encoder positions
    private final double SHOOTER_BACKSTOP = -60;     // stop retracting when angle is less than this
    private final double SHOOTER_SAFEANGLE = -10;     // move backward until the angle is less than this
    private final double SHOOTER_FORWARDSTOP = 70;      // stop forward motion here
    
    private final DoubleSolenoid Jaws;
    private final Solenoid ShooterHolder;
    private final Talon ShooterMotor;
    private final Encoder ShooterEncoder;
    private final Joystick CoOpstick;
    private final int SHOOTER_OVERRIDE;
}
