/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author John
 */
public class SmartDashboardData {

    
   public void putTeleopData(Victor LeftMotor1, Victor LeftMotor2, Victor RightMotor1, Victor RightMotor2, Encoder LeftEncoder, Encoder RightEncoder) {
        SmartDashboard.putNumber("Left Motor 1", LeftMotor1.get());
        SmartDashboard.putNumber("Left Motor 2", LeftMotor2.get());
        SmartDashboard.putNumber("Right Motor 1", RightMotor1.get());
        SmartDashboard.putNumber("Right Motor 2", RightMotor2.get());
        SmartDashboard.putNumber("Left Encoder", LeftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder", RightEncoder.getDistance());
        SmartDashboard.putNumber("Left Encoder Rate", LeftEncoder.getRate());
        SmartDashboard.putNumber("Right Encoder Rate", RightEncoder.getRate());
        SmartDashboard.putNumber("PID Thingy Left", LeftController.get());
   }
   
}
