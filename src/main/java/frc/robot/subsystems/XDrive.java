// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;

public class XDrive extends SubsystemBase {
    private SparkMax lf = new SparkMax(1, MotorType.kBrushless);
    private SparkMax rf = new SparkMax(2, MotorType.kBrushless);
    private SparkMax rb = new SparkMax(3, MotorType.kBrushless);
    private SparkMax lb = new SparkMax(4, MotorType.kBrushless);
    
    public XDrive() {}

    public void drive(double y, double x, double r) {
        double Vx = x;
        double Vy = y;
        double omega = -r;

        double FL = Vy + Vx + omega;
        double FR = Vy - Vx - omega;
        double BL = Vy - Vx + omega;
        double BR = Vy + Vx - omega;

        // Normalização
        double max = Math.max(Math.abs(FL), Math.max(Math.abs(FR), Math.max(Math.abs(BL), Math.abs(BR))));
        if (max > 1.0) {
            FL /= max;
            FR /= max;
            BL /= max;
            BR /= max;
        }

        // Aplicando aos motores
        lf.set(FL);
        rf.set(FR);
        lb.set(BL);
        rb.set(BR);
    }
}
