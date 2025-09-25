// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PS4Controller;

public class XDrive extends SubsystemBase {
    private final SparkMax fl, fr, bl, br;
     private PS4Controller PS4Controller;
    
        public XDrive(SparkMax fl, SparkMax fr, SparkMax bl, SparkMax br, PS4Controller joystick) {
            this.fl = fl; this.fr = fr; this.bl = bl; this.br = br;
            this.PS4Controller = joystick;
    }

    public void drive() {
        double Vx = -PS4Controller.getLeftX();
        double Vy = -PS4Controller.getLeftY();
        double omega = -PS4Controller.get?

        double FL = Vy + Vx + omega;
        double FR = Vy - Vx - omega;
        double BL = Vy - Vx + omega;
        double BR = Vy + Vx - omega;

        // Normalização
        double max = Math.max(Math.abs(FL), Math.max(Math.abs(FR), Math.max(Math.abs(BL), Math.abs(BR))));
        if (max > 1.0) {
            FL /= max; FR /= max; BL /= max; BR /= max;
        }

        // Aplicando aos motores
        fl.set(FL);
        fr.set(FR);
        bl.set(BL);
        br.set(BR);
    }
}
