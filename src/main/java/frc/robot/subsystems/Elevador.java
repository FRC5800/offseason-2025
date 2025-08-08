package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevador extends SubsystemBase {

    private static final int LEAD_ID = 1;
    private static final int FOLLOWER_ID = 2;

    WPI_TalonSRX elevadorMotorLead = new WPI_TalonSRX(LEAD_ID);
    WPI_TalonSRX elevadorMotorFollower = new WPI_TalonSRX(FOLLOWER_ID);

    //pid ta sem uso ainda mas ja fiz variaveis de posição inicial e final, as constantes eu peguei do codigo do climber da ultima seazon
    public PIDController pidControllerElevador = new PIDController(0.015, 0.01, 0.005);

    public Elevador() {
        elevadorMotorFollower.follow(elevadorMotorLead);
    };

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void levantagem(double controlePos) {
        elevadorMotorLead.set(controlePos * 0.5);
    }
}

