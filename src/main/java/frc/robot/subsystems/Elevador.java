package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.google.flatbuffers.Struct;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevadorConstants;

public class Elevador extends SubsystemBase {
    // Elevator constants
    private static final int LEAD_ID = 5;
    private static final int FOLLOWER_ID = 6;
    private static final double REDUCTION = 5.14;
    private static final double CARRIAGE_MASS = 12.54956;
    private static final double DRUM_RADIUS = 0.01429;
    private static final double MIN_HEIGHT = 5;
    private static final double MAX_HEIGHT = 50;
    private static final double FIRST_STATE = 0.93980;
    private static final double SECOND_STATE = 1.77765;
    private static final double THIRD_STATE = 1.97174;

    private double maxspeed = 0.30;

    public boolean ePID = true;

    // Motors controllers
    private SparkMax elevatorMaster = new SparkMax(LEAD_ID, MotorType.kBrushless);
    private SparkMax elevatorSlave = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder masterEncoder;
    private RelativeEncoder slaveEncoder;
    double speed = 0;

    double target = 0;
    // Controllers (temporary constants)
    public PIDController pidControllerElevador = new PIDController(0.05, 0, 0);
    // Canva mechanism
    // private Mechanism2d elevatorMech = new Mechanism2d(3, 3);
    // private MechanismRoot2d elevatorRoot = elevatorMech.getRoot("Elevator root", 1.5, 0);
    // private MechanismLigament2d elevator = new MechanismLigament2d("Elevator", 1, 90);

    // Simulation classes
    // private ElevatorSim elevatorSim = new ElevatorSim(
    //     DCMotor.getCIM(2),
    //     REDUCTION,
    //     CARRIAGE_MASS,
    //     DRUM_RADIUS,
    //     MIN_HEIGHT,
    //     MAX_HEIGHT,
    //     true,
    //     MIN_HEIGHT);

    // States positions
    // private Pose3d state1Position;
    // private Pose3d state2Position;
    // private Pose3d state3Position;

    // Publishers states positions
    // private StructArrayPublisher<Pose3d> state1Publisher = NetworkTableInstance.getDefault().getStructArrayTopic("State 1", Pose3d.struct).publish();
    // private StructArrayPublisher<Pose3d> state2Publisher = NetworkTableInstance.getDefault().getStructArrayTopic("State 2", Pose3d.struct).publish();
    // private StructArrayPublisher<Pose3d> state3Publisher = NetworkTableInstance.getDefault().getStructArrayTopic("State 3", Pose3d.struct).publish();

    // Temp publishers
    // private StructArrayPublisher<Pose3d> tempPose01 = NetworkTableInstance.getDefault().getStructArrayTopic("Temp 01", Pose3d.struct).publish();
    // private StructArrayPublisher<Pose3d> tempPose02 = NetworkTableInstance.getDefault().getStructArrayTopic("Temp 02", Pose3d.struct).publish();

    public Elevador() {

        SmartDashboard.putString("Rodou?", "Não");
        SmartDashboard.putNumber("Target", -1);
        SmartDashboard.putNumber("Velocidade Máxima Elevador", maxspeed);
        pidControllerElevador.setTolerance(0.5);

        SmartDashboard.putData("reset elevator encoders", new InstantCommand(() -> {masterEncoder.setPosition(0); slaveEncoder.setPosition(0);}));
        // Configure controllers
        var motorMasterConfig = new SparkMaxConfig(); // C
        // motorMasterConfig.smartCurrentLimit(80); // Set max current in Ampers
        motorMasterConfig.idleMode(IdleMode.kBrake); // Set idle mode
        motorMasterConfig.inverted(false);
        motorMasterConfig.disableFollowerMode();
        elevatorMaster.configure(motorMasterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters); // Apply the configuration to the controller

        var motorSlaveConfig = new SparkMaxConfig();
        // motorSlaveConfig.smartCurrentLimit(80);
        motorSlaveConfig.idleMode(IdleMode.kBrake);
        motorSlaveConfig.inverted(false);
        motorSlaveConfig.disableFollowerMode();
        elevatorSlave.configure(motorSlaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Configure encoders
        masterEncoder = elevatorMaster.getEncoder();
        slaveEncoder = elevatorSlave.getEncoder();

        // Canva configuration
        // elevatorRoot.append(elevator); // Append the elevator on the root
        // SmartDashboard.putData("Elevator Mech", elevatorMech); // Put the canva of the elevator on the Dashboard
    };

    double lerp(double a, double b, double f)  {
        return a + f * (b - a);
    }

    @Override
    public void periodic() {
        // if(ePID)
        maxspeed = SmartDashboard.getNumber("Velocidade Máxima Elevador", 0.30);
        elevatorPIDMove(target);
    // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Height", getHeight());
        // SmartDashboard.putNumber("left elevator", ticksToMeters(masterEncoder.getPosition()));
        // SmartDashboard.putNumber("right elevator", ticksToMeters(slaveEncoder.getPosition()));
        // SmartDashboard.putNumber("left elevator motor", elevatorMaster.get());
        // SmartDashboard.putNumber("right elevator motor", elevatorSlave.get());
        SmartDashboard.putBoolean("in the setpoint", pidControllerElevador.atSetpoint());
        SmartDashboard.putNumber("elevator setpoint", pidControllerElevador.getSetpoint());
        SmartDashboard.putNumber("LeftPower", elevatorMaster.get());
        SmartDashboard.putNumber("RightPower", elevatorSlave.get());
    }

    /*
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
        elevatorSim.setInput(elevatorMaster.get() * RobotController.getInputVoltage()); // Set the inputs of the simulation object
        elevatorSim.update(0.02); // Update the simulation within 20ms

        elevator.setLength(elevatorSim.getPositionMeters()); // Set the length of the elevator on canva

        // Set the new states positions
        state1Position = new Pose3d(new Translation3d(0, 0, FIRST_STATE), new Rotation3d());
        state2Position = new Pose3d(new Translation3d(0, 0, MathUtil.clamp(getHeight(), FIRST_STATE, SECOND_STATE)), new Rotation3d());
        state3Position = new Pose3d(new Translation3d(0, 0, MathUtil.clamp(getHeight(), FIRST_STATE, THIRD_STATE)), new Rotation3d());
        // Update the states position on the NetworkTable
        state1Publisher.set(new Pose3d[]{ state1Position });
        state2Publisher.set(new Pose3d[]{ state2Position });
        state3Publisher.set(new Pose3d[]{ state3Position });

        // Temp pose
        Pose3d pivotPose = new Pose3d(new Translation3d(-0.33, 0, 0.81875), new Rotation3d(0, Units.degreesToRadians(0), 0));
        Pose3d basePivot = new Pose3d();
        tempPose01.set(new Pose3d[]{ basePivot });
        tempPose02.set(new Pose3d[]{ pivotPose });
    }*/



    // Raise elevator method
    public void levantagem(double controlePos) {
        speed = lerp(speed, -controlePos, 0.1);
        elevatorMaster.set(-speed);
        elevatorSlave.set(speed);
    }

    // set elevator position with PID
    // target = 0 minimum height
    // target = 1 max height
    public void elevatorPIDMove(double target) {
        // SmartDashboard.putString("h,jjgh,hkvvhk.vk,jh?", "Rodou");
        SmartDashboard.putNumber("Target", target);

        pidControllerElevador.setSetpoint(target);
        double speed = MathUtil.clamp(pidControllerElevador.calculate(getHeight()), -maxspeed, maxspeed);

        elevatorMaster.set(speed);
        elevatorSlave.set(-speed);
    }

    public void setTarget(double target){
        this.target = target;
    }

    // Get the height of the elevator
    public double getHeight() {
        // if(RobotBase.isSimulation())
        //     return elevatorSim.getPositionMeters();
        return (ticksToMeters(masterEncoder.getPosition()) + -ticksToMeters(slaveEncoder.getPosition()))/2;
    }

    public boolean inPosition() {
        return pidControllerElevador.atSetpoint();
    }

    // convert encoder ticks to meters
    public double ticksToMeters(double ticks) {
        return ticks * 0.7950125000000001 * 5.14;
    }
}

