package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

//import edu.wpi.first.wpilibj.Encoder;

//import com.ctre.*;

//import com.ctre.phoenix6.hardware.CANcoder;

// import com.ctre.sensors.CANCoderConfiguration;

// import com.ctre.phoenix6.sensors.CANCoderFaults;
// import com.ctre.phoenix.sensors.CANCoderStickyFaults;
// import com.ctre.phoenix.sensors.MagnetFieldStrength;
// import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
    boolean flag = false;

    private static final int MOTOR1_CAN_ID = 2;
    private static final int MOTOR1_ROTATE_CAN_ID = 3;

    private static final int MOTOR2_CAN_ID = 4;
    private static final int MOTOR2_ROTATE_CAN_ID = 5;

    private static final int MOTOR3_CAN_ID = 7;
    private static final int MOTOR3_ROTATE_CAN_ID = 6;

    private static final int MOTOR4_CAN_ID = 8;
    private static final int MOTOR4_ROTATE_CAN_ID = 9;

    // Motor 1
    private CANSparkMax neoMotor1;
    private CANSparkMax neoMotor1Rotate;

    private RelativeEncoder neo1MotorEncoder;
    private SparkPIDController neo1MotorPID;

    // Motor 2
    private CANSparkMax neoMotor2;
    private CANSparkMax neoMotor2Rotate;

    private RelativeEncoder neo2MotorEncoder;
    private SparkPIDController neo2MotorPID;

    // Motor 3
    private CANSparkMax neoMotor3;
    private CANSparkMax neoMotor3Rotate;

    private RelativeEncoder neo3MotorEncoder;
    private SparkPIDController neo3MotorPID;

    // Motor 4
    private CANSparkMax neoMotor4;
    private CANSparkMax neoMotor4Rotate;

    private RelativeEncoder neo4MotorEncoder;
    private SparkPIDController neo4MotorPID;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    /* CanCoders: */
    // private static final String canBusName = "rio";

    // private final CANcoder cancoder1 = new CANcoder(10, canBusName);
    // private final CANcoder cancoder2 = new CANcoder(11, canBusName);
    // private final CANcoder cancoder3 = new CANcoder(12, canBusName);
    // private final CANcoder cancoder4 = new CANcoder(13, canBusName);

    // private CANEncoder encoder1;

    private static final double WHEEL_DIAMETER_METERS = 0.15; // Zmień na średnicę koła w metrach
    private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    XboxController xbox_stick = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver
                                                       // Station
    // private Joystick m_stick = new Joystick(0);

    private RelativeEncoder m_alternateEncoder;

    @Override
    public void robotInit() {

        neoMotor1 = new CANSparkMax(MOTOR1_CAN_ID, MotorType.kBrushless);
        neoMotor1Rotate = new CANSparkMax(MOTOR1_ROTATE_CAN_ID, MotorType.kBrushless);

        neoMotor2 = new CANSparkMax(MOTOR2_CAN_ID, MotorType.kBrushless);
        neoMotor2Rotate = new CANSparkMax(MOTOR2_ROTATE_CAN_ID, MotorType.kBrushless);

        neoMotor3 = new CANSparkMax(MOTOR3_CAN_ID, MotorType.kBrushless);
        neoMotor3Rotate = new CANSparkMax(MOTOR3_ROTATE_CAN_ID, MotorType.kBrushless);

        neoMotor4 = new CANSparkMax(MOTOR4_CAN_ID, MotorType.kBrushless);
        neoMotor4Rotate = new CANSparkMax(MOTOR4_ROTATE_CAN_ID, MotorType.kBrushless);

        

        neoMotor2Rotate.restoreFactoryDefaults();
        neo2MotorEncoder = neoMotor2Rotate.getEncoder();
        neo2MotorPID = neoMotor2Rotate.getPIDController();

        neoMotor3Rotate.restoreFactoryDefaults();
        neo3MotorEncoder = neoMotor3Rotate.getEncoder();
        neo3MotorPID = neoMotor3Rotate.getPIDController();

        neoMotor1Rotate.restoreFactoryDefaults();
        neo1MotorEncoder = neoMotor1Rotate.getEncoder();
        neo1MotorPID = neoMotor1Rotate.getPIDController();

        neoMotor4Rotate.restoreFactoryDefaults();
        neo4MotorEncoder = neoMotor4Rotate.getEncoder();
        neo4MotorPID = neoMotor4Rotate.getPIDController();
        
        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        neo2MotorPID.setP(kP);
        neo2MotorPID.setI(kI);
        neo2MotorPID.setD(kD);
        neo2MotorPID.setIZone(kIz);
        neo2MotorPID.setFF(kFF);
        neo2MotorPID.setOutputRange(kMinOutput, kMaxOutput);

        neo3MotorPID.setP(kP);
        neo3MotorPID.setI(kI);
        neo3MotorPID.setD(kD);
        neo3MotorPID.setIZone(kIz);
        neo3MotorPID.setFF(kFF);
        neo3MotorPID.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);


        // /* Configure CANcoder */
        // var toApply = new CANcoderConfiguration();

        // /*
        // * User can change the configs if they want, or leave it empty for
        // * factory-default
        // */
        // cancoder1.getConfigurator().apply(toApply);
        // cancoder2.getConfigurator().apply(toApply);
        // cancoder3.getConfigurator().apply(toApply);
        // cancoder4.getConfigurator().apply(toApply);

        /* Speed up signals to an appropriate rate */
        // cancoder1.getPosition().setUpdateFrequency(100);
        // cancoder1.getVelocity().setUpdateFrequency(100);

        // cancoder2.getPosition().setUpdateFrequency(100);
        // cancoder2.getVelocity().setUpdateFrequency(100);

        // cancoder3.getPosition().setUpdateFrequency(100);
        // cancoder3.getVelocity().setUpdateFrequency(100);

        // cancoder4.getPosition().setUpdateFrequency(100);
        // cancoder4.getVelocity().setUpdateFrequency(100);

        // m_alternateEncoder = neoMotor1.getAlternateEncoder(kAltEncType, kCPR);

        // encoder = neoMotor.getEncoder();
        // m_alternateEncoder = neoMotor1.getAlternateEncoder(kAltEncType, kCPR);

        // encoder.setPosition(0); // Resetowanie pozycji enkodera przy uruchomieniu
    }

    @Override
    public void robotPeriodic() {

        // var pos = cancoder1.getPosition();
        // System.out.println("Position is " + pos.toString() + " with " +
        // pos.getTimestamp().getLatency() + " seconds of latency");

        // var vel = cancoder1.getVelocity();
        // /* This time wait for the signal to reduce latency */
        // //vel.waitForUpdate(PRINT_PERIOD); // Wait up to our period

        // System.out.println("Velocity is " +
        // // vel.getValue() + " " +
        // // vel.getUnits() + " with " +
        // // vel.getTimestamp().getLatency() + " seconds of latency");

        // /* Angle */
        // System.out.println("Position can1: " + cancoder1.getPosition() + " cancoder2
        // posititon: " + cancoder2.getPosition() + "\n");
        // System.out.println("Position can3: " + cancoder3.getPosition() + " cancoder4
        // posititon: " + cancoder4.getPosition() + "\n");
        // System.out.println("Angle : " +
        // (Double.valueOf(cancoder3.getPosition().toString().substring(0,4) ) *
        // (360.0/4096.0) * 4000 ) + "\n\n");

        /* Angle */

        /*
         * System.out.println("Position can1: " + cancoder1.getAbsolutePosition() +
         * " cancoder2 posititon: " + cancoder2.getAbsolutePosition() + "\n");
         * System.out.println("Position can3: " + cancoder3.getAbsolutePosition() +
         * " cancoder4 posititon: " + cancoder4.getAbsolutePositi on() + "\n\n");
         */

        // System.out.println();

        // mechanism.update(cancoder.getPosition());
    }

    @Override
    public void teleopPeriodic() {

        double LeftX = xbox_stick.getLeftX();
        double RightX = xbox_stick.getRightX();
        double LeftY = xbox_stick.getLeftY();
        double RightY = xbox_stick.getRightY();

        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);



        if(xbox_stick.getAButtonPressed())
        {
            neoMotor1Rotate.getEncoder().setPosition(0);
            neoMotor2Rotate.getEncoder().setPosition(0);
            neoMotor3Rotate.getEncoder().setPosition(0);
            neoMotor4Rotate.getEncoder().setPosition(0);
        }


        // if PID coefficients on SmartDashboard have changed, write new values to controller
        
        if((p != kP)) { neo2MotorPID.setP(p); kP = p; }
        if((i != kI)) { neo2MotorPID.setI(i); kI = i; }
        if((d != kD)) { neo2MotorPID.setD(d); kD = d; }
        if((iz != kIz)) { neo2MotorPID.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { neo2MotorPID.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            neo2MotorPID.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        neo2MotorPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", neo2MotorEncoder.getPosition());
        
        System.out.println("Encoder pos: " + neo2MotorEncoder.getPosition() + "\n");

        
        if((p != kP)) { neo3MotorPID.setP(p); kP = p; }
        if((i != kI)) { neo3MotorPID.setI(i); kI = i; }
        if((d != kD)) { neo3MotorPID.setD(d); kD = d; }
        if((iz != kIz)) { neo3MotorPID.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { neo3MotorPID.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            neo3MotorPID.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        neo3MotorPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", neo3MotorEncoder.getPosition());
        
        System.out.println("Encoder pos: " + neo3MotorEncoder.getPosition() + "\n");
        
        

        // System.out.println("RightX: " + RightX);
        // System.out.println("RightY: " + RightY);
        // System.out.println("LeftY: " + LeftY);

        /*
         * neoMotor.set(-0.2); // Uruchomienie silnika z prędkością 10% !! nie używać
         * 100%
         * neoMotor1.set(-0.2);
         * 
         * double rotations = encoder.getPosition(); // Odczytanie liczby obrotów
         * double distanceMeters = rotations * WHEEL_CIRCUMFERENCE_METERS; // Obliczenie
         * dystansu
         * 
         * double rotations1 = encoder1.getPosition();
         * double distanceMeters1 = rotations1 * WHEEL_CIRCUMFERENCE_METERS; //
         * Obliczenie dystansu
         * 
         * // System.out.println("Dystans: " + distanceMeters + " metrow");
         */



        if (LeftY <= -0.3)
        {
            neoMotor1.set(0.2);
            neoMotor4.set(-0.2);

            neoMotor2.set(0.2);
            neoMotor3.set(-0.2);
        }
        if (LeftY >= 0.3)
        {

            neoMotor1.set(-0.2);
            neoMotor4.set(0.2);

            neoMotor2.set(-0.2);
            neoMotor3.set(0.2);

        }
        if (LeftY <= 0.3 && LeftY >= -0.3)
        {
            neoMotor1.set(0);
            neoMotor4.set(0);

            neoMotor2.set(0.0);
            neoMotor3.set(0.0);
        }

        if (RightX >= 0.3)
        {
            neoMotor1Rotate.set(-0.2);
            neoMotor4Rotate.set(-0.2);
        }
        if (RightX <= -0.3)
        {
        
            neoMotor1Rotate.set(0.2);
            neoMotor4Rotate.set(0.2);
        }
        if (RightX < 0.3 && RightX > -0.3)
        {
             neoMotor1Rotate.set(0);
             neoMotor4Rotate.set(0);
        }

        

        // if (xbox_stick.getBButtonPressed()) {
        // cancoder1.setPosition(0);
        // cancoder2.setPosition(0);
        // cancoder3.setPosition(0);
        // cancoder4.setPosition(0);

        // }
        // if (xbox_stick.getAButtonPressed()) {
        // // how much from current to 9
        // var rots = 9.0 -
        // Double.valueOf(cancoder2.getAbsolutePosition().toString().substring(0, 4));
        // double power = 0.1 * rots / Math.abs(9.0);

        // // neoMotor1.set(power);
        // }

    }

    @Override
    public void disabledPeriodic() { }

}