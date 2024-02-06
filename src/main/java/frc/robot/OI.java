// package frc.robot;


// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.XboxController;

// public class Robot extends TimedRobot {
//     private static final int PWM_PORT = 9; // Zmień na numer portu PWM, do którego podłączony jest Spark MAX
//     private static final int CONTROLLER_PORT = 0;

//     private PWMSparkMax neoMotor;
//     private XboxController xboxController;

//     @Override
//     public void robotInit() {
//         neoMotor = new PWMSparkMax(PWM_PORT);
//         xboxController = new XboxController(CONTROLLER_PORT);
//     }

//     @Override
//     public void teleopPeriodic() {
//       double speed = xboxController.getLeftY(); // Odczytuje pozycję lewej gałki joysticka na osi Y
//       neoMotor.set(speed);
//     }

// }