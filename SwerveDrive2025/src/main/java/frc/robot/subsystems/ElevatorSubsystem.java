package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.hardware.*;

public class ElevatorSubsystem extends SubsystemBase{
    public TalonFX backElevator = new TalonFX(14);
    public TalonFX frontElevator = new TalonFX(15);

    CommandXboxController controller;
    Timer a_timer = new Timer();
    // Indicates if the elevator is in action
    Boolean isRunning = false;

    double motorPower = 0.5;

    // Initializes the motors and controller
    public ElevatorSubsystem(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        // Intakes on A button
        if (controller.a().getAsBoolean()) {
            setMotorPower(motorPower);
        }
        else{
            setMotorPower(0);
        }
    }

    // Sets power of both motors
    public void setMotorPower(double speed){
        frontElevator.set(speed);
        backElevator.set(speed);
    }

    /** Sets the position of both motors.
     * @param rotations is the number of rotations it will hold at.
     */
    public void setPositions(double rotations){
        frontElevator.setPosition(rotations);
        backElevator.setPosition(rotations);
    }

    /** Sets the position of both motors.
     * @param rotations is the number of rotations it will hold at.
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     */
    public void setPositions(double rotations, double timeoutSeconds){
        frontElevator.setPosition(rotations, timeoutSeconds);
        backElevator.setPosition(rotations, timeoutSeconds);
    }

    /** Holds the elevator at reef level 1. */
  public Command setReefLevel1() {
    return this.runOnce(() -> setPositions(2));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

}