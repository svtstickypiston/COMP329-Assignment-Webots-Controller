import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.TouchSensor;

public class BumperSensor {

  public static void main(String[] args) {

    // create the Robot instance.
    Robot robot = new Robot();
    Motor left_motor;
    Motor right_motor;

    int movement_counter = 0;
    int speed=6;
    int left_speed, right_speed;

    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    TouchSensor ts = robot.getTouchSensor("touch sensor");
    ts.enable(timeStep);

    left_motor = robot.getMotor("left wheel");
    right_motor = robot.getMotor("right wheel");
    left_motor.setPosition(Double.POSITIVE_INFINITY);
    right_motor.setPosition(Double.POSITIVE_INFINITY);

    left_motor.setVelocity(0.0);
    right_motor.setVelocity(0.0);   
    
    while (robot.step(timeStep) != -1) {
      double v = ts.getValue();
      if (v > 0)
        movement_counter = 31;

      /*
       * We use the movement_counter to manage the movements
       * of the robot. When the value is 0 we move straight,
       * then when there is another value this means that we
       * are avoiding an obstacle. For avoiding we first move
       * backward for some cycles and then we turn on ourself.
       */
      if (movement_counter == 0) {
        left_speed = speed;
        right_speed = speed;
      
      } else if (movement_counter >= 20) {
        left_speed = -speed/2;
        right_speed = -speed/2;
        movement_counter--;
      } else {
        left_speed = -speed / 2;
        right_speed = speed;
        movement_counter--;
      }
      
      left_motor.setVelocity(left_speed);
      right_motor.setVelocity(right_speed);   
        };

  }
}