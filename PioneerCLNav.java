// PioneerCLNav.java
/*
 * PioneerNavigation Class Definition
 * File: PioneerCLNav.java
 * Date: 9th Nov 2024
 * Description: More Advanced Navigation Class support (2024)
 * Author: Terry Payne (trp@liv.ac.uk)
 */
 
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.TouchSensor;

public class PioneerCLNav {

  private Supervisor robot;         // reference to the robot
  private Node robot_node;          // reference to the robot node
  private PioneerSimpleProxSensors pps; // reference to the sensor model
  private Motor left_motor;
  private Motor right_motor;
  
  private TouchSensor ts;
  private int timeStep;
  private int movement_counter;
  private int timer;
  private int decel_counter;
  private double bearing;
  private double goal_distance;
  private boolean goal_reachable;
  private boolean reachable;
  
  private Pose start;               // Initial location - set by set_goal()
  private Pose goal;                // goal location - set by set_goal()

  private boolean state;            // state of the robot - goal or wall following
  private boolean WALLFOLLOWING = true;
  private boolean GOALSEEKING = false;

  private int wf_state;         // state of the wall following algorithm
  private int WF_HIT = 0;
  private int WF_SEARCH = 1;
  private int WF_LEAVE = 2;
  private int WF_ROTATE = 3;

  private double bug_radius = 0.4;  // Because of wall following, allow a greater distance to hit/leave points
  private double goal_radius = 0.2; // If the robot is this dist from goal, then success
  private double goal_vel = 1;      // min velocity when robot reaches goal before stopping
  private double start_vel = 3;     // Initial move velocity
  private double accel_dist = 0.4;  // Accelerate up to distance away from the start  
  private double decel_dist = 0.8;  // Decelerate when this distance away from the goal  
  private double accel_angle = 0.524;  // Decelerate rotation when this angle (rads) from goal
  private double wf_ahead_dist = 0.4;     // Distance when starting wall following
  private double wf_dist = 0.25;     // Distance when wall following
  private double wf_vel = 5;        // Default velocity when wall following
  
  private double max_vel;           // Maximum velocity when robot travels

  private double prev_error;        // Previous error (PID controller)     
  private double total_error;       // Previous error (PID controller)     

  private long pid_counter;
  private double pid_diff;
  private Pose hit_point;           // Whenre the robot encountered the obstacle
  private Pose leave_point;         // Where the robot should leave an obstacle
  private double leave_dist;        // cached distance from leave point to goal
  
  private final double BEARINGERROR = Math.PI/64.0;    // error for bearing from start
  private final double MINDIST = 0.2;

  // ==================================================================================
  // Constructor
  // ==================================================================================
  public PioneerCLNav(Supervisor robot, PioneerSimpleProxSensors pps) {
    this.robot = robot;                       // reference to the robot
    this.robot_node = this.robot.getSelf();   // reference to the robot node
    this.pps = pps;                           // reference to the sensor model
    this.goal = this.get_real_pose();         // assume the goal is our current position
    this.start = this.get_real_pose();        // retain our initial position
    this.state = GOALSEEKING;

    this.prev_error = 0;              // Reset values used by the PID controller
    this.total_error = 0;             // Reset values used by the PID controller
        
    // enable motors
    this.left_motor = robot.getMotor("left wheel");
    this.right_motor = robot.getMotor("right wheel");
    this.left_motor.setPosition(Double.POSITIVE_INFINITY);
    this.right_motor.setPosition(Double.POSITIVE_INFINITY);
    
    this.max_vel = this.left_motor.getMaxVelocity();
    
    this.ts = robot.getTouchSensor("touch sensor");
    this.timeStep = (int) Math.round(robot.getBasicTimeStep());
    this.ts.enable(timeStep);
    this.movement_counter = 0;
    this.timer = 0;
    this.decel_counter = 0;

    // Initialise motor velocity
    this.stop();
  }

  // ==================================================================================
  // Internal (helper) methods
  // ==================================================================================
  
  private final void set_velocity(double lv, double rv) {
    this.left_motor.setVelocity(lv);
    this.right_motor.setVelocity(rv);
  }
  
  // returns the range to an obstacle in front of the robot
  // up to max_range. Used to trigger wall following
  private double range_to_frontobstacle() {
    // uses front sensors so3 & so4
    double range = this.pps.get_maxRange();
    range = Math.min (range, this.pps.get_value(2));
    range = Math.min (range, this.pps.get_value(3));
    range = Math.min (range, this.pps.get_value(4));
    range = Math.min (range, this.pps.get_value(5));    
    return range;
  }

  private double range_to_frontcornerobstacle() {
    // uses front sensors so3 & so4
    double range = this.pps.get_maxRange();
    range = Math.min (range, this.pps.get_value(1));
    range = Math.min (range, this.pps.get_value(2));
    range = Math.min (range, this.pps.get_value(5));
    range = Math.min (range, this.pps.get_value(6));    
    return range;
  }
  
  private double range_to_leftobstacle() {
    // uses front sensors so0 & so15
    double range = this.pps.get_maxRange();
    //range = Math.min (range, this.pps.get_value(1));
    range = Math.min (range, this.pps.get_value(0));
    range = Math.min (range, this.pps.get_value(15));
    range = Math.min (range, this.pps.get_value(14));
    return range;
  }

  

  // ==================================================================================
  // Internal (navigation) methods
  // ==================================================================================


  // ==================================================================================
  
  private double pid(double error) {
    double kp = 10.0; // proportional weight (may need tuning)
    double kd = 30.0; // differential weight (may need tuning)
    double ki = 0.0; // integral weight (may need tuning)
      
    double prop = error;
    this.total_error += error;
      
    if ((this.pid_counter % 4)==0) {
      pid_diff = error - this.prev_error;
      this.prev_error = error;
      }
    this.pid_counter+= 1; // increment our counter

    return (kp * prop) + (ki * this.total_error) + (kd * this.pid_diff);
  }
  
  // ==================================================================================
  public void adjust_velocity(double angle, double velocity) {
    // Modify velocity to track to the bearing
     double rotate_vel = Math.min(this.max_vel, 
                  Math.abs(angle*this.max_vel/this.accel_angle));
     
    // Determine which way to turn
    if (angle < 0)
      rotate_vel = -rotate_vel;
    double lv = Math.min(this.max_vel, velocity-rotate_vel);
    double rv = Math.min(this.max_vel, velocity+rotate_vel);
    this.set_velocity(lv, rv);
  }  
  
  public boolean rotate(double angle, double velocity) {
    // Check we are (approximately) at the correct
    // orientation (i.e. is the bearing small?)
    if (Math.abs(angle) < 0.1) {
      return true;
    }
    this.adjust_velocity(angle, wf_vel);
    return false;
  }

  // ==================================================================================
  private void track_leave_point(Pose p) {
    double dist = p.get_range(this.goal);   // distance to goal
    if (dist < this.leave_dist) {
      this.leave_dist = dist;
      this.leave_point = p;
    }
  }

  // ==================================================================================
  // Note we don't handle unaccessible goals
  private void bug1(Pose p) {
    double hit_dist = p.get_range(this.hit_point);
    if (this.wf_state == WF_ROTATE) {
      // About to leave, but need to rotate to the goal
      this.bearing = p.get_bearing(this.goal);
      // System.out.println(String.format(
        // "State: %d, goal dist %.03f bearing: %.03f",
        // this.wf_state, p.get_range(this.goal), bearing));
      if (this.rotate(bearing, wf_vel)) {
        this.state = GOALSEEKING;
        this.start = this.get_real_pose(); // reset start point
      }
      return;
    }
    
    if (this.wf_state == WF_LEAVE) {
      // System.out.println(String.format(
        // "State: %d, goal dist %.03f dist to leave: %.03f",
        // this.wf_state, p.get_range(this.goal), 
        // p.get_range(this.leave_point)));
        
      // If the robot crosses the m-line (straight line between the start and goal points)
      // then we know the goal is accessible.
      
      // Checks for crossing the m-line
      if (Math.abs(this.start.get_bearing(p)-this.bearing)<BEARINGERROR && 
                              p.get_range(this.goal)<this.goal_distance &&
                              p.get_range(this.goal)>MINDIST) {
        this.goal_reachable = true;
      }
      
      // Found Leave point; now need to rotate to goal
      if(p.get_range(this.leave_point) < this.bug_radius && this.goal_reachable) {
        this.start = p;
        
        double deltaX = goal.getX()-start.getX();
        double deltaY = goal.getY()-start.getY();
    
        this.goal_distance = Math.sqrt(Math.pow(deltaX,2)+Math.pow(deltaY,2));     // range
        
        this.wf_state = WF_ROTATE;
        return;
      }
    }

    
    if (this.wf_state == WF_SEARCH) {
      // System.out.println(String.format(
        // "State: %d, hit dist: %.03f goal dist %.03f leave point: %s %.03f",
        // this.wf_state, hit_dist, p.get_range(this.goal), 
        // this.leave_point, this.leave_dist));
        
      // If the robot crosses the m-line (straight line between the start and goal points)
      // then we know the goal is accessible. 
      
      // Checks for crossing the m-line
      if (Math.abs(this.start.get_bearing(p)-this.bearing)<BEARINGERROR && p.get_range(this.goal)<this.goal_distance) {
        this.goal_reachable = true;
      }  
      
      // Tracking distance to goal, and checking for hit point
      if (hit_dist < this.bug_radius) {
        if (this.goal_reachable)
          this.wf_state = WF_LEAVE;
        else
          this.reachable = false;
      } else {
        track_leave_point(p);
      }
    }
    
    if (this.wf_state == WF_HIT) {
      // System.out.println(String.format(
        // "State: %d, hit dist: %.03f goal dist %.03f leave point: %s %.03f",
        // this.wf_state, hit_dist, p.get_range(this.goal), 
        // this.leave_point, this.leave_dist));
      // Don't want to check for hit point until we have moved away
      this.start = p;
      this.goal_distance = p.get_range(this.goal);
      this.goal_reachable = false;
      
      if (hit_dist > this.bug_radius)
        this.wf_state = WF_SEARCH;
      track_leave_point(p);
    } 
    wall_following(p);
  }       
 
  private void wall_following(Pose p) {
    double front_range, left_range, control, error;
    double hit_dist = p.get_range(this.hit_point);
    
    this.decel_counter = 0;
    
    // Is the obstacle in front of us?  If so, then rotate right,
    // around the right wheel
    front_range = Math.min(range_to_frontobstacle(),
                                  range_to_frontcornerobstacle());
    if (front_range <= wf_ahead_dist) {
      this.set_velocity(wf_vel, 0.0);
    } else {
      left_range = this.range_to_leftobstacle();
      if (left_range < this.pps.get_maxRange()) {
        error = left_range - wf_dist;
        control = this.pid(error);
        
        // Keep control in a set range
        control = Math.min(control, wf_vel);
        control = Math.max(control, -wf_vel);      
        this.set_velocity(wf_vel, wf_vel+control);
      } else {
        // No wall, so turn
        this.set_velocity(wf_vel*0.25, wf_vel*0.75);
      }
    }
      
    return;
  }
  
  private void goal_seeking(Pose p) {
    // Insert goal seeking code here
    double accel_rate = (this.max_vel-start_vel)/this.accel_dist;
    double decel_rate = (this.max_vel-goal_vel)/(this.decel_dist-this.goal_radius);
    double yintercept = this.max_vel - (decel_rate * this.decel_dist);
    double start_dist = p.get_range(this.start);    // distance from start
    
    double goal_dist = p.get_range(this.goal);      // distance to goal
    this.bearing = p.get_bearing(this.goal);      // bearing to goal
    
    double obstacle_dist = this.range_to_frontobstacle();

    if (obstacle_dist <= this.wf_dist) {
      this.state = WALLFOLLOWING;
      this.wf_state = WF_HIT;
      this.prev_error = 0;       // Reset PID controller values
      this.total_error = 0;      // Reset PID controller values 
      this.pid_counter = 0;      // update every loop
      this.pid_diff = 0.0;
      this.hit_point = p;
      this.leave_point = p;
      this.leave_dist = goal_dist;
    } else {
    
      // Calculate acceleration velocity when leaving start
      double accel_vel = Math.min(this.max_vel, 
                          (start_dist*accel_rate) + this.start_vel);
      // Calculate deceleration velocity when approaching goal      
      double decel_vel = Math.min(this.max_vel, 
                          (goal_dist*decel_rate) + yintercept);
                        
      // Update deceleration velocity (e.g. when approaching obstacle)
      if ((obstacle_dist - this.wf_dist) < this.decel_dist) {
        decel_vel = Math.min(decel_vel, 
                 ((obstacle_dist - this.wf_dist)*decel_rate) + yintercept);
        
        this.decel_counter++; // counts how long it is decelerating for due to getting stuck
        
        // if it has been decelerating too long, it will go forwards, then start wall following
        if (this.decel_counter>60) {
          this.set_velocity(this.max_vel/2, this.max_vel/6);
          
          this.state = WALLFOLLOWING;
          this.wf_state = WF_HIT;
          this.prev_error = 0;       // Reset PID controller values
          this.total_error = 0;      // Reset PID controller values 
          this.pid_counter = 0;      // update every loop
          this.pid_diff = 0.0;
          this.hit_point = p;
          this.leave_point = p;
          this.leave_dist = goal_dist;
        }
        else if (this.decel_counter>50){
          this.set_velocity(this.max_vel/2, this.max_vel/6);
        }
      }
    
    // Determine final velocity
    double final_vel = Math.min(decel_vel, accel_vel);
    
    this.adjust_velocity(this.bearing, final_vel);
    }
  }

  // ==================================================================================
  // External methods (Navigation)
  // ==================================================================================

  public final void stop() {
    this.set_velocity(0.0, 0.0);
  }
  
  // ==================================================================================
  // Get Real Pose - ask the supervisor where the robot is
  // ==================================================================================
  // Note that the following method is defined as final, as it is used in the constructor  public final Pose get_real_pose() {
  public final Pose get_real_pose() {
    if (this.robot_node == null)
      return new Pose(0,0,0);
      
    double[] realPos = robot_node.getPosition();
    double[] rot = this.robot_node.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(-rot[0], rot[3]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
        theta2 = -(3*halfPi)+theta1;
    
    return new Pose(realPos[0], realPos[1], theta2);
  }
  
  // ==================================================================================
  // Go to some pose
  // ==================================================================================
  // update() updates the current action of the robot in moving to the goal
  // returns false if at goal or true if moving

  public boolean update() {
    Pose p = this.get_real_pose(); // current position
    if (p.get_range(this.goal) < goal_radius) {
      this.stop();
      return true;  // Found goal
    }
    
    double v = ts.getValue();
    
    // handles collisions with bumper sensor
    if (v > 0)
      this.movement_counter = 31;
    
    // in a normal case, the robot will follow standard procedure for navigation
    if (this.movement_counter == 0) {
    
      if (state==GOALSEEKING)
        goal_seeking(p);
      else
        bug1(p);
        
      this.timer++; // increment timer
    
    // in the event that the robot hits a wall, it will go back then reverse to the left
    } else if (this.movement_counter >= 23) {
        this.set_velocity(-this.max_vel/1.5, -this.max_vel/1.5);
        this.movement_counter--;
    } else {
      this.set_velocity(-this.max_vel/8, -this.max_vel/2);
      this.movement_counter--;
    }
    
    // Give up on trying to find goal after a certain length of time, set a new one
    if (this.timer>4000) {
      return true;
    }
    else {
      if (reachable)
        return false;  // Not yet found goal
      else
        return true;   // Goal is unreachable, set a new one
    }
  }
  
  public void set_goal(Pose p) { 
    this.goal = new Pose(p);
    this.start = get_real_pose();
    state = GOALSEEKING;
    
    // Set attributes for the distance between start and goal, the bearing between them, and 
    // whether the goal is reachable or not.
    double deltaX = goal.getX()-start.getX();
    double deltaY = goal.getY()-start.getY();
    this.goal_distance = Math.sqrt(Math.pow(deltaX,2)+Math.pow(deltaY,2));     // range
    this.bearing = start.get_bearing(goal);                                    // bearing
    this.goal_reachable = false;
    this.reachable = true;
    
    this.update();
  }
  
  // resets the timer when a new goal is set
  public void reset_timer() {
    this.timer = 0;
  }
  
  // these functions are used in debugging
  public Pose getStart() {
    return this.start;
  }
  
  public Pose getGoal() {
    return this.goal;
  }
  
  public double getMLine() {
    return this.bearing;
  }
  
  public double getGoalDist() {
    return this.goal_distance;
  }
}
