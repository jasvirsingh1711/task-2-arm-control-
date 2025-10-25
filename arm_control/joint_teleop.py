import rclpy # ROS2 Python client library
from rclpy.node import Node # Node class from rclpy
from sensor_msgs.msg import JointState # JointState message type
from std_msgs.msg import Header # Header message type
import sys
import termios # Terminal I/O settings library
import tty # Terminal control library means to set terminal to raw mode
import select # Library for waiting for I/O completion

key_bindings={
    'q':('Joint_1', 0.1 ), # increase joint 1 by 0.1 rad
    'a':('Joint_1', -0.1), # decrease joint 1 by 0.1 rad
    'w':('Joint_2', 0.1 ), # increase joint 2 by 0.1 rad
    's':('Joint_2', -0.1), # decrease joint 2 by 0.1 rad
    'e':('Joint_3', 0.1 ), # increase joint 3 by 0.1 rad
    'd':('Joint_3', -0.1), # decrease joint 3 by 0.1 rad
    'r':('Joint_4', 0.1 ), # increase joint 4 by 0.1 rad
    'f':('Joint_4', -0.1), # decrease joint 4 by 0.1 rad
    't':('Joint_5', 0.1 ), # increase joint 5 by 0.1 rad
    'g':('Joint_5', -0.1), # decrease joint 5 by 0.1 rad
    'y':('Joint_6', 0.1 ), # increase joint 6 by 0.1 rad
    'h':('Joint_6', -0.1), # decrease joint 6 by 0.1 rad

    'u': ('pointer_extend_joint', 0.01), # extend pointer
    'j': ('pointer_extend_joint', -0.01), # retract pointer
}

# [NEW] Yeh function controls print karega
def print_controls():
    print("\n--- Control Your Arm ---")
    print("---------------------------")
    print("Joint 1 (q/a) | Joint 2 (w/s) | Joint 3 (e/d)")
    print("Joint 4 (r/f) | Joint 5 (t/g) | Joint 6 (y/h)")
    print("Pointer (u/j)")
    print("---------------------------")
    print("Press 'l' or 'Ctrl-C' to quit.")
    print("\n") # extra line for spacing

def get_key():
    #check if a key has been pressed
    if select.select([sys.stdin], [],[],0.05)[0]==[sys.stdin]:
        return sys.stdin.read(1)
    return None

def save_terminal_settings():
    settings=termios.tcgetattr(sys.stdin)
    return settings 

def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class JointTeleopNode(Node):
    def __init__(self):
        super().__init__('joint_teleop_node')

        self.joint_names=[ 
            'Joint_1',
            'Joint_2',
            'Joint_3',
            'Joint_4',
            'Joint_5',
            'Joint_6',
            'pointer_extend_joint' 
        ]
        self.joint_positions=[0.0]* len(self.joint_names)

        self.publisher=self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info("Joint Teleop Node has been started.") # Yeh message abhi bhi rahega

    def publish_joint_states(self):
        # Create JointState message
        msg=JointState()
        msg.header=Header()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.joint_names
        msg.position=self.joint_positions

        # Publish the message
        self.publisher.publish(msg)

    
        # log_positions = [f" {position:.2f}" for position in self.joint_positions]
        # self.get_logger().info(f"Published Joint States:{','.join(log_positions)}")

    def update_joint_positions(self, key):
        if key in key_bindings:
            joint_name, delta= key_bindings[key]

            try:
                index=self.joint_names.index(joint_name) # find index of the joint
                self.joint_positions[index] += delta # update joint position
                
                # Console par current positions dikhayein (spam kiye bina)
                positions_str = [f"{pos:.2f}" for pos in self.joint_positions]
                print(f"Current Positions: {', '.join(positions_str)}", end='\r', flush=True)

                return True
            except ValueError:
                self.get_logger().warn(f"Joint name {joint_name} not found.")
                return False
        return False
    
def main(args=None):
    rclpy.init(args=args)

    settings = save_terminal_settings() # Settings save ho gayi
    
    # Terminal ko RAW mode mein daala
    # Yeh zaroori hai taaki key press turant register ho (bina Enter dabaye)
    tty.setraw(sys.stdin.fileno())

    node = JointTeleopNode()
    
    print_controls() # [NEW] Instructions yahaan print hongi

    try:
        #the main loop
        while rclpy.ok():
            key=get_key() 
            if key:
                if key=='\x03' or key == 'l': # Ctrl-C ya 'l' se quit
                    print("\nQuitting...")
                    break
                node.update_joint_positions(key)

            #  Publish hamesha hona chahiye
            # RViz ko lagaataar data chahiye, varna woh "stuck" ho jayega.
            # Isliye publish() ko 'if key:' ke bahar move kar diya hai.
            node.publish_joint_states()

            # Allow ROS2 to process incoming messages and events
            rclpy.spin_once(node, timeout_sec=0.01) # Thoda fast spin
            
    except Exception as e:
        print(e) # print any exception that occurs
    finally:
        # Terminal settings ko restore kiya
        restore_terminal_settings(settings)
        # Terminal cursor ko saaf karne ke liye ek extra print
        print("\n") 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

