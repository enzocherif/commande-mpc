#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
import tf.transformations as tf
from pympc.controllers.mpc import MPCController
from pympc.models.dynamics.turtlebot import Turtlebot
from pympc.models.ros_interface.turtlebot import TurtlebotROSInterface

class MPCControlLaw:
    def __init__(self, leader_pose_topic, follower_pose_topic, follower_cmd_vel_topic, horizon=10, d_min=0.5, d_max=2.0):
        """
        Initialisation du contrôleur MPC.
        """
        self.leader_pose = None
        self.follower_pose = None
        self.d_min = d_min
        self.d_max = d_max
        self.horizon = horizon

        # Définition des modèles et interface Turtlebot
        self.turtlebot_model = Turtlebot()
        self.mpc_controller = MPCController(self.turtlebot_model, horizon=horizon)
        self.ros_interface = TurtlebotROSInterface()

        # Abonnements aux topics
        self.leader_sub = rospy.Subscriber(leader_pose_topic, PoseStamped, self.leader_callback)
        self.follower_sub = rospy.Subscriber(follower_pose_topic, PoseStamped, self.follower_callback)

        # Publication des commandes de mouvement
        self.cmd_vel_pub = rospy.Publisher(follower_cmd_vel_topic, Twist, queue_size=10)

        rospy.loginfo("MPCControlLaw initialized.")

    def leader_callback(self, msg):
        """Callback pour mettre à jour la pose du leader."""
        self.leader_pose = msg.pose

    def follower_callback(self, msg):
        """Callback pour mettre à jour la pose du suiveur."""
        self.follower_pose = msg.pose

    def compute_command(self):
        """
        Calculer la commande MPC pour le suiveur.
        """
        if self.leader_pose is None or self.follower_pose is None:
            rospy.logwarn("Leader or Follower pose not available.")
            return

        # Position et orientation actuelles
        follower_x = self.follower_pose.position.x
        follower_y = self.follower_pose.position.y
        yaw_follower = self.get_yaw_from_pose(self.follower_pose)

        # Position cible du leader
        leader_x = self.leader_pose.position.x
        leader_y = self.leader_pose.position.y

        # Contrainte sur la distance optimale
        distance = math.sqrt((leader_x - follower_x)**2 + (leader_y - follower_y)**2)
        distance_error = distance - (self.d_min + self.d_max) / 2

        # Définition des états et des contraintes pour le MPC
        x0 = [follower_x, follower_y, yaw_follower]
        target = [leader_x, leader_y]

        self.mpc_controller.set_target(target)
        self.mpc_controller.add_constraint("distance", lower_bound=self.d_min, upper_bound=self.d_max)

        # Calcul de la commande optimale
        u = self.mpc_controller.compute_optimal_control(x0)

        # Commande Twist
        cmd = Twist()
        cmd.linear.x = u[0]  # Vitesse linéaire optimale
        cmd.angular.z = u[1]  # Vitesse angulaire optimale

        # Publier la commande
        self.cmd_vel_pub.publish(cmd)

    def get_yaw_from_pose(self, pose):
        """Extraire l'angle de rotation (yaw) à partir de la quaternion d'une pose."""
        orientation_q = pose.orientation
        _, _, yaw = tf.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        return yaw

    def spin(self):
        """Exécuter en boucle la logique de contrôle."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.compute_command()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("mpc_control")

    # Paramètres ROS
    leader_pose_topic = rospy.get_param("~leader_pose_topic", "/qtm/pose/tb0")
    follower_pose_topic = rospy.get_param("~follower_pose_topic", "/qtm/pose/tb1")
    follower_cmd_topic = rospy.get_param("~follower_cmd_vel_topic", "/tb1/cmd_vel")
    horizon = rospy.get_param("~horizon", 10)
    d_min = rospy.get_param("~d_min", 0.5)
    d_max = rospy.get_param("~d_max", 2.0)

    # Initialisation du contrôleur
    controller = MPCControlLaw(leader_pose_topic, follower_pose_topic, follower_cmd_topic, horizon, d_min, d_max)
    controller.spin()
