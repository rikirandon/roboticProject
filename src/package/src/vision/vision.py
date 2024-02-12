#!/usr/bin/env python

import rospy
from package.srv import coordinates
from package.srv import coordinatesResponse
from geometry_msgs.msg import Pose

import numpy as np

i = 0
# height, width and length of the blocks
length = 0.03/2
width = 0.06/2
height = 0.02/2


blocks = [
    [0.64, 0.6, 0.925, 0.0, 0.0, 0.0],
    [0.82, 0.57, 0.925, 0.0, 0.0, 0.0],
    [0.5, 0.7, 0.925, 0.0, 0.0, 0.0]]

final = [
    [0.2, 0.2, 0.925, 0.0, 0.0, 0.0],
    [0.2, 0.5, 0.925, 0.0, 0.0, 0.0],
    [0.3, 0.7, 0.925, 0.0, 0.0, 0.0]]


def center_blocks(blocks):
    for block in blocks:
        block[0]+=length
        block[1]+=width
        block[2]+=height
    return blocks

def vision():
    rospy.init_node('vision')  # Inizializza il nodo ROS
    # Crea un server del servizio chiamato 'coordinates' utilizzando la definizione del tipo di servizio e la funzione di callback
    service = rospy.Service('coordinates', coordinates, handle_coordinates)
    rospy.loginfo("Server del servizio 'coordinates' pronto")
    rospy.spin()  # Mantieni il programma in esecuzione


def handle_coordinates(req):
    global i
    # Questa è la funzione di callback che gestisce le richieste del servizio
    pose = Pose()
    rospy.loginfo("Richiesta di coordinate ricevuta")
    robot_coords = transform_vector(center_blocks(blocks)[i])
    pose.position.x= robot_coords[0]
    pose.position.y= robot_coords[1]
    pose.position.z= robot_coords[2]
 
    
    final_pose = Pose()
    final_coords= final[i]
    final_pose.position.x= final_coords[0]
    final_pose.position.y= final_coords[1]
    final_pose.position.z= final_coords[2]

    response = coordinatesResponse()
    response.poseStart= pose
    response.poseFinal= final_pose
    if(i==2):
       response.flag = False
    else:
        response.flag = True
        i = i + 1
    return response

        
def transform_vector(vector):
    m= np.zeros((4, 4))
    vector_translation = vector[:3]
    orientation_part = vector[3:]
    m = [[1, 0, 0, 0.5],
        [0, -1, 0, 0.55],
        [0, 0, -1, 0],
        [0, 0, 0, 1]]

    homogeneous_vector = np.append(vector_translation, 1)
    transformed_vector = np.dot(m, homogeneous_vector)
    transformed_vector=transformed_vector[:-1]
    return transformed_vector+orientation_part


if __name__ == "__main__":
    vision()

