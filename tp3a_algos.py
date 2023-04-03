#!/usr/bin/env python
'''
   CentraleSupelec TP3A
   Sylvain BERTRAND, 2023
   (all variables in SI unit)
   
   
   variables used by the functions of this script
       - nbRobots: nb of robots in the fleet (>1)    
       - robotNo: no of the current robot for which control is coputed (1 .. nbRobots)
       - poses:  size (3 x nbRobots) 
                 eg. of use: for robot no 'robotNo', its pose can be obtained by: poses[:,robotNo-1]   (indexes in Python start from 0 !)
                           poses[0,robotNo-1]: x-coordinate of robot position (in m)
                           poses[1,robotNo-1]: y-coordinate of robot position (in m)
                           poses[2,robotNo-1]: orientation angle of robot (in rad)
   
'''

import numpy as np
import math
import rospy




# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# if a variable here needs to be modified by a function, use 'global' keyword inside the function

toto = 1.0

# ===================================================================================




# =======================================
def consensus(robotNo, nbRobots, poses):
    # Paramètres
    K = 0.5  # Gain de consensus
    graph = np.array([[0, 1, 1, 0],
                      [1, 0, 1, 0],
                      [1, 1, 0, 1],
                      [0, 0, 1, 0]])  # Matrice d'adjacence du graphe de communication

    # Calcul du vecteur de consensus
    vx_sum, vy_sum = 0, 0
    for j in range(nbRobots):
        if graph[robotNo - 1, j] == 1:
            delta_x = poses[0, j] - poses[0, robotNo - 1]
            delta_y = poses[1, j] - poses[1, robotNo - 1]
            vx_sum += delta_x
            vy_sum += delta_y

    vx = K * vx_sum
    vy = K * vy_sum

    return vx, vy

# ====================================        





# ============================================    

def leaderFollower(robotNo, nbRobots, poses):
    K = 0.5  # Gain de suivi
    d_min = 0.5  # Distance minimale pour éviter les collisions

    if robotNo == 1:  # Leader
        vx = 0.2  # Vitesse constante en x pour le leader
        vy = 0.0  # Vitesse constante en y pour le leader
    else:  # Suiveurs
        # Trouver le robot précédent
        prev_robot = robotNo - 2  # L'index du tableau commence à 0

        # Calculer la différence de position avec le robot précédent
        delta_x = poses[0, prev_robot] - poses[0, robotNo - 1]
        delta_y = poses[1, prev_robot] - poses[1, robotNo - 1]

        # Calculer la distance avec le robot précédent
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Si la distance est inférieure à la distance minimale, réduire la vitesse
        if distance < d_min:
            K = K * (distance / d_min)

        # Calculer les vitesses en x et y
        vx = K * delta_x
        vy = K * delta_y

    return vx, vy

# ====================================    
    



# ======== ! DO NOT MODIFY ! ============
def controller(robotNo, nbRobots, poses):
# =======================================
    
    # UNCOMMENT THE ONE TO BE TESTED FOR EXPERIMENT
    vx,vy = consensus(robotNo, nbRobots, poses)
    #vx,vy = leaderFollower(robotNo, nbRobots, poses)
    
    return vx,vy
    
# ====================================   