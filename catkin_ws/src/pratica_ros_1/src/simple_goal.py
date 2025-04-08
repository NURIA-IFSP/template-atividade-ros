#! /usr/bin/env python

"""
simplegoal_pub.py é um programa para aprender a usar nos no python
Este no publica uma mensagem do tipo Pose2D necessaria ao programa
go_to_xy.py que direciona a turtlesim para uma posicao na tela.
Este programa acompanha o livro Practical Robotics in C++
escrito por Lloyd Brombach e publicado pela Packt Publishing 
É uma adaptação do programa original em C++
Autor: Luiz C M Oliveira
"""
# Import the Python library for ROS
import rospy

# Importa as mensagens do tipo Twist e Pose2D do pacote geometry_msgs
from geometry_msgs.msg import Pose2D

alvo = Pose2D()  # posicao alvo

def get_position():
    x_alvo = float(input("Qual posicao x desejada [0 e 11]: "))
    y_alvo = float(input("Qual posicao y desejada [0 e 11]: "))
    alvo.x = x_alvo
    alvo.y = y_alvo

# Exemplo de chamada de configuração
if __name__ == "__main__":
        
    # Inicializa o topico go_to_x - conecta ao roscore
    rospy.init_node('simple_goal')

    # chama funcao para obter a posicao desejada
    get_position()
    
    # Set a publish rate of 10 Hz
    rate = rospy.Rate(5)

    # Registrar o nó como um publicador
    pub_alvo = rospy.Publisher('alvo', Pose2D, queue_size=10)

    rospy.sleep(1)  # Aguardar 1 segundo antes de iniciar o loop

    pub_alvo.publish(alvo)

    rospy.loginfo("Publishing goal...")

    # Loop que sera executado ate o encerramento
    # da execucao do programa
    #while not rospy.is_shutdown():
        
        # Publica a velocidade - movimenta a tartaruga
    #    pub_alvo.publish(alvo)
        
        # Aguarda ate o proximo ciclo - frequencia 5Hz
    #    rate.sleep()   