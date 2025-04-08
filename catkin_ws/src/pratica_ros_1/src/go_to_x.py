#! /usr/bin/env python

# 
# go_to_x.py eh programa para aprender a escrever ROS nodes.
# Este node comanda o turtlesim turtle para a posicao X
# manualmente informada na constante do tipo double GOAL abaixo.
# Ao mudar o valor da constante para qualquer valor entre 0 e 11 recarregue o programa
# Este programa eh uma conversao baseada no livro "Practical Robotics in C++"
# Escrito por Lloyd Brombach e publicado pela Packt Publishing
# Luiz Marangoni


# Import the Python library for ROS
import rospy

# Importa as mensagens do tipo Twist e Pose2D do pacote geometry_msgs
from geometry_msgs.msg import Twist, Pose2D

# importa a mensagem do tipo Pose do pacote turtlesim
from turtlesim.msg import Pose

# Declaração de variáveis
cmdVel = Twist()    # comando de velocidade
current = Pose2D()  # posicao atual
desired = Pose2D()  # posicao alvo

# Define o objetivo (pode ser qualquer valor de 0 a 11)
GOAL = 10.5

# Coeficiente (ou ganho) para o cálculo da velocidade linear
K1 = 1

# Tolerancia aceita como "perto suficiente"
distanceTolerance = .1

# Callback da subscricao em turtle1/pose chamada toda vez que
# um dado for recebido
# Obtem a posicao atual da tartaruga - currentPose e carrega
# na variavel current

def update_Pose(currentPose):
  global current
  current.x = currentPose.x
  current.y = currentPose.y
  current.theta = currentPose.theta

# inicializacao das variaveis de velocidade e pos desejada
def misc_setup():
  desired.x = GOAL
  cmdVel.linear.x = 0
  cmdVel.linear.y = 0
  cmdVel.linear.z = 0
  cmdVel.angular.x = 0
  cmdVel.angular.y = 0
  cmdVel.angular.z = 0

# calculo do erro na distancia
def getDistanceError():
  return (desired.x - current.x)

# Se nao estiver perto suficiente ajuste o valor de cmd_vel
# Se estiver, ajuste para zero (para a tartaruga)
def set_velocity():
  if abs(getDistanceError()) > distanceTolerance:
    cmdVel.linear.x = K1 * getDistanceError()
  else:
    rospy.loginfo("I'm here")
    cmdVel.linear.x = 0

    
# Exemplo de chamada de configuração
if __name__ == "__main__":
        
    # Inicializa o topico go_to_x - conecta ao roscore
    rospy.init_node('go_to_x')

    # chama funcao para iniciar a velocidade
    misc_setup()

    # Registra o no como subscritor, chama a funcao callback
    # quando receber um dado com a posicao da tartaruga
    sub = rospy.Subscriber('/turtle1/pose', Pose, update_Pose)

    # Atualiza a velocidade da tartaruga
    # Se vincula ao topico /turtle1/cmd_vel
    # para publicacao

    # Registrar o nó como um publicador
    pub_velocity = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Set a publish rate of 10 Hz
    rate = rospy.Rate(10)

    # Loop que sera executado ate o encerramento
    # da execucao do programa
    while not rospy.is_shutdown():
        # Ajusta a velocidade
        set_velocity()
        
        # Publica a velocidade - movimenta a tartaruga
        pub_velocity.publish(cmdVel)
        
        # escreve na tela para acompanhamento

        rospy.loginfo("goal x = %f", desired.x)
        rospy.loginfo("current x = %f", current.x)
        rospy.loginfo("disError = %f", getDistanceError())
        rospy.loginfo("cmd_vel = %f", cmdVel.linear.x)


        # Aguarda ate o proximo ciclo - frequencia 10Hz
        rate.sleep()                             