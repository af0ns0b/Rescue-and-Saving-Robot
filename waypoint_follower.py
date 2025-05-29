#!/usr/bin/env python

import rospy
import math
import os
import numpy as np
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import ActuatorControl
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')
        
        # Parameters
        self.waypoint_radius = rospy.get_param('~waypoint_radius', 2.0)  # metros
        self.base_value = 0.5  # valor base/parado
        self.control_delta = 0.4  # quanto adicionar/subtrair do valor base
        self.waypoints_file = rospy.get_param('~waypoints_file', 'waypoints.txt')
        self.control_rate = rospy.get_param('~control_rate', 5)  # Hz
        
        # Variáveis de estado
        self.current_position = None
        self.reference_position = None  # Posição de referência para cálculos relativos
        self.current_heading = None
        self.current_waypoint_index = 0
        self.waypoints = []
        
        # Publishers - apenas para controle de atuadores
        self.actuator_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
        
        # Debug publishers
        self.debug_pub = rospy.Publisher('/waypoint_follower/debug', Float64, queue_size=10)
        self.debug_string_pub = rospy.Publisher('/waypoint_follower/status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        
        # Carregar waypoints
        self.load_waypoints()
        
        # Log de configuração
        rospy.loginfo("Inicializado com método de controle: actuator")
        rospy.loginfo(f"Delta de controle: {self.control_delta}")
        rospy.loginfo(f"Raio de waypoint: {self.waypoint_radius}m")
        
        # Executar o controlador
        self.run()
    
    def position_callback(self, msg):
        # Assegura que posição é armazenada como (latitude, longitude)
        # Ignora altitude já que é um robô terrestre
        position = (msg.latitude, msg.longitude, 0.0)  # Define altitude como 0
        
        # Definir posição de referência para cálculos relativos se ainda não tivermos uma
        if self.reference_position is None:
            self.reference_position = position
            rospy.loginfo(f"Posição de referência definida: Lat={position[0]:.7f}, Lon={position[1]:.7f}")
        
        self.current_position = position
    
    def odom_callback(self, msg):
        # Extrair quaternion de orientação da odometria
        # A odometria é necessária para obter a orientação (heading) atual do robô
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        
        # Converter para ângulos de Euler (roll, pitch, yaw)
        euler = euler_from_quaternion(quaternion)
        self.current_heading = euler[2]  # yaw em radianos
    
    def load_waypoints(self):
        """Carregar waypoints do arquivo"""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        waypoints_path = os.path.join(script_dir, self.waypoints_file)
        rospy.loginfo(f"Carregando waypoints de: {waypoints_path}")
        
        try:
            with open(waypoints_path, 'r') as file:
                for line in file:
                    if line.strip() and not line.strip().startswith('#'):
                        parts = line.strip().split(',')
                        if len(parts) >= 2:
                            # Verifica e corrige a ordem das coordenadas se necessário
                            first_val = float(parts[0])
                            second_val = float(parts[1])
                            
                            # Verifica se a ordem parece estar invertida (long, lat)
                            if first_val < 0 and (second_val > 30 and second_val < 60):
                                # Parece estar no formato (long, lat) - invertendo
                                lat, lon = second_val, first_val
                                rospy.logwarn(f"Detectada ordem invertida no waypoint. Corrigindo: {first_val},{second_val} -> {lat},{lon}")
                            else:
                                # Assumimos formato (lat, lon)
                                lat, lon = first_val, second_val
                                
                            # Para robô terrestre, ignoramos altitude
                            self.waypoints.append((lat, lon, 0.0))
                            rospy.loginfo(f"Waypoint carregado -> Lat: {lat:.7f}, Lon: {lon:.7f}")
            
            if not self.waypoints:
                rospy.logerr("Nenhum waypoint válido encontrado no arquivo!")
        except Exception as e:
            rospy.logerr(f"Erro ao carregar waypoints: {e}")
    
    def simple_relative_position(self, ref_pos, target_pos):
        """
        Calcula a posição relativa entre dois pontos GPS de forma simples
        usando aproximação plana para curtas distâncias
        """
        ref_lat, ref_lon, _ = ref_pos
        target_lat, target_lon, _ = target_pos
        
        # Constantes para converter diferenças de graus para metros
        # Estas são aproximações que funcionam bem para distâncias pequenas
        METERS_PER_LAT = 111320.0  # 1 grau de latitude = ~111.32 km
        METERS_PER_LON = 111320.0 * math.cos(math.radians(ref_lat))  # Varia com latitude
        
        # Calcular diferenças simples
        lat_diff_m = (target_lat - ref_lat) * METERS_PER_LAT
        lon_diff_m = (target_lon - ref_lon) * METERS_PER_LON
        
        # Retorna como ENU (East-North-Up), com Up=0 para robô terrestre
        return lon_diff_m, lat_diff_m, 0.0
    
    def compute_control_value(self, error, max_error=10.0):
        """
        Computa valor de controle na faixa [0.0-1.0]:
        - < 0.5 para trás/esquerda
        - = 0.5 para parado
        - > 0.5 para frente/direita
        """
        # Se o erro for muito pequeno, fique parado
        if abs(error) < 0.1:
            return self.base_value
        
        # Normalizar o erro para no máximo 1.0 (em proporção a max_error)
        normalized_error = min(1.0, abs(error) / max_error)
        
        # Calcular o valor de controle
        if error > 0:
            return self.base_value + self.control_delta * normalized_error
        else:
            return self.base_value - self.control_delta * normalized_error
    
    def run(self):
        """Loop principal de controle"""
        rate = rospy.Rate(self.control_rate)
        
        rospy.loginfo("Aguardando posição GPS...")
        
        # Publicar mensagem de status inicial
        self.publish_status("Inicializando e aguardando GPS")
        
        # Contadores para debug
        cycle_count = 0
        last_status_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            cycle_count += 1
            
            # Esperar pela primeira posição GPS
            if self.current_position is None:
                if cycle_count % 10 == 0:  # Reduzir frequência de logs
                    rospy.loginfo("Aguardando posição GPS...")
                    self.publish_status("Aguardando posição GPS")
                rate.sleep()
                continue
            
            # Verificar se completamos todos os waypoints
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("Todos os waypoints alcançados!")
                self.stop_vehicle()
                self.publish_status("Missão completa")
                break
            
            # Obter waypoint atual
            waypoint = self.waypoints[self.current_waypoint_index]
            
            # Calcular posição relativa ao waypoint usando aproximação plana
            east, north, _ = self.simple_relative_position(self.current_position, waypoint)
            
            # Calcular distância até o waypoint (2D para robô terrestre)
            distance = math.sqrt(east*east + north*north)
            
            # Publicar informações de debug a cada segundo
            if (rospy.Time.now() - last_status_time).to_sec() >= 1.0:
                status_msg = f"Para waypoint {self.current_waypoint_index+1}: E={east:.1f}m, N={north:.1f}m, dist={distance:.1f}m"
                self.publish_status(status_msg)
                last_status_time = rospy.Time.now()
                
                # Log mais detalhado
                rospy.loginfo(f"Posições: Current={self.current_position[0]:.7f},{self.current_position[1]:.7f} | " +
                             f"Target={waypoint[0]:.7f},{waypoint[1]:.7f}")
                rospy.loginfo(f"Relativa: E={east:.2f}m, N={north:.2f}m, dist={distance:.2f}m")
            
            # Verificar se alcançamos o waypoint
            if distance < self.waypoint_radius:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} alcançado")
                self.current_waypoint_index += 1
                
                # Parar e estabilizar antes de ir para o próximo waypoint
                self.stop_vehicle()
                self.publish_status(f"Alcançou waypoint {self.current_waypoint_index}, pausando...")
                rospy.sleep(1.0)
                continue
            
            # Calcular rumo desejado
            desired_heading = math.atan2(east, north)
            
            # Calcular valores de controle (usando faixas de erro razoáveis)
            forward_control = self.compute_control_value(north, 20.0)  # 20m como erro máximo para frente/trás
            lateral_control = self.compute_control_value(east, 20.0)   # 20m como erro máximo para esquerda/direita
            
            # Calcular controle de orientação se tivermos informação de heading
            if self.current_heading is not None:
                # Calcular erro de heading (normalizado entre -pi e pi)
                heading_error = desired_heading - self.current_heading
                if heading_error > math.pi:
                    heading_error -= 2 * math.pi
                elif heading_error < -math.pi:
                    heading_error += 2 * math.pi
                
                yaw_control = self.compute_control_value(heading_error, math.pi)
            else:
                yaw_control = self.base_value
            
            # Aplicar controle adicional para distâncias maiores
            if distance > 10.0:
                # Se estamos muito longe, usar controle mais extremo
                rospy.logwarn_throttle(5.0, f"Distância grande ({distance:.1f}m), usando controle mais forte!")
                
                # Ajustar valores para mais extremos
                if forward_control > self.base_value:
                    forward_control = min(1.0, self.base_value + 2*self.control_delta)
                elif forward_control < self.base_value:
                    forward_control = max(0.0, self.base_value - 2*self.control_delta)
                
                if lateral_control > self.base_value:
                    lateral_control = min(1.0, self.base_value + 2*self.control_delta)
                elif lateral_control < self.base_value:
                    lateral_control = max(0.0, self.base_value - 2*self.control_delta)
            
            # Enviar comandos de controle
            self.send_actuator_control(forward_control, lateral_control, self.base_value, yaw_control)
            
            # Registrar status de controle (menos frequente)
            if cycle_count % 10 == 0:
                rospy.loginfo(f"Controles: Fwd={forward_control:.2f}, Lat={lateral_control:.2f}, Yaw={yaw_control:.2f}")
            
            rate.sleep()
    
    def send_actuator_control(self, fwd, lat, alt, yaw):
        """Enviar controle direto para os atuadores"""
        control_msg = ActuatorControl()
        control_msg.controls = [self.base_value] * 8  # inicializa o vetor com valores neutros
        control_msg.header.stamp = rospy.Time.now()
        control_msg.group_mix = 0  # Grupo MAIN
        
        # Definir valores de controle
        control_msg.controls[0] = fwd    # Frente/Trás
        control_msg.controls[1] = lat    # Esquerda/Direita
        control_msg.controls[2] = self.base_value    # Altitude (sempre neutro para robô terrestre)
        control_msg.controls[3] = yaw    # Yaw
        
        # Outros canais em neutro
        for i in range(4, 7):
            control_msg.controls[i] = self.base_value
        
        self.actuator_pub.publish(control_msg)
    
    def stop_vehicle(self):
        """Para o veículo usando o controle de atuadores"""
        # Parar usando valores neutros
        neutral = self.base_value
        
        # Enviar comando neutro para todos os canais
        self.send_actuator_control(neutral, neutral, neutral, neutral)
        
        rospy.loginfo("Comando de parada enviado através do controle de atuadores")
    
    def publish_status(self, status):
        """Publica mensagem de status para debug"""
        msg = String()
        msg.data = status
        self.debug_string_pub.publish(msg)

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
    except rospy.ROSInterruptException:
        pass
