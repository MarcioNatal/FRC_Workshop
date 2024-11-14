# FRC_Workshop

Uma iniciativa da equipes de FRC 7563, 7565, 7567, 9085, 9199, 9200, 9458, 9459, 9460 e 9461.

Workshop realizado na escola SENAI "Comendador Santoro Mirone" na cidade de Indaiatuba - SP de 11/11/2024 a 14/11/2024.

# Lab1: Motor REV controlado por eixo de Joystick

-> Teoria de framework

-> Boas práticas

-> JAVA Docs

-> Instalação biblioteca REV

-> Criando constantes na classe

-> Criando subsystem

-> Criando Joystick.


# Lab2: Motor REV controlado por botões e por eixo de Joystick

-> Criação de comandos

-> Criação de métodos para aquisição de valores do motor

-> Telemetria + Dashboard

-> Enviando mensagens para a DriverStation no formato de Logs


# Lab3: Motor REV controlado por botão em uma velocidade desejada PID degrau

-> PID de posicao pelo RoboRio disparado por botão e período padrão 20mS

	-> Extra KP e Posição ajustado via SmartDashboard;
 

# Lab4: Motor REV controlado por botão em uma velocidade desejada PID trapezoidal

-> PID trapezoidal de posicao pelo RoboRio disparado por botão e período padrão 20mS, Velocidade máxima e aceleração

	-> Extra KP e Posição ajustado via SmartDashboard;
 

# Lab5: Motor REV configurado para PID no SPark controle de velocidade e de posição.

-> PID declarado por Slot

# Lab6: Template para Swerve SDS M4k L4 usando motores Kraken para tração e NEO para turning, CanCoder no turning e NavX.

-> Controle por joystick

-> Antes de usar o template é necessário atualizar:
	
 	- a cinemática do robô, 
  
  	- redução dos motores, 
   
   	- offset dos cancoders e 
    
    	- constates relacionadas aos cálculos da odometria.

# Lab7: Autonomous Trajetória para Swerve

-> Criar um comando sequencial

 	- Criar modelos de trajetória para autonomous
  
   	- Criar Sendable Chooser no Robot Container para selecionar o autonomous na SuffleBoard

    	- Movimentação da trajetória somente no eixo frente/tras

# Lab8: Autonomous Trajetória para Swerve e Sendable chooser para seleção

-> Criar um comando sequencial

 	- Criar dois modelos de trajetória para autonomous
  
   	- Criar Sendable Chooser no Robot Container para selecionar o autonomous na SuffleBoard

    	- Dois autonomous para movimentação do swerve

# Lab9: ProfilePID Trapezoidal

-> Criar dois comandos que usam mesmos subsistemas

-> Dois motores acionados e com controle trapezoidal

# Lab10: Auto Path - PathPlanner

-> Criar autonomous usando o pathplanner

# Lab11: Swerve Pose Estimator

-> Instanciado pose estimator

-> Robô atualiza posição automaticamente quando lê AprilTag usando Megatag2

-> Instanciado seleção de Aliança para escolha de modo de leitura da AprilTag

# Lab12: Autonomous + PathPlanner + Pose Estimator

-> Configuração do AutoBuilder (pathPlanner) para atualização pelo PoseEstimator

-> Criado autonomous usando PathPlanner com odometria baseada pela AprilTag

# Obs:.

- Todos os programas foram testados , porém para cada implementação é preciso fazer as devidas adaptações de acordo com  tipo de módulo que a equipe utiliza.

- Sugestão: antes de usar qualquer modelo que primeiramente seja feita uma avaliação e análise do código fonte.

 - Trabalhe com segurança e reduza a velocidade nos testes iniciais.




