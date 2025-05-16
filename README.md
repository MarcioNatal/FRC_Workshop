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

# Lab12: Autonomous + PathPlanner + Pose Estimator + Limelight

-> Configuração do AutoBuilder (pathPlanner) para atualização pelo PoseEstimator

-> Criado autonomous usando PathPlanner com odometria baseada pela AprilTag

# Lab13: Swerve SDS M4ki L3, usando motores Kraken para tração e NEO para turning, CanCoder no turning e Pigeon.

-> Controle por joystick

-> Antes de usar o template é necessário atualizar:
	
 	- a cinemática do robô, 
  
  	- redução dos motores, 
   
   	- offset dos cancoders e 
    
    	- constates relacionadas aos cálculos da odometria.

-> Modelo com sistema de visão Limelight, leitura de AprilTags, Path Planner, Pose Estimator e Autonomous

# Obs:.

- Todos os programas foram testados , porém para cada implementação é preciso fazer as devidas adaptações de acordo com  tipo de módulo que a equipe utiliza.

- Sugestão: antes de usar qualquer modelo que primeiramente seja feita uma avaliação e análise do código fonte.

 - Trabalhe com segurança e reduza a velocidade nos testes iniciais.


## Robot Systems Overview

### Swerve Drive

The **Swerve Drive** enables precise, omnidirectional movement, allowing the robot to navigate the field with agility. The code is separated into three main components:

### 1. **SwerveModules**

This part of the code controls the output for each individual wheel. Each module is responsible for one wheel's direction and speed. The configuration of the modules is shown in the image below:

![Swerve Modules](https://github.com/user-attachments/assets/3db5946f-9272-4c8a-95ba-dd79039eae32)

### 2. **Swerve Subsystem / Command**

Here, the code combines the wheel outputs and creates **ChassisSpeeds** to control the robot’s movement. Joystick inputs are processed and translated into robot output speeds.

![Swerve Subsystem](https://github.com/user-attachments/assets/bf6685db-2e2a-4c8e-84aa-dfef68be0d86)

### 3. **Swerve Control System**

This section includes key controls to automate the robot's movements, such as:

- **Pose Estimator**: Used to estimate the robot's position on the field by combining encoder data and vision inputs.
- **Path Planner**: For autonomous navigation and path-following.

### Pose Estimator

The **Pose Estimator** class wraps the **Swerve Drive Odometry** and helps fuse latency-compensated vision data with encoder data. This allows us to accurately estimate the robot's position on the field.

#### Instantiating the Pose Estimator:

```java
private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );
```

---

## Path Planner

**Path Planner** is a motion profile generator used in FRC robots. It allows for precise path-following and generates motion profiles using Bézier curves. Some notable features of **Path Planner** include:

- **Holonomic mode**: This mode decouples the robot's rotation from its direction of travel, allowing for more flexible path following.
- **Real-time path preview**: Visualize paths in real-time during robot operation.
- **Event markers**: Markers can trigger other code along the path, such as triggering actions at specific points during path following.
- **Path generation**: Create motion profiles and commands for autonomous routines.
- **Path reloading**: Paths can be reloaded on the robot without the need to redeploy code.

### Instantiating Path Planner

To set up **Path Planner**, we configure the **AutoBuilder** with the required parameters, such as the robot's pose, odometry reset method, and driving method:

```java
AutoBuilder.configure(
    this::getPoseEstimator,        // Robot pose supplier
    this::resetOdometry,           // Method to reset odometry
    this::getChassisSpeeds,        // ChassisSpeeds supplier (ROBOT RELATIVE)
    this::drive,                   // Method to drive the robot (ROBOT RELATIVE)
    PathPlannerConstants.AutoConfig, 
    config,                        // Robot configuration
    () -> {
        // Control path mirroring for the red alliance
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    },
    this                            // Reference to this subsystem
);
```

In this code, we configure the **AutoBuilder** with the following:
- A **pose estimator** to provide the current pose of the robot.
- A method to **reset odometry** when a new autonomous path starts.
- A **ChassisSpeeds supplier** that provides robot velocities relative to the robot's frame of reference.
- A **drive method** to apply velocities to the robot's motors.

### In the Robot Container

To use **Path Planner** for autonomous routines, define an autonomous path and add it to the **SendableChooser**. This allows the robot to select and execute specific paths at runtime:

```java
PathPlannerAuto autoTest = new PathPlannerAuto("AutoTest");
chooserAuto.addOption("AutoTest", autoTest);
```

This code adds a new autonomous routine named "AutoTest" to the **SendableChooser**, which will allow the robot to choose this routine during a match.

---

## Pathfinding

**Pathfinding** with **PathPlannerLib** enables the robot to automatically generate a path between two points while avoiding obstacles. The generated path can be combined with pre-planned paths for more refined control.

### Initializing Pathfinding

To use the **Pathfinding** capabilities, first create the necessary constraints (e.g., velocity, acceleration, etc.):

```java
// Pathfinding constraints
PathConstraints constraints = new PathConstraints(
    maxVelocity, 
    PathPlannerConstants.maxAccelerationPath,
    PathPlannerConstants.maxAngularVelocityRadPerSec,
    PathPlannerConstants.maxAngularAccelerationRadPerSecSq
);

// Building the pathfinding command
pathFindingCommand = AutoBuilder.pathfindToPose(
    targetPose,
    constraints,
    setEndVelocity  // Goal end velocity in meters/sec
);

pathFindingCommand.schedule();
```

This code defines the constraints for pathfinding, including maximum velocity, angular velocity, and acceleration. Then, it creates the pathfinding command that will generate a path to the target pose and execute the movement.

### Stopping Pathfinding

When pathfinding is no longer needed (for example, if the task is finished or the robot encounters an obstacle), the following method can be called to cancel the pathfinding command and stop the robot:

```java
public void endCommand() {
    pathFindingCommand.cancel();
    PPHolonomicDriveController.overrideXYFeedback(() -> swerveDrive.getPoseEstimator().getX(), 
                                                   () -> swerveDrive.getPoseEstimator().getY());
    PPHolonomicDriveController.overrideRotationFeedback(() -> swerveDrive.getPoseEstimator().getRotation().getRadians());
    System.out.println("EndCommand");
    finish = true;
}
```

This code cancels the ongoing **Pathfinding** command and stops the robot's movement. It also overrides the robot's feedback controls to ensure that the robot is no longer following the generated path.
```

```
## Swerve initialization

### Pigeon Orientation

![Screenshot 2025-05-16 091657](https://github.com/user-attachments/assets/945b1f4b-337a-4076-a818-06c7f3e861c8)







