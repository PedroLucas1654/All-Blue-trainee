# AllBlue 2 — pacote ROS 2 único

Este projeto foi organizado em **um único pacote ROS 2**, chamado `allblue2`.

- `allblue2_py/motor_controller.py`: servidor das Actions e publicação de PWM em `/motors`.
- `allblue2_py/motor_gui.py`: interface gráfica Tkinter.
- `action/TestMotor.action` e `action/TestAllMotors.action`: interfaces geradas pelo próprio pacote.
- `launch/motors_test.launch.py`: inicia o controlador e a interface.

## Dependências (ROS 2 Humble)

```bash
sudo apt update
sudo apt install ros-humble-rosidl-default-generators ros-humble-ament-cmake-python python3-tk
```

## Compilar

Extraia a pasta `allblue2` para `~/ros2_ws/src/allblue2` e execute:

```bash
cd ~/ros2_ws
rm -rf build/allblue2 install/allblue2
colcon build --packages-select allblue2
source install/setup.bash
```

## Executar

```bash
ros2 launch allblue2 motors_test.launch.py
```

Para observar os comandos publicados, em outro terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /motors
```

**Atenção:** este software publica valores de PWM de teste; não conecte a motores reais sem validar a faixa, a interface elétrica e o procedimento de segurança do hardware.
