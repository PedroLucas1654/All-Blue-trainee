"""Controle dos motores do AUV: ações test_motor e test_all_motors."""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from allblue2.action import TestAllMotors, TestMotor

NEUTRAL = 1500
TEST_FORCE_N = 10.0  # use um valor menor no hardware real, ex.: 2.0


def force_to_pwm(force):
    """-10 N -> 1000, 0 N -> 1500, +10 N -> 2000 (linear)."""
    return int(NEUTRAL + force * 50)


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.pub = self.create_publisher(Int32MultiArray, '/motors', 10)
        self.lock = threading.Lock()  # tomado = há teste em andamento

        group = ReentrantCallbackGroup()
        cancel = lambda _: CancelResponse.ACCEPT  # noqa: E731
        ActionServer(self, TestMotor, 'test_motor', self.run_motor,
                     goal_callback=self.accept_motor,
                     cancel_callback=cancel, callback_group=group)
        ActionServer(self, TestAllMotors, 'test_all_motors', self.run_all,
                     goal_callback=self.accept_all,
                     cancel_callback=cancel, callback_group=group)
        self.publish([NEUTRAL] * 7)

    def publish(self, pwm):
        self.pub.publish(Int32MultiArray(data=pwm))

    def accept_motor(self, goal):
        if not 1 <= goal.motor_id <= 7:
            return GoalResponse.REJECT
        return self.accept_all(goal)

    def accept_all(self, goal):
        busy = not self.lock.acquire(blocking=False)
        return GoalResponse.REJECT if busy else GoalResponse.ACCEPT

    def drive(self, goal_handle, motors, feedback):
        """Aciona 'motors' por 2 s. Retorna False se for cancelado."""
        pwm = [NEUTRAL] * 7
        for m in motors:
            pwm[m - 1] = force_to_pwm(TEST_FORCE_N)
        start = time.monotonic()
        while (t := time.monotonic() - start) < 2.0:
            if goal_handle.is_cancel_requested:
                return False
            self.publish(pwm)
            feedback(t)
            time.sleep(0.05)
        return True

    def run_motor(self, goal_handle):
        motor = goal_handle.request.motor_id
        result = TestMotor.Result()

        def feedback(t):
            goal_handle.publish_feedback(TestMotor.Feedback(
                motor_id=motor, elapsed_s=t,
                status=f'Testando motor {motor} ({t:.1f}/2.0 s)'))

        try:
            result.success = self.drive(goal_handle, [motor], feedback)
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.canceled()
            result.message = f'Motor {motor}: ' + (
                'concluído' if result.success else 'cancelado')
        except Exception as exc:
            result.success = False
            result.message = f'Falha no teste do motor {motor}: {exc}'
            if goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().exception(result.message)
        finally:
            try:
                self.publish([NEUTRAL] * 7)
            except Exception as exc:
                self.get_logger().error(f'Falha ao publicar PWM neutro: {exc}')
            self.lock.release()
        return result

    def run_all(self, goal_handle):
        result = TestAllMotors.Result()
        steps = [[m] for m in range(1, 8)] + [list(range(1, 8))]
        tested = []

        try:
            for i, active in enumerate(steps):
                def feedback(t):
                    goal_handle.publish_feedback(TestAllMotors.Feedback(
                        active_motors=active, tested_motors=tested,
                        progress=(i + t / 2.0) / len(steps),
                        status=f'Etapa {i + 1}/{len(steps)}'))

                if not self.drive(goal_handle, active, feedback):
                    goal_handle.canceled()
                    result.message = 'Teste completo cancelado'
                    return result
                tested = sorted(set(tested) | set(active))
                self.publish([NEUTRAL] * 7)
                # Publica também o progresso atualizado após cada etapa.
                goal_handle.publish_feedback(TestAllMotors.Feedback(
                    active_motors=[], tested_motors=tested,
                    progress=(i + 1) / len(steps),
                    status=f'Etapa {i + 1}/{len(steps)} concluída'))

            result.success = True
            result.message = 'Teste completo concluído'
            goal_handle.succeed()
        except Exception as exc:
            result.success = False
            result.message = f'Falha no teste completo: {exc}'
            if goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().exception(result.message)
        finally:
            result.tested_motors = tested
            try:
                self.publish([NEUTRAL] * 7)
            except Exception as exc:
                self.get_logger().error(f'Falha ao publicar PWM neutro: {exc}')
            self.lock.release()
        return result


def main():
    rclpy.init()
    node = MotorController()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.publish([NEUTRAL] * 7)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
