"""Interface gráfica amigável (Tkinter) para testar os motores do AUV."""

import tkinter as tk
from tkinter import ttk

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from allblue2.action import TestAllMotors, TestMotor


class MotorGui(Node):
    def __init__(self):
        super().__init__('motor_gui')
        self.motor_client = ActionClient(self, TestMotor, 'test_motor')
        self.all_client = ActionClient(self, TestAllMotors, 'test_all_motors')
        self.goal = None
        self.active_test = None

        self.root = tk.Tk()
        self.root.title('AllBlue | Central de Teste dos Motores')
        self.root.geometry('700x440')
        self.root.minsize(620, 400)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        ttk.Label(
            self.root, text='Central de Teste dos Motores',
            font=('TkDefaultFont', 16, 'bold')
        ).pack(pady=(14, 2))
        ttk.Label(
            self.root,
            text='Escolha um motor para testar ou execute a sequência completa.'
        ).pack(pady=(0, 12))

        motor_frame = ttk.LabelFrame(self.root, text='Teste individual')
        motor_frame.pack(fill='x', padx=16, pady=5)
        self.buttons = []
        for i in range(1, 8):
            button = ttk.Button(
                motor_frame, text=f'Motor {i}',
                command=lambda m=i: self.test_motor(m)
            )
            button.grid(row=0, column=i - 1, padx=5, pady=10, sticky='ew')
            motor_frame.columnconfigure(i - 1, weight=1)
            self.buttons.append(button)

        actions = ttk.Frame(self.root)
        actions.pack(fill='x', padx=16, pady=8)
        self.all_button = ttk.Button(
            actions, text='▶  Testar todos os motores', command=self.test_all
        )
        self.all_button.pack(side='left')
        self.cancel_button = ttk.Button(
            actions, text='Cancelar teste', command=self.cancel, state='disabled'
        )
        self.cancel_button.pack(side='left', padx=8)

        status_frame = ttk.LabelFrame(self.root, text='Como está indo?')
        status_frame.pack(fill='x', padx=16, pady=5)
        self.status = tk.StringVar(
            value='👋 Tudo pronto! Selecione um motor para começar.'
        )
        ttk.Label(
            status_frame, textvariable=self.status, wraplength=640,
            justify='left'
        ).pack(anchor='w', padx=10, pady=(9, 5))

        self.progress = ttk.Progressbar(
            status_frame, orient='horizontal', mode='determinate',
            maximum=100, value=0
        )
        self.progress.pack(fill='x', padx=10, pady=5)

        self.tested = tk.StringVar(value='Motores testados: nenhum ainda')
        ttk.Label(
            status_frame, textvariable=self.tested, wraplength=640
        ).pack(anchor='w', padx=10, pady=(4, 9))

        self.log = tk.Text(self.root, height=5, wrap='word', state='disabled')
        self.log.pack(fill='both', expand=True, padx=16, pady=(5, 12))
        self.add_log('A interface está pronta. Os comandos serão enviados ao controlador ROS 2.')

    def add_log(self, message):
        self.log.configure(state='normal')
        self.log.insert('end', message + '\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def test_motor(self, motor):
        self.active_test = ('single', motor)
        self.progress['value'] = 0
        self.status.set(f'🚀 Preparando o teste do Motor {motor}...')
        self.add_log(f'Iniciando teste individual do Motor {motor}.')
        goal = TestMotor.Goal(motor_id=motor)

        def feedback(msg):
            self.status.set(f'⚙️ Motor {motor} em teste — {msg.feedback.elapsed_s:.1f} de 2,0 segundos. Os outros motores ficam em neutro.')
            self.progress['value'] = min(100, msg.feedback.elapsed_s / 2.0 * 100)

        self.send(self.motor_client, goal, feedback)

    def test_all(self):
        self.active_test = ('all', None)
        self.tested.set('Motores testados: preparando...')
        self.progress['value'] = 0
        self.status.set('🚀 Vamos começar! Primeiro, testaremos cada motor e depois todos juntos.')
        self.add_log('Iniciando a sequência completa dos sete motores.')
        self.send(self.all_client, TestAllMotors.Goal(), self.all_feedback)

    def all_feedback(self, msg):
        fb = msg.feedback
        self.progress['value'] = max(0, min(100, fb.progress * 100))
        tested = list(fb.tested_motors)
        self.tested.set(
            'Motores testados: ' + (', '.join(map(str, tested)) if tested else 'nenhum ainda')
        )
        if fb.active_motors:
            active = ', '.join(map(str, fb.active_motors))
            if len(fb.active_motors) == 7:
                self.status.set('⚙️ Testando os sete motores simultaneamente...')
            else:
                self.status.set(
                    f'⚙️ Testando o Motor {fb.active_motors[0]} de 7... '
                    f'Já testados: {len(tested)}/7.'
                )
        elif fb.progress >= 1.0:
            self.status.set('✅ Etapas concluídas! Finalizando e colocando os motores em neutro...')
        else:
            self.status.set(f'🔄 {fb.status.replace("/", " de ")}. Tudo está progredindo!')
        if fb.status.endswith('concluída'):
            self.add_log(f'Etapa concluída. Motores já testados: {tested}')

    def send(self, client, goal, feedback):
        if not client.server_is_ready():
            self.status.set('⚠️ Não consegui encontrar o controlador. Confira se o launch está rodando.')
            self.add_log('Não foi possível iniciar: servidor de ação indisponível.')
            self.active_test = None
            return
        self.enable(False)
        self.cancel_button.configure(state='normal')
        try:
            future = client.send_goal_async(goal, feedback_callback=feedback)
            future.add_done_callback(self.on_response)
        except Exception as exc:
            self.status.set('❌ Não consegui enviar o pedido de teste.')
            self.add_log(f'Erro ao enviar ação: {exc}')
            self.active_test = None
            self.enable(True)

    def on_response(self, future):
        try:
            self.goal = future.result()
        except Exception as exc:
            self.goal = None
            self.status.set('❌ Não consegui receber a resposta do controlador.')
            self.add_log(f'Erro na resposta: {exc}')
            self.active_test = None
            self.enable(True)
            return
        if not self.goal.accepted:
            self.goal = None
            self.status.set('⏳ O controlador recusou o teste. Talvez outro teste já esteja em andamento.')
            self.add_log('Pedido recusado pelo controlador.')
            self.active_test = None
            self.enable(True)
            return
        self.goal.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.status.set('❌ Ocorreu um problema ao receber o resultado.')
            self.add_log(f'Erro ao receber resultado: {exc}')
            self.goal = None
            self.active_test = None
            self.enable(True)
            return

        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success
        cancelled = wrapped.status == GoalStatus.STATUS_CANCELED
        if ok:
            self.progress['value'] = 100
            self.status.set('🎉 Teste concluído com sucesso! O controlador finalizou a sequência.')
            self.add_log('Teste finalizado com sucesso.')
        elif cancelled:
            self.status.set('🛑 Teste cancelado. O controlador solicitou o retorno dos motores ao neutro.')
            self.add_log('Teste cancelado pelo usuário.')
        else:
            message = getattr(wrapped.result, 'message', 'Não foi possível concluir o teste.')
            self.status.set('❌ O teste não terminou como esperado: ' + message)
            self.add_log('Teste finalizado com falha: ' + message)

        if hasattr(wrapped.result, 'tested_motors'):
            done = list(wrapped.result.tested_motors)
            self.tested.set('Motores testados: ' + (', '.join(map(str, done)) if done else 'nenhum'))
        elif self.active_test and self.active_test[0] == 'single' and ok:
            motor = self.active_test[1]
            self.tested.set(f'Motor {motor} testado com sucesso.')
        self.goal = None
        self.active_test = None
        self.enable(True)

    def cancel(self):
        if self.goal:
            self.status.set('🛑 Pedido de cancelamento enviado. Aguarde o controlador finalizar com segurança...')
            self.add_log('Pedido de cancelamento enviado.')
            self.goal.cancel_goal_async()
        else:
            self.status.set('ℹ️ Ainda não há um teste ativo para cancelar.')

    def enable(self, enabled):
        state = 'normal' if enabled else 'disabled'
        for button in self.buttons:
            button.configure(state=state)
        self.all_button.configure(state=state)
        self.cancel_button.configure(state='disabled' if enabled else 'normal')

    def close(self):
        if self.goal:
            try:
                self.goal.cancel_goal_async()
            except Exception:
                pass
        self.root.destroy()

    def spin(self):
        """Processa os callbacks ROS no loop da interface Tkinter."""
        if not rclpy.ok():
            if self.root.winfo_exists():
                self.root.destroy()
            return
        rclpy.spin_once(self, timeout_sec=0)
        if self.root.winfo_exists():
            self.root.after(20, self.spin)


def main():
    rclpy.init()
    gui = MotorGui()
    gui.spin()
    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
