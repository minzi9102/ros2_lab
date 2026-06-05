import sys

import rclpy
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QGridLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from ur3e_controller_msgs.srv import ExecuteNamedTarget

from ur3e_named_motion_gui_py.request_logic import (
    build_execute_request,
    summarize_response,
)


class NamedMotionGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.service_name = node.declare_parameter(
            'service_name',
            '/ur3e_named_motion_controller/execute_named_target',
        ).value
        self.human_confirmation = node.declare_parameter(
            'human_confirmation',
            '',
        ).value
        self.poll_period_ms = int(
            node.declare_parameter('poll_period_ms', 200).value
        )

        self.client = node.create_client(ExecuteNamedTarget, self.service_name)
        self.pending_future = None
        self.pending_target = None

        self.setWindowTitle('UR3e Named Motion')
        self.home_button = QPushButton('HOME')
        self.ready_button = QPushButton('READY')
        self.service_label = QLabel('service: checking')
        self.request_label = QLabel('request: idle')
        self.response_text = QTextEdit()
        self.response_text.setReadOnly(True)
        self.response_text.setMinimumHeight(96)

        self.home_button.clicked.connect(lambda: self.send_target('home'))
        self.ready_button.clicked.connect(lambda: self.send_target('ready'))

        button_layout = QGridLayout()
        button_layout.addWidget(self.home_button, 0, 0)
        button_layout.addWidget(self.ready_button, 0, 1)

        layout = QVBoxLayout()
        layout.addLayout(button_layout)
        layout.addWidget(self.service_label)
        layout.addWidget(self.request_label)
        layout.addWidget(self.response_text)
        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(self.poll_period_ms)
        self.update_controls()

    def service_ready(self):
        return self.client.wait_for_service(timeout_sec=0.0)

    def update_controls(self):
        ready = self.service_ready()
        busy = self.pending_future is not None
        self.service_label.setText(
            f'service: {"connected" if ready else "not connected"}'
        )
        self.home_button.setEnabled(ready and not busy)
        self.ready_button.setEnabled(ready and not busy)

    def send_target(self, target_name: str):
        if self.pending_future is not None:
            return
        if not self.service_ready():
            self.request_label.setText('request: service unavailable')
            self.update_controls()
            return

        request = build_execute_request(target_name, self.human_confirmation)
        self.pending_target = target_name
        self.pending_future = self.client.call_async(request)
        self.request_label.setText(f'request: {target_name} executing')
        self.response_text.setPlainText('')
        self.update_controls()

    def on_timer(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.pending_future is not None and self.pending_future.done():
            self.finish_pending_request()
        self.update_controls()

    def finish_pending_request(self):
        future = self.pending_future
        target_name = self.pending_target
        self.pending_future = None
        self.pending_target = None

        error = future.exception()
        if error is not None:
            self.request_label.setText(f'request: {target_name} failed')
            self.response_text.setPlainText(str(error))
            return

        response = future.result()
        if response is None:
            self.request_label.setText(f'request: {target_name} failed')
            self.response_text.setPlainText('service returned no response')
            return

        summary = summarize_response(response)
        self.request_label.setText(
            f'request: {target_name} {"done" if summary.success else "rejected"}'
        )
        self.response_text.setPlainText(summary.text)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('ur3e_named_motion_gui')
    app = QApplication(sys.argv)
    window = NamedMotionGui(node)
    window.resize(460, 220)
    window.show()
    try:
        return app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
