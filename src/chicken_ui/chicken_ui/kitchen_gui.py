# import rclpy
# import threading
# from rclpy.node import Node
# from rcl_interfaces.srv import SetParameters
# from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

# from PyQt5.QtWidgets import QApplication, QMainWindow
# from PyQt5.QtCore import pyqtSignal
# from PyQt5 import uic

# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from rclpy.action.client import GoalStatus

# import os
# import sys

# # 미리 지정된 위치 정보
# locations = {
#     "kitchen": [-2.5, 4.2],
#     "table1": [1.5, -3.5],
#     "table2": [0.4, -2.5],
#     "table3": [-5.5, -2.5],
#     "table4": [2.2, 2.1],
#     "table5": [0.3, 0.9],
#     "table6": [-5.5, 0.9]
# }

# # 한국어 출력을 위한 번역
# dictionary = {
#     "kitchen": "주방",
#     "table1": "1번 테이블",
#     "table2": "2번 테이블",
#     "table3": "3번 테이블",
#     "table4": "4번 테이블",
#     "table5": "5번 테이블",
#     "table6": "6번 테이블"
# }

# class GuiNode(Node):
#     def __init__(self, GUI):
#         super().__init__("gui_node")
#         self.GUI = GUI

#         # NavigateToPose 액션을 사용하는 Action Client 생성
#         self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
#         # SetParameters 서비스를 호출할 수 있는 Service Client 생성
#         self.set_yaw_goal_tolerance_client = self.create_client(SetParameters, "/controller_server/set_parameters")
#         # 초기 설정
#         self.set_yaw_goal_tolerance("general_goal_checker.yaw_goal_tolerance", 7.0)
#         self.position = [0.0, 0.0]  # 목표 위치 초기화

#     # 위치를 설정하고 NavigateToPose 액션 서버에 목표를 전송하는 함수
#     def navigate_to_pose(self, position):
#         self.position = position

#         # 액션 서버가 준비될 때까지 대기
#         wait_count = 1
#         while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
#             if wait_count > 3:
#                 message = "[WARN] Navigate action server is not available."
#                 self.GUI.update_signal.emit(message)
#                 return False
#             wait_count += 1

#         # 목표 메시지 생성 및 설정
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = "map"
#         goal_msg.pose.pose.position.x = self.position[0]
#         goal_msg.pose.pose.position.y = self.position[1]
#         goal_msg.pose.pose.orientation.w = 1.0

#         # 비동기적으로 목표 전송
#         self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg, feedback_callback=self.navigate_to_pose_action_feedback)
#         self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
#         return True

#     # 목표 전송 후 결과를 처리하는 콜백 함수
#     def navigate_to_pose_action_goal(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             message = "[WARN] Action goal rejected."
#             self.GUI.update_signal.emit(message)
#             return

#         # 각 위치에 따른 출발 메시지 출력
#         location_name = None
#         for name, pos in locations.items():
#             if pos == self.position:
#                 location_name = name
#                 break
#         if location_name is not None:
#             message = f"[INFO] {dictionary.get(location_name, location_name)}으로 이동합니다."
#         else:
#             message = "[WARN] 위치가 올바르게 설정되지 않았습니다."

#         self.GUI.update_signal.emit(message)

#         self.action_result_future = goal_handle.get_result_async()
#         self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)

#     def navigate_to_pose_action_feedback(self, feedback_msg):
#         """NavigateToPose 액션의 피드백 메시지를 로그로 기록"""
#         action_feedback = feedback_msg.feedback
#         self.get_logger().info("Action feedback: {0}".format(action_feedback))

#     def navigate_to_pose_action_result(self, future):
#         """NavigateToPose 액션 결과를 확인하고 성공 또는 실패 메시지를 GUI에 출력"""
#         action_status = future.result().status
#         if action_status == GoalStatus.STATUS_SUCCEEDED:
#             # 도착 메시지 출력
#             location_name = None
#             for name, pos in locations.items():
#                 if pos == self.position:
#                     location_name = name
#                     break
#             if location_name:
#                 message = f"[INFO] {dictionary.get(location_name, location_name)}으로 서빙 로봇이 도착하였습니다."
#             else:
#                 message = "[WARN] 위치 이름을 찾을 수 없습니다."
#         else:
#             message = f"[WARN] Action failed with status: {action_status}"

#         self.GUI.update_signal.emit(message)

#     # Yaw 목표 허용 오차 설정 서비스 호출
#     def set_yaw_goal_tolerance(self, parameter_name, value):
#         """Yaw goal tolerance 파라미터를 설정하기 위해 서비스 요청 전송"""
#         request = SetParameters.Request()
#         parameter = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
#         request.parameters = [Parameter(name=parameter_name, value=parameter)]
#         service_client = self.set_yaw_goal_tolerance_client
#         return self.call_service(service_client, request, "yaw_goal_tolerance parameter")

#     # 서비스 호출 처리 함수
#     def call_service(self, service_client, request, service_name):
#         """서비스 준비 상태를 확인하고 비동기적으로 호출하는 함수"""
#         wait_count = 1
#         while not service_client.wait_for_service(timeout_sec=0.1):
#             if wait_count > 3:
#                 message = f"[WARN] {service_name} service is not available"
#                 self.GUI.update_signal.emit(message)
#                 return False
#             wait_count += 1

#         message = f"[INIT] Set to have no yaw goal"
#         self.GUI.update_signal.emit(message)
#         service_client.call_async(request)
#         return True

# # PyInstaller를 사용해 UI 파일의 경로를 설정하는 함수
# def resource_path(relative_path):
#     base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
#     return os.path.join(base_path, relative_path)

# form = resource_path('control_robot.ui')
# form_class = uic.loadUiType(form)[0]


# class GUI(QMainWindow, form_class):
#     update_signal = pyqtSignal(str)  # GUI 업데이트 신호

#     def __init__(self):
#         super().__init__()
#         self.setupUi(self)

#         # 각 버튼에 on_button_click 함수를 연결
#         for name, position in locations.items():
#             button_name = f"{name.lower()}_button"
#             button = getattr(self, button_name, None)
#             if button:
#                 button.clicked.connect(lambda _, pos=position: self.on_button_click(pos))

#         self.update_signal.connect(self.update_display)

#         # ROS2 초기화 및 GuiNode 생성
#         rclpy.init()
#         self.node = GuiNode(self)
#         # ROS2 노드를 별도의 스레드에서 실행
#         self.thread_spin = threading.Thread(target=rclpy.spin, args=(self.node,))
#         self.thread_spin.start()

#     # 버튼 클릭 시 목표 위치로 이동 명령 전송
#     def on_button_click(self, position):
#         """버튼을 클릭하면 주어진 위치로 이동하는 명령을 전송"""
#         self.node.navigate_to_pose(position)

#     # 텍스트 영역에 메시지 업데이트
#     def update_display(self, message):
#         """GUI 텍스트 창에 메시지를 추가하여 표시"""
#         self.message_display.append(str(message))

# # 메인 함수: PyQt5 애플리케이션 실행
# def main():
#     app = QApplication(sys.argv)
#     window = GUI()
#     window.show()
#     sys.exit(app.exec_())

# # 스크립트가 직접 실행될 때 main() 함수 호출
# if __name__ == "__main__":
#     main()


import rclpy
import threading
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSignal
from PyQt5 import uic

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.action.client import GoalStatus

import os
import sys

# 미리 지정된 위치 정보
locations = {
    "kitchen": [-2.5, 4.2],
    "table1": [1.5, -3.5],
    "table2": [0.4, -2.5],
    "table3": [-5.5, -2.5],
    "table4": [2.2, 2.1],
    "table5": [0.3, 0.9],
    "table6": [-5.5, 0.9]
}

# 한국어 출력을 위한 번역
dictionary = {
    "kitchen": "주방",
    "table1": "1번 테이블",
    "table2": "2번 테이블",
    "table3": "3번 테이블",
    "table4": "4번 테이블",
    "table5": "5번 테이블",
    "table6": "6번 테이블"
}

class GuiNode(Node):
    def __init__(self, GUI):
        super().__init__("gui_node")
        self.GUI = GUI

        # NavigateToPose 액션을 사용하는 Action Client 생성
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # SetParameters 서비스를 호출할 수 있는 Service Client 생성
        self.set_yaw_goal_tolerance_client = self.create_client(SetParameters, "/controller_server/set_parameters")
        # 초기 설정
        self.set_yaw_goal_tolerance("general_goal_checker.yaw_goal_tolerance", 7.0)
        self.position = [0.0, 0.0]  # 목표 위치 초기화

    # 위치를 설정하고 NavigateToPose 액션 서버에 목표를 전송하는 함수
    def navigate_to_pose(self, position):
        self.position = position

        # 액션 서버가 준비될 때까지 대기
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                message = "[WARN] Navigate action server is not available."
                self.GUI.update_signal.emit(message)
                return False
            wait_count += 1

        # 목표 메시지 생성 및 설정
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.position[0]
        goal_msg.pose.pose.position.y = self.position[1]
        goal_msg.pose.pose.orientation.w = 1.0

        # 비동기적으로 목표 전송
        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg, feedback_callback=self.navigate_to_pose_action_feedback)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
        return True

    # 목표 전송 후 결과를 처리하는 콜백 함수
    def navigate_to_pose_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = "[WARN] Action goal rejected."
            self.GUI.update_signal.emit(message)
            return

        # 각 위치에 따른 출발 메시지 출력
        location_name = None
        for name, pos in locations.items():
            if pos == self.position:
                location_name = name
                break
        if location_name is not None:
            message = f"[INFO] {dictionary.get(location_name, location_name)}으로 이동합니다."
        else:
            message = "[WARN] 위치가 올바르게 설정되지 않았습니다."

        self.GUI.update_signal.emit(message)

        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)

    def navigate_to_pose_action_feedback(self, feedback_msg):
        """NavigateToPose 액션의 피드백 메시지를 로그로 기록"""
        action_feedback = feedback_msg.feedback
        self.get_logger().info("Action feedback: {0}".format(action_feedback))

    def navigate_to_pose_action_result(self, future):
        """NavigateToPose 액션 결과를 확인하고 성공 또는 실패 메시지를 GUI에 출력"""
        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            # 도착 메시지 출력
            location_name = None
            for name, pos in locations.items():
                if pos == self.position:
                    location_name = name
                    break
            if location_name:
                message = f"[INFO] {dictionary.get(location_name, location_name)}으로 서빙 로봇이 도착하였습니다."
            else:
                message = "[WARN] 위치 이름을 찾을 수 없습니다."
        else:
            message = f"[WARN] Action failed with status: {action_status}"

        self.GUI.update_signal.emit(message)

    # Yaw 목표 허용 오차 설정 서비스 호출
    def set_yaw_goal_tolerance(self, parameter_name, value):
        """Yaw goal tolerance 파라미터를 설정하기 위해 서비스 요청 전송"""
        request = SetParameters.Request()
        parameter = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        request.parameters = [Parameter(name=parameter_name, value=parameter)]
        service_client = self.set_yaw_goal_tolerance_client
        return self.call_service(service_client, request, "yaw_goal_tolerance parameter")

    # 서비스 호출 처리 함수
    def call_service(self, service_client, request, service_name):
        """서비스 준비 상태를 확인하고 비동기적으로 호출하는 함수"""
        wait_count = 1
        while not service_client.wait_for_service(timeout_sec=0.1):
            if wait_count > 3:
                message = f"[WARN] {service_name} service is not available"
                self.GUI.update_signal.emit(message)
                return False
            wait_count += 1

        message = f"[INIT] Set to have no yaw goal"
        self.GUI.update_signal.emit(message)
        service_client.call_async(request)
        return True

# UI 파일 경로 설정
form = os.path.join(os.path.dirname(__file__), 'control_robot.ui')
try:
    form_class = uic.loadUiType(form)[0]
except FileNotFoundError:
    print(f"[ERROR] UI file not found at: {form}")
    sys.exit(1)

class GUI(QMainWindow, form_class):
    update_signal = pyqtSignal(str)  # GUI 업데이트 신호

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 각 버튼에 on_button_click 함수를 연결
        for name, position in locations.items():
            button_name = f"{name.lower()}_button"
            button = getattr(self, button_name, None)
            if button:
                button.clicked.connect(lambda _, pos=position: self.on_button_click(pos))

        self.update_signal.connect(self.update_display)

        # ROS2 초기화 및 GuiNode 생성
        rclpy.init()
        self.node = GuiNode(self)
        # ROS2 노드를 별도의 스레드에서 실행
        self.thread_spin = threading.Thread(target=rclpy.spin, args=(self.node,))
        self.thread_spin.start()

    # 버튼 클릭 시 목표 위치로 이동 명령 전송
    def on_button_click(self, position):
        """버튼을 클릭하면 주어진 위치로 이동하는 명령을 전송"""
        self.node.navigate_to_pose(position)

    # 텍스트 영역에 메시지 업데이트
    def update_display(self, message):
        """GUI 텍스트 창에 메시지를 추가하여 표시"""
        self.message_display.append(str(message))

# 메인 함수: PyQt5 애플리케이션 실행
def main():
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())

# 스크립트가 직접 실행될 때 main() 함수 호출
if __name__ == "__main__":
    main()
