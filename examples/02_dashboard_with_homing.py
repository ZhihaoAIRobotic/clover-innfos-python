import time
import sys

import numpy as np
from scipy import signal
from clover_innfos_python import *

from PyQt5.QtWidgets import QWidget,QTextEdit,QApplication,QListWidget,QGridLayout,QLabel,QVBoxLayout,QStatusBar
from PyQt5.QtCore import QTimer,QDateTime
import threading



class WinForm(QWidget):
    def __init__(self,parent=None,arm=None):
        super(WinForm, self).__init__(parent)
        self.setWindowTitle('Joint Display example')
        self.arm = arm
        self.Nlabels = 3

        indexes = list(range(1+self.arm.dof))
        self.headers = [QLabel(f'H{i}') for i in indexes]
        self.headers[0].setText("Joint: ")
        for i,h in enumerate(self.headers[1:]):
            h.setText(f"{i+1}")

        self.joint_positions = [QLabel(f'J{i}') for i in indexes]
        self.joint_positions[0].setText("Pos (Deg):")

        self.joint_velocities = [QLabel(f'J{i}') for i in indexes]
        self.joint_velocities[0].setText("Vel (Deg/min):")
        
        # Make timer to update joint position
        self.timer=QTimer()
        self.timer.timeout.connect(self.showInfo)

        # Populate Qt Application window with label
        layout=QGridLayout()
        for i,h in enumerate(self.headers):
            layout.addWidget(h,0,i)

        for i,h in enumerate(self.joint_positions):
            layout.addWidget(h,1,i)

        for i,h in enumerate(self.joint_velocities):
            layout.addWidget(h,2,i)

        self.setLayout(layout)
        
        # Start the timer
        self.timer.start(10)
        self.velarray = None

    def showInfo(self):
        pos = self.arm.getPositions()/Degrees
        for l,p in zip(self.joint_positions[1:], pos):
            l.setText(f'{float(p):> 5.1f}')


        vel = self.arm.getVelocitys(bRefresh=True)/(Degrees/minute)

        # Moving average
        if self.velarray is None:
            self.velarray = np.zeros((10,len(vel)))
        self.velarray = np.vstack([self.velarray,vel])[-10:,:]
        vel = np.mean(self.velarray,0)

        for l,p in zip(self.joint_velocities[1:], vel):
            l.setText(f'{float(p):> 5.0f}')


def StartDashboard(arm_sequence_thread, arm_instance):
    app = QApplication(sys.argv)
    form=WinForm(arm=arm_instance)
    form.show()

    try:
        thread_args = [arm_instance]
        ui_th = threading.Thread(target=arm_sequence_thread, args=thread_args)
        ui_th.start()
        app.exec_()

    finally: # Make sure we disable the arm
        arm_instance.disableAllActuators()
        
    sys.exit(app.quit() )


if __name__ == '__main__':
    np.set_printoptions(precision=4,floatmode='fixed',suppress=True)
    actuator_ids =  [1,2,4,7,6] # J1=id(1) ... J5=id(6)
    arm = ArmInterface(actuator_ids)

    def arm_sequence(*args):
        """ Sequence of operations on the arm to run """
        arm = args[0]

        print("Enabling arm...")
        start = time.time()
        arm.enableAllActuators()
        endtime = time.time()
        print("It took ",endtime-start," seconds to enable the arm")

        input("Move to zero")
        arm.home() # Will set position to profile mode

        arm.setPositionMode()
        arm.setArmPosition([10,10,10,10,10,10])
        time.sleep(5)
        arm.setArmPosition([0,0,0,0,0,0])


    StartDashboard(arm_sequence, arm)




    