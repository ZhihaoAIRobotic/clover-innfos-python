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

        columns = list(range(1+self.arm.dof))

        self.headers = [QLabel(f'H{i}') for i in columns]
        self.headers[0].setText("Joint: ")
        for i,h in enumerate(self.headers[1:]):
            h.setText(f"{i+1}")

        self.joint_currents = [QLabel(f'J{i}') for i in columns]
        self.joint_currents[0].setText("Current :")

        self.joint_positions = [QLabel(f'J{i}') for i in columns]
        self.joint_positions[0].setText("Pos (Deg):")

        self.joint_velocities = [QLabel(f'J{i}') for i in columns]
        self.joint_velocities[0].setText("Vel (Deg/min):")
        
        # Make timer to update joint position
        self.timer=QTimer()
        self.timer.timeout.connect(self.showInfo)

        # Populate Qt Application window with label
        layout=QGridLayout()
        row = 0
        for i,h in enumerate(self.headers):
            layout.addWidget(h,row,i)
        row += 1

        for i,h in enumerate(self.joint_currents):
            layout.addWidget(h,row,i)
        row += 1

        for i,h in enumerate(self.joint_positions):
            layout.addWidget(h,row,i)
        row += 1

        for i,h in enumerate(self.joint_velocities):
            layout.addWidget(h,row,i)
        row += 1
            

        self.setLayout(layout)
        
        # Start the timer
        self.timer.start(10)
        self.velarray = None

    def showInfo(self):
        
        current = self.arm.getCurrents(bRefresh=True)
        for l,p in zip(self.joint_currents[1:], current):
            l.setText(f'{float(p):> 5.1f}')

    
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

    thread_args = [arm_instance]
    ui_th = threading.Thread(target=arm_sequence_thread, args=thread_args)
    ui_th.start()
    app.exec_()

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
        arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

        input("Move to zero")
        arm.home() # Will set position to profile mode

        arm.safePositionMode(max_vel=30*60,min_pos=-360,max_pos=+360)

        arm.setPositionMode()
        jidx = 0
        arm.setArmPosition([360,0,0,0,0,0])
        while(current:=abs(arm.getCurrents(True)[0]) < 1.0):
            lastpos = arm.getArmPosition()
        # Reset position mode (remove profile ramp and integrator windup) and go the other direction
        arm.setPositionMode()
        
        print('pos',lastpos)
        arm.setArmPosition([-360,0,0,0,0,0])
        time.sleep(2)
        pos_stop = lastpos[jidx]
        while(current:=abs(arm.getCurrents(True)[0]) < 1.0):
            lastpos = arm.getArmPosition()

        arm.setPositionMode()

        current=abs(arm.getCurrents(True)[0])
        while( current < 1.0):
            current=abs(arm.getCurrents(True)[0])
            lastpos = arm.getArmPosition()

        print('neg',lastpos)
        neg_stop = lastpos[jidx]
        print(pos_stop,neg_stop)
        print("going to",[(neg_stop+pos_stop)/2,0,0,0,0,0])

        arm.setArmPosition([(neg_stop+pos_stop)/2,0,0,0,0,0])




    StartDashboard(arm_sequence, arm)




    