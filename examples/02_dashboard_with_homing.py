import time
import sys

import numpy as np
from scipy import signal
from clover_innfos_python import *

from PyQt5.QtWidgets import QWidget,QTextEdit,QApplication,QListWidget,QGridLayout,QLabel,QVBoxLayout,QStatusBar
from PyQt5.QtCore import QTimer,QDateTime
from PyQt5.QtGui import QKeySequence
import threading



class WinForm(QWidget):
    def __init__(self,parent=None,arm=None):
        super(WinForm, self).__init__(parent)
        self.setObjectName("mainDialog")
        self.setWindowTitle('Joint Display example')
        self.arm = arm
        self.arm.qt = self
        

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

    ## Handle CTRL+C with Qt ##
    def quit(self):
        QApplication.instance().quit()

    def keyPressEvent(self, event):
        ## Handle CTRL+C with Qt ##
        if QKeySequence(event.key()+int(event.modifiers())) == QKeySequence("Ctrl+C"):
            self.quit()

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

    def thread_wrapper(*args):
        try:
            arm_sequence_thread(*args)
        finally:
            print("Arm sequence thread is done")
            

    thread_args = [arm_instance]
    ui_th = threading.Thread(target=thread_wrapper, args=thread_args)
    ## Handle CTRL+C with Qt ##
    ui_th.daemon = True # Thread will be killed on exit() of Qt
    ui_th.start()
    app.exec_()
    
    sys.exit(app.quit() )


if __name__ == '__main__':

    np.set_printoptions(precision=4,floatmode='fixed',suppress=True)
    actuator_ids =  [1,2,4,7,6] # J1=id(1) ... J5=id(6)
    arm = ArmInterface()

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
        trigger_current = 0.5
        search_limit = 360
        target = np.zeros(arm.dof)

        def find_limits(arm, jidx, search_limit, trigger_current):
            target[jidx] = search_limit
            print("Moving to ",target)
            arm.setArmPosition(target)
            while(current:=abs(arm.getCurrents(True)[jidx]) < trigger_current):
                lastpos = arm.getArmPosition()
            # Reset position mode (remove profile ramp and integrator windup) and go the other direction
            arm.setPositionMode()
            
            print('positive stop position:',lastpos)
            target[jidx] = -search_limit
            arm.setArmPosition(target)

            time.sleep(2) # Sleep a little to wait for direction change

            pos_stop = lastpos[jidx]
            while(current:=abs(arm.getCurrents(True)[jidx]) < trigger_current):
                lastpos = arm.getArmPosition()
            arm.setPositionMode()
            print(arm.getArmPosition())

            print('negative stop position:',lastpos)
            neg_stop = lastpos[jidx]
            
            
            target[jidx] = (neg_stop+pos_stop)/2
            arm.setArmPosition(target)

            # Wait until we get to our target
            while((pos:=np.abs(arm.getArmPosition()-A(target)) > 0.1).any()):
                pass
                

            arm.home() # Will set position to profile mode
            return (neg_stop, pos_stop)

        j0_limit = find_limits(arm, 0, 360, 0.5)
        print("j0 limit was",j0_limit)


    StartDashboard(arm_sequence, arm)




    