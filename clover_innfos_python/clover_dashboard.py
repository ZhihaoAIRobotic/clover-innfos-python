import time
import sys
from PyQt5.QtWidgets import QWidget,QTextEdit,QApplication,QListWidget,QGridLayout,QLabel,QVBoxLayout,QStatusBar
from PyQt5.QtCore import QTimer,QDateTime
from PyQt5.QtGui import QKeySequence
import threading



class CloverDashboard(QWidget):
    def __init__(self,parent=None,arm=None):
        super(CloverDashboard, self).__init__(parent)
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
        
        # get joint currents
        current = self.arm.getCurrents(bRefresh=True)
        # Display currents
        for qlabel, value in zip(self.joint_currents[1:], current):
            qlabel.setText(f'{float(value):> 5.1f}')

        # get and display joint positions
        pos = self.arm.getPositions()/Degrees
        for l,p in zip(self.joint_positions[1:], pos):
            l.setText(f'{float(p):> 5.1f}')


        # get and display joint velocities
        vel = self.arm.getVelocitys(bRefresh=True)/(Degrees/minute)
        
        # Moving average
        if self.velarray is None:
            self.velarray = np.zeros((10,len(vel)))
        self.velarray = np.vstack([self.velarray,vel])[-10:,:]
        vel = np.mean(self.velarray,0)

        for l,p in zip(self.joint_velocities[1:], vel):
            l.setText(f'{float(p):> 5.0f}')


def run_with_dashboard(arm_sequence_thread, arm_instance, *args):
    app = QApplication(sys.argv)
    form=CloverDashboard(arm=arm_instance)
    form.show()

    def thread_wrapper(*args):
        try:
            arm_sequence_thread(*args)
        finally:
            print("Arm sequence thread is done")
            arm_instance.end_operation()

    thread_args = [arm_instance] + args
    ui_th = threading.Thread(target=thread_wrapper, args=thread_args)
    ## Handle CTRL+C with Qt ##
    ui_th.daemon = True # Thread will be killed on exit() of Qt
    ui_th.start()
    app.exec_()
    
    sys.exit(app.quit() )


if __name__ == '__main__':
        
    import numpy as np
    from scipy import signal
    from clover_innfos_python import *

    np.set_printoptions(precision=4,floatmode='fixed',suppress=True)
    actuator_ids =  [1,2,4,7,6] # J1=id(1) ... J5=id(6)
    arm = ArmInterface(actuator_ids)

    def arm_sequence(*args):
        """ Sequence of operations on the arm to run """
        arm = args[0]
        
        arm.enableAllActuators()
        endtime = time.time()
        print("It took ",endtime-start," seconds to enable the arm")
        arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

        input("Move to zero")
        arm.home() # Will set position to profile mode

        


    StartDashboard(arm_sequence, arm)
