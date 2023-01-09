from pyPS4Controller.controller import Controller
import time
import numpy as np
from clover_innfos_python import *




import sys
from PyQt5.QtWidgets import QWidget,QTextEdit,QApplication,QListWidget,QGridLayout,QLabel,QVBoxLayout,QStatusBar
from PyQt5.QtCore import QTimer,QDateTime

class WinForm(QWidget):
    def __init__(self,parent=None,arm=None):
        super(WinForm, self).__init__(parent)
        self.setWindowTitle('Joint Display example')
        self.arm = arm
        self.Nlabels = 3

        self.label=QLabel('Jpos')
        self.modelabel=QLabel('mode')
        self.labels = [QLabel('Label_i') for i in range(self.Nlabels )]
        self.status = QStatusBar()

        # Make timer to update joint position
        self.timer=QTimer()
        self.timer.timeout.connect(self.showInfo)

        # Populate Qt Application window with label
        layout=QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.modelabel)
        for i in range(self.Nlabels):
            layout.addWidget(self.labels[i])
        self.setLayout(layout)
        
        # Start the timer
        self.timer.start(10)

    def showInfo(self):
        self.label.setText( f"Joint positions: {self.arm.getPositions()}")
        self.modelabel.setText( f"Joint positions: {[mode.name for mode in arm.getActuatorModes()]}")
        self.labels[0].setText(f"Joint offsets: {self.arm.getPositionOffsets(True)}")


import threading


def GUI(arm_sequence_thread, arm_instance):
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
    arm = Clover_GLUON()

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


        arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

    GUI(arm_sequence, arm)




    