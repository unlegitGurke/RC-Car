# Form implementation generated from reading ui file 'pds.ui'
#
# Created by: PyQt6 UI code generator 6.4.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(716, 726)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.front_1 = QtWidgets.QLabel(self.centralwidget)
        self.front_1.setGeometry(QtCore.QRect(410, 200, 31, 211))
        self.front_1.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.front_1.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.front_1.setLineWidth(2)
        self.front_1.setMidLineWidth(0)
        self.front_1.setText("")
        self.front_1.setObjectName("front_1")
        self.front_2 = QtWidgets.QLabel(self.centralwidget)
        self.front_2.setGeometry(QtCore.QRect(450, 200, 31, 211))
        self.front_2.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.front_2.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.front_2.setLineWidth(2)
        self.front_2.setMidLineWidth(0)
        self.front_2.setText("")
        self.front_2.setObjectName("front_2")
        self.front_3 = QtWidgets.QLabel(self.centralwidget)
        self.front_3.setGeometry(QtCore.QRect(530, 200, 31, 211))
        self.front_3.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.front_3.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.front_3.setLineWidth(2)
        self.front_3.setMidLineWidth(0)
        self.front_3.setText("")
        self.front_3.setObjectName("front_3")
        self.front_4 = QtWidgets.QLabel(self.centralwidget)
        self.front_4.setGeometry(QtCore.QRect(490, 200, 31, 211))
        self.front_4.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.front_4.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.front_4.setLineWidth(2)
        self.front_4.setMidLineWidth(0)
        self.front_4.setText("")
        self.front_4.setObjectName("front_4")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 716, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())
