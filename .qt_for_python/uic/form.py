# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/Users/cjc/gui-dev/SpaceCAL/src/jetsonDisplay/jetsonDisplay/form.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(400, 300)
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(0, 10, 211, 271))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.frame)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 191, 231))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout.addWidget(self.pushButton_2)
        self.frame_2 = QtWidgets.QFrame(Form)
        self.frame_2.setGeometry(QtCore.QRect(229, 9, 151, 271))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.formLayoutWidget = QtWidgets.QWidget(self.frame_2)
        self.formLayoutWidget.setGeometry(QtCore.QRect(10, 19, 131, 231))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.statusLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.statusLabel.setObjectName("statusLabel")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.statusLabel)
        self.statusLineEdit = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.statusLineEdit.setObjectName("statusLineEdit")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.statusLineEdit)
        self.typeLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.typeLabel.setObjectName("typeLabel")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.typeLabel)
        self.typeLineEdit = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.typeLineEdit.setObjectName("typeLineEdit")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.typeLineEdit)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.pushButton.setText(_translate("Form", "PushButton"))
        self.pushButton_2.setText(_translate("Form", "PushButton"))
        self.statusLabel.setText(_translate("Form", "status"))
        self.typeLabel.setText(_translate("Form", "type"))
