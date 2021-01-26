# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/interface.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
import vue.container as c


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(830, 606)
        MainWindow.setMinimumSize(QtCore.QSize(830, 606))
        MainWindow.setMaximumSize(QtCore.QSize(830, 606))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(-1, -1, 851, 571))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.frame.setFont(font)
        self.frame.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.frame.setStyleSheet("background: #111")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.btn_mvt = QtWidgets.QPushButton(self.frame)
        self.btn_mvt.setGeometry(QtCore.QRect(60, 440, 121, 31))
        font = QtGui.QFont()
        font.setFamily("Righteous")
        font.setPointSize(11)
        self.btn_mvt.setFont(font)
        self.btn_mvt.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.btn_mvt.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.btn_mvt.setStyleSheet("background-Color: rgb(52, 50, 200); color: white;border-radius: 10px;")
        self.btn_mvt.setObjectName("btn_mvt")
        self.btn_reprendre = QtWidgets.QPushButton(self.frame)
        self.btn_reprendre.setGeometry(QtCore.QRect(210, 440, 101, 31))
        font = QtGui.QFont()
        font.setFamily("Righteous")
        font.setPointSize(9)
        self.btn_reprendre.setFont(font)
        self.btn_reprendre.setStyleSheet("background-Color:white ; color: black;border-radius: 10px;")
        self.btn_reprendre.setObjectName("btn_reprendre")
        self.btn_quitter = QtWidgets.QPushButton(self.frame)
        self.btn_quitter.setGeometry(QtCore.QRect(350, 440, 121, 31))
        font = QtGui.QFont()
        font.setFamily("Righteous")
        font.setPointSize(9)
        self.btn_quitter.setFont(font)
        self.btn_quitter.setStyleSheet("background: rgb(187, 11, 11); color: white;border-radius: 10px;")
        self.btn_quitter.setObjectName("btn_quitter")
        self.val_nbrpas = QtWidgets.QSpinBox(self.frame)
        self.val_nbrpas.setGeometry(QtCore.QRect(650, 440, 41, 21))
        self.val_nbrpas.setStyleSheet("background: white;border-Style: none;border-radius: 5px;")
        self.val_nbrpas.setObjectName("val_nbrpas")
        self.label_18 = QtWidgets.QLabel(self.frame)
        self.label_18.setGeometry(QtCore.QRect(520, 440, 121, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_18.setFont(font)
        self.label_18.setStyleSheet("color:white")
        self.label_18.setObjectName("label_18")
        self.widget = QtWidgets.QWidget(self.frame)
        self.widget.setGeometry(QtCore.QRect(519, 89, 291, 341))
        self.widget.setCursor(QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.widget.setStyleSheet("background: white; border-radius:10px;")
        self.widget.setObjectName("widget")
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setGeometry(QtCore.QRect(10, 30, 201, 21))
        font = QtGui.QFont()
        font.setFamily("Candara")
        font.setPointSize(18)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.line_2 = QtWidgets.QFrame(self.widget)
        self.line_2.setGeometry(QtCore.QRect(10, 160, 271, 6))
        self.line_2.setStyleSheet("background: rgb(90,40,200)")
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.line_3 = QtWidgets.QFrame(self.widget)
        self.line_3.setGeometry(QtCore.QRect(10, 260, 271, 6))
        self.line_3.setStyleSheet("background: rgb(90,40,200)")
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.label_6 = QtWidgets.QLabel(self.widget)
        self.label_6.setGeometry(QtCore.QRect(140, 90, 21, 16))
        self.label_6.setObjectName("label_6")
        self.label_3 = QtWidgets.QLabel(self.widget)
        self.label_3.setGeometry(QtCore.QRect(10, 130, 201, 21))
        font = QtGui.QFont()
        font.setFamily("Candara")
        font.setPointSize(18)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.line = QtWidgets.QFrame(self.widget)
        self.line.setGeometry(QtCore.QRect(10, 60, 271, 6))
        self.line.setStyleSheet("background: rgb(90,40,200);")
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.label_14 = QtWidgets.QLabel(self.widget)
        self.label_14.setGeometry(QtCore.QRect(60, 280, 21, 16))
        self.label_14.setObjectName("label_14")
        self.label_5 = QtWidgets.QLabel(self.widget)
        self.label_5.setGeometry(QtCore.QRect(60, 90, 21, 16))
        self.label_5.setObjectName("label_5")
        self.label_4 = QtWidgets.QLabel(self.widget)
        self.label_4.setGeometry(QtCore.QRect(10, 230, 231, 21))
        font = QtGui.QFont()
        font.setFamily("Candara")
        font.setPointSize(18)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.label_7 = QtWidgets.QLabel(self.widget)
        self.label_7.setGeometry(QtCore.QRect(220, 90, 21, 16))
        self.label_7.setObjectName("label_7")
        self.label_11 = QtWidgets.QLabel(self.widget)
        self.label_11.setGeometry(QtCore.QRect(60, 190, 21, 16))
        self.label_11.setObjectName("label_11")
        self.label_16 = QtWidgets.QLabel(self.widget)
        self.label_16.setGeometry(QtCore.QRect(140, 190, 21, 16))
        self.label_16.setObjectName("label_16")
        self.label_20 = QtWidgets.QLabel(self.widget)
        self.label_20.setGeometry(QtCore.QRect(140, 280, 21, 16))
        self.label_20.setObjectName("label_20")
        self.val_xb = QtWidgets.QTextEdit(self.widget)
        self.val_xb.setGeometry(QtCore.QRect(10, 280, 41, 21))
        self.val_xb.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_xb.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_xb.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_xb.setObjectName("val_xb")
        self.val_yb = QtWidgets.QTextEdit(self.widget)
        self.val_yb.setGeometry(QtCore.QRect(90, 280, 41, 21))
        self.val_yb.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_yb.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_yb.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_yb.setObjectName("val_yb")
        self.val_theta2 = QtWidgets.QTextEdit(self.widget)
        self.val_theta2.setGeometry(QtCore.QRect(90, 190, 41, 21))
        self.val_theta2.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_theta2.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_theta2.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_theta2.setObjectName("val_theta2")
        self.val_theta1 = QtWidgets.QTextEdit(self.widget)
        self.val_theta1.setGeometry(QtCore.QRect(10, 190, 41, 21))
        self.val_theta1.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_theta1.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_theta1.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_theta1.setObjectName("val_theta1")
        self.val_l1 = QtWidgets.QTextEdit(self.widget)
        self.val_l1.setGeometry(QtCore.QRect(10, 90, 41, 21))
        self.val_l1.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_l1.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_l1.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_l1.setObjectName("val_l1")
        self.val_l2 = QtWidgets.QTextEdit(self.widget)
        self.val_l2.setGeometry(QtCore.QRect(90, 90, 41, 21))
        self.val_l2.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_l2.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_l2.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_l2.setObjectName("val_l2")
        self.val_l3 = QtWidgets.QTextEdit(self.widget)
        self.val_l3.setGeometry(QtCore.QRect(170, 90, 41, 21))
        self.val_l3.setStyleSheet("border-Color: white;border-style: solid;border-width: 3px;background: #f0f0f0;")
        self.val_l3.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_l3.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.val_l3.setObjectName("val_l3")
        self.Container = c.Container(self.frame)
        self.Container.setGeometry(QtCore.QRect(29, 89, 471, 341))
        self.Container.setCursor(QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.Container.setStyleSheet("background: white;box-shadow: 5px 10px;border-radius: 10px")
        self.Container.setObjectName("Container")
        self.label_8 = QtWidgets.QLabel(self.Container)
        self.label_8.setGeometry(QtCore.QRect(30, 280, 31, 16))
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.Container)
        self.label_9.setGeometry(QtCore.QRect(410, 20, 31, 21))
        self.label_9.setObjectName("label_9")
        self.label_9.raise_()
        self.label_8.raise_()
        self.afficheur = QtWidgets.QLabel(self.frame)
        self.afficheur.setGeometry(QtCore.QRect(36, 20, 771, 51))
        font = QtGui.QFont()
        font.setFamily("Century")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.afficheur.setFont(font)
        self.afficheur.setCursor(QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.afficheur.setStyleSheet("background: white;box-shadow: 5px 10px;border-radius: 10px")
        self.afficheur.setAlignment(QtCore.Qt.AlignCenter)
        self.afficheur.setObjectName("afficheur")
        self.afficheur_pk = QtWidgets.QLabel(self.frame)
        self.afficheur_pk.setGeometry(QtCore.QRect(40, 480, 461, 31))
        font = QtGui.QFont()
        font.setFamily("Righteous")
        font.setPointSize(9)
        self.afficheur_pk.setFont(font)
        self.afficheur_pk.setCursor(QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.afficheur_pk.setStyleSheet("background: white; border-radius:10px;")
        self.afficheur_pk.setAlignment(QtCore.Qt.AlignCenter)
        self.afficheur_pk.setObjectName("afficheur_pk")
        self.btn_mvt_2 = QtWidgets.QPushButton(self.frame)
        self.btn_mvt_2.setGeometry(QtCore.QRect(700, 440, 91, 31))
        font = QtGui.QFont()
        font.setFamily("Righteous")
        font.setPointSize(11)
        self.btn_mvt_2.setFont(font)
        self.btn_mvt_2.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.btn_mvt_2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.btn_mvt_2.setStyleSheet("background-Color: rgb(52, 20, 250); color: white;border-radius: 10px;")
        self.btn_mvt_2.setObjectName("btn_mvt_2")
        self.widget.raise_()
        self.btn_mvt.raise_()
        self.btn_reprendre.raise_()
        self.btn_quitter.raise_()
        self.val_nbrpas.raise_()
        self.label_18.raise_()
        self.Container.raise_()
        self.afficheur.raise_()
        self.afficheur_pk.raise_()
        self.btn_mvt_2.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 830, 21))
        self.menubar.setStyleSheet("background-Color: rgb(90,40,200);color: white")
        self.menubar.setObjectName("menubar")
        self.menuFichier = QtWidgets.QMenu(self.menubar)
        self.menuFichier.setObjectName("menuFichier")
        self.menuAide = QtWidgets.QMenu(self.menubar)
        self.menuAide.setObjectName("menuAide")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionOuvrir = QtWidgets.QAction(MainWindow)
        self.actionOuvrir.setObjectName("actionOuvrir")
        self.actionSauvegarder = QtWidgets.QAction(MainWindow)
        self.actionSauvegarder.setObjectName("actionSauvegarder")
        self.actionSauvegarder_2 = QtWidgets.QAction(MainWindow)
        self.actionSauvegarder_2.setObjectName("actionSauvegarder_2")
        self.actionQuitter = QtWidgets.QAction(MainWindow)
        self.actionQuitter.setObjectName("actionQuitter")
        self.actionAbout = QtWidgets.QAction(MainWindow)
        self.actionAbout.setObjectName("actionAbout")
        self.actionSauvegarder_3 = QtWidgets.QAction(MainWindow)
        self.actionSauvegarder_3.setObjectName("actionSauvegarder_3")
        self.actionSave = QtWidgets.QAction(MainWindow)
        self.actionSave.setObjectName("actionSave")
        self.actionSettings = QtWidgets.QAction(MainWindow)
        self.actionSettings.setObjectName("actionSettings")
        self.menuFichier.addSeparator()
        self.menuFichier.addAction(self.actionQuitter)
        self.menuAide.addAction(self.actionAbout)
        self.menubar.addAction(self.menuFichier.menuAction())
        self.menubar.addAction(self.menuAide.menuAction())

        self.retranslateUi(MainWindow)
        self.val_nbrpas.editingFinished.connect(self.btn_mvt.click)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.btn_mvt.setText(_translate("MainWindow", "Démarrer"))
        self.btn_reprendre.setText(_translate("MainWindow", "Réinitialiser"))
        self.btn_quitter.setText(_translate("MainWindow", "Quitter"))
        self.label_18.setText(_translate("MainWindow", "Nombre de pas"))
        self.label_2.setText(_translate("MainWindow", "Valeurs des liens"))
        self.label_6.setText(_translate("MainWindow", "L2"))
        self.label_3.setText(_translate("MainWindow", "Valeurs des angles"))
        self.label_14.setText(_translate("MainWindow", "Xb"))
        self.label_5.setText(_translate("MainWindow", "L1"))
        self.label_4.setText(_translate("MainWindow", "Coordonnées B(x0, y0)"))
        self.label_7.setText(_translate("MainWindow", "L3"))
        self.label_11.setText(_translate("MainWindow", "θ1"))
        self.label_16.setText(_translate("MainWindow", "θ2"))
        self.label_20.setText(_translate("MainWindow", "Yb"))
        self.label_8.setText(_translate("MainWindow", "Axe y"))
        self.label_9.setText(_translate("MainWindow", "Axe x"))
        self.afficheur.setText(_translate("MainWindow", "Bot Simulator"))
        self.afficheur_pk.setAccessibleName(_translate("MainWindow", "Pk"))
        self.afficheur_pk.setAccessibleDescription(_translate("MainWindow", "Coordonnees Pk"))
        self.afficheur_pk.setText(_translate("MainWindow", "Point k"))
        self.btn_mvt_2.setText(_translate("MainWindow", "Valider"))
        self.menuFichier.setTitle(_translate("MainWindow", "Fichier"))
        self.menuAide.setTitle(_translate("MainWindow", "Aide"))
        self.actionOuvrir.setText(_translate("MainWindow", "Ouvrir"))
        self.actionOuvrir.setShortcut(_translate("MainWindow", "Ctrl+O"))
        self.actionSauvegarder.setText(_translate("MainWindow", "Sauvegarder"))
        self.actionSauvegarder_2.setText(_translate("MainWindow", "Sauvegarder"))
        self.actionSauvegarder_2.setShortcut(_translate("MainWindow", "Ctrl+Shift+S"))
        self.actionQuitter.setText(_translate("MainWindow", "Quitter"))
        self.actionQuitter.setShortcut(_translate("MainWindow", "Ctrl+Q"))
        self.actionAbout.setText(_translate("MainWindow", "About"))
        self.actionSauvegarder_3.setText(_translate("MainWindow", "Sauvegarder"))
        self.actionSave.setText(_translate("MainWindow", "Save"))
        self.actionSave.setShortcut(_translate("MainWindow", "Ctrl+Shift+S"))
        self.actionSettings.setText(_translate("MainWindow", "Settings"))
        self.actionSettings.setShortcut(_translate("MainWindow", "Ctrl+Shift+R"))