# -*- coding: utf-8 -*-
"""
Created on Mon Jan 25 11:10:33 2021

@author: NANOk
"""
import sys
import threading
import time

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QFileDialog, QMessageBox 
from PyQt5.QtCore import pyqtSlot
from PyQt5 import QtGui

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

from vue.container import Container
import vue.MainInterface as MainView

import Bot

import numpy as np
import math

class Robot(QMainWindow, MainView.Ui_MainWindow):

    TIME_RATE = 1

    def __init__(self,parent=None):
        
        super(Robot,self).__init__(parent)

        self.bot = Bot.Bot()
        self.bot.init()

        self.setupUi(self)
        #self.setWindowFlag(Qt.FramelessWindowHint)
        self.setWindowIcon(QtGui.QIcon('images/robot.png'))
        self.connectActions()
        self.addToolBar(NavigationToolbar(self.Container.canvas, self))
        self.afficheur_pk.setVisible(False)

        self.equation = ""
        self.nb_pas = 0
        self.pas = 0

    def connectActions(self):

        #Set default values to fields

        self.val_l1.setPlainText("3.5")
        self.val_l2.setPlainText("3")
        self.val_l3.setPlainText("3")

        self.val_theta1.setPlainText("55")
        self.val_theta2.setPlainText("75")
        
        self.val_xb.setPlainText("0")
        self.val_yb.setPlainText("1")

        # On connecte les signaux au fonction qui les traites
        self.actionQuitter.triggered.connect(self.close)
        self.btn_quitter.clicked.connect(self.close)
        #self.actionAbout.triggered.connect(self.affiche_aide)
        self.btn_reprendre.clicked.connect(self.robot_init)
        self.btn_mvt.clicked.connect(self.update_graph)
        self.btn_mvt_2.clicked.connect(self.robot_init)
        #Tracer du sol
        self.sol()
        #Tracer robot initial
        self.robot_init()


    def main(self):
        self.show()

    @pyqtSlot()								
    def	closeEvent(self,event):
        event.accept()	
    
    @pyqtSlot()
    def affiche_aide(self):
        aide = HelpView.Ui_MainWindow()
        aide.setupUi(self)

    @pyqtSlot()
    def affiche_accueil(self):
        self.setupUi(self)
        self.connectActions()

    @pyqtSlot()
    def move(self):

        #check if user enter a step value
        self.nb_pas = int(self.val_nbrpas.value())
        if (self.nb_pas <= 0):
            self.Affiche_moi("Erreur: nombre de pas nul", "e")
            return False

        #initialize BH params
        self.bot.init([
            float(self.val_l1.toPlainText()), float(self.val_l2.toPlainText()), float(self.val_l3.toPlainText())
        ], [
            0, float(self.val_theta1.toPlainText()), float(self.val_theta2.toPlainText())
        ])
        
        #Get A and B position in R0
        point_a = self.bot.getPositionA()
        point_b = [float(self.val_xb.toPlainText()), float(self.val_yb.toPlainText()), 0, 1]

        #if the distance between point a and point b is more than the sum of the two last link then the point b is out of range
        if (math.sqrt((point_b[0] - point_a[0]) * (point_b[0] - point_a[0]) + (point_b[1] - point_a[1]) * (point_b[1] - point_a[1])) > float(self.val_l2.toPlainText()) + float(self.val_l3.toPlainText())):
            self.Affiche_moi("Erreur: Le poin B est hors d'atteinte", "e")
            return False

        #Get path equation parameters
        self.bot.eqTcoord(point_a, point_b)
        self.equation = '(D):Y='+ str(round(self.bot.ta,4))+'x'+ '+'+ str(round(self.bot.tb,4))

        #Get x axis step
        self.pas = self.bot.getStep(point_a, point_b, self.nb_pas)

        #Display equation found
        self.Affiche_moi(self.equation)

        self.afficheur_pk.setVisible(True)
        time.sleep(self.TIME_RATE)
        
        #For each step
        for i in range(self.nb_pas + 1):

            #Get step target position in R0
            dest_x = point_a[0] + (i * self.pas)
            dest = np.array([dest_x, self.bot.eqT(dest_x), 0, 1])

            self.afficheur_pk.setText("P{} = ({}, {})".format(i, dest[0], dest[1]))
            print("P{} = ({}, {})".format(i, dest[0], dest[1]))

            #Use Paul method to find angles
            self.bot.mgi(dest)            

            #Display angles got
            self.Affiche_moi("Angles en radiant : θ1 = {}, θ2 = {}".format(math.degrees(self.bot.params[0][self.bot.THETA]), math.degrees(self.bot.params[1][self.bot.THETA])))
            
            #Reset displaying and draw new bot positions
            self.remise_zero()
            self.Container.canvas.axes.plot([point_a[1], point_b[1]], [point_a[0], point_b[0]], color = 'g',linestyle='--')

            self.drawBot()
            
            self.Container.canvas.draw()

            #Sleep for TIME_RATE
            time.sleep(self.TIME_RATE)

            self.remise_zero()


        return False

    
    @pyqtSlot()
    def update_graph(self):
        threading.Thread(None,target=self.move).start()

    @pyqtSlot()
    def drawBot(self):
        #New position of R2 origin
        x1 = self.bot.params[1][self.bot.D] * math.cos(self.bot.params[0][self.bot.THETA]) + self.bot.params[0][self.bot.D]
        y1 = self.bot.params[1][self.bot.D] * math.sin(self.bot.params[0][self.bot.THETA])

        #New position of terminal organe
        angle = self.bot.params[0][self.bot.THETA] + self.bot.params[1][self.bot.THETA]
        x2 = self.bot.l * math.cos(angle) + x1
        y2 = self.bot.l * math.sin(angle) + y1

        self.lien(0, 0, self.bot.params[0][self.bot.D], 0) #Afficher lien L1
        self.pince(x1, y1, x2, y2)    #pince
        self.lien(self.bot.params[0][self.bot.D], 0, x1, y1)    #Afficher lien L2

    
    @pyqtSlot()
    def robot_init(self):

        self.afficheur_pk.setVisible(False)

        self.bot.init([
            float(self.val_l1.toPlainText()), float(self.val_l2.toPlainText()), float(self.val_l3.toPlainText())
        ], [
            0, float(self.val_theta1.toPlainText()), float(self.val_theta2.toPlainText())
        ])

        self.origin_robot(0, 0)
        
        self.drawBot()

        self.b(float(self.val_xb.toPlainText()),float(self.val_yb.toPlainText()))
        self.Container.canvas.draw()
    
    @pyqtSlot()
    def articulation(self, x_p, y_p):
        if x_p != 0 or y_p != 0:
            self.Container.canvas.axes.plot((y_p), (x_p), marker="h",markersize=20, color='orange')
        else:
            print("Le point ({}, {}), ne peut etre une articulation".format(x_p, y_p))
    
    @pyqtSlot()
    def pince(self, x_1 , y_1, x_2, y_2):
        X = [x_1, x_2]
        Y = [y_1, y_2]
        self.articulation(x_1, y_1)
        if (y_2 == 0 or x_2 == 0):
            self.Container.canvas.axes.plot((y_2), (x_2), marker='$∩$' ,markersize=20, color='black')
        else:
            self.Container.canvas.axes.plot((y_2 + 0.1), (x_2 - 0.3), marker='$∩$' ,markersize=20, color='black')
        self.Container.canvas.axes.plot(Y, X,linestyle='-', linewidth=6, color='blue')
        self.Container.canvas.axes.plot((y_2), (x_2), marker="o",markersize=10, color='black')
    
    @pyqtSlot()
    def lien(self, x_1 , y_1, x_2, y_2):
        X = [x_1, x_2]
        Y = [y_1, y_2]
        self.articulation(x_1, y_1)
        self.articulation(x_2, y_2)
        self.Container.canvas.axes.plot(Y, X,linestyle='-', linewidth=6, color='blue')
    
    @pyqtSlot()
    def origin_robot(self, x_o, y_o):
        Y = [y_o - 0.5, y_o, y_o + 0.5]
        X = [x_o, x_o,x_o]
        self.Container.canvas.axes.plot(Y, X,linestyle='-', linewidth=5, color='red',label="O")
    
    @pyqtSlot()
    def sol(self):
        Y = list(range(-1,9))
        X = [x*0 for x in Y]
        self.Container.canvas.axes.plot(Y, X,linestyle='--', linewidth=3, color='green',label="Sol")
        self.Container.canvas.axes.legend('Sol',loc='upper left')
    
    @pyqtSlot()
    def b(self, x, y):
        self.Container.canvas.axes.plot((y), (x), marker='o',markersize=12, color='y')
    
    @pyqtSlot()
    def remise_zero(self, avec_b = 0):
        self.Container.canvas.axes.clear()
        self.Container.canvas.axes.grid(which='both', linestyle='--')
        self.Container.canvas.axes.grid(which='minor', alpha=0.2)
        self.Container.canvas.axes.set_xlim(8, -1)
        self.Container.canvas.axes.set_ylim(-1, 8)
        self.origin_robot(0, 0)
        self.sol()
        if(avec_b == 0):
            self.b(float(self.val_xb.toPlainText()),float(self.val_yb.toPlainText()))
    
    @pyqtSlot()
    def Affiche_moi(self, message = "Bot Simulator", error=None):
        self.afficheur.setText(message)
        red = "red"
        white = "white"
        green = "green"
        black = "black"
        if error is None:
            self.afficheur.setStyleSheet("background:{};color:{};box-shadow: 5px 10px;border-radius: 10px".format(white, black))
        elif(error == "success"):
            self.afficheur.setStyleSheet("background:{};color:{};box-shadow: 5px 10px;border-radius: 10px".format(green,white))
        else:
            self.afficheur.setStyleSheet("background:{};color:{};box-shadow: 5px 10px;border-radius: 10px".format(red, white))

if __name__=='__main__':
    app = QtWidgets.QApplication(sys.argv)
    robot = Robot()
    robot.main()
    app.exec_()
