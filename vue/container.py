# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 21:23:10 2021

@author: ASUS
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout

from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure

    
class Container(QWidget):
    
    def __init__(self, parent = None):

        QWidget.__init__(self, parent)
        
        self.canvas = FigureCanvas(Figure())
        
        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.canvas)
        
        self.canvas.axes = self.canvas.figure.add_subplot(111)
        self.canvas.axes.spines['top'].set_visible(False)
        self.canvas.axes.spines['left'].set_visible(False)
        self.canvas.axes.xaxis.set_ticks_position('bottom')
        self.canvas.axes.spines['bottom'].set_position(('data',-1))
        self.canvas.axes.yaxis.set_ticks_position('right')
        self.canvas.axes.spines['right'].set_position(('data',-1))
        self.canvas.axes.grid(which='both', linestyle='--')
        self.canvas.axes.grid(which='minor', alpha=0.2)
        self.canvas.axes.set_xlim(8, -1)
        self.canvas.axes.set_ylim(-1, 8)
        self.setLayout(vertical_layout)