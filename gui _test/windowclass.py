from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QIcon
import sys


class Window(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Goomba")
        self.setWindowIcon(QIcon("mecha_goomba_karniz.png"))  # should just be file in same folder
        
        #self.setFixedHeight(200)
        #self.setFixedWidth(400)
        self.setGeometry(550,200, 400,300)
        #self.setStyleSheet('background-color:blue')

        stylesheet = (
            'background-color:blue'
        )
        self.setStyleSheet(stylesheet)

    
app = QApplication([])
window = Window()
window.show()
sys.exit(app.exec())