#!/usr/bin/env python
# encoding: utf-8

#   ASTRE a-contrario single trajectory extraction
#   Copyright (C) 2011 Mael Primet (mael.primet AT gmail.com)
# 
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
# 
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
# 
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
from pymage.vendor import argparse

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from random import randint, random

from pymage.trajs.pointsdesc import PointsDesc
from pymage.trajs.structs import Point, PointSequence, Trajectory

# TODO: Why aren't the points centered at line extremities?
# TODO: shift+"+"/"-" should zoom and set the window size to accommodate the
#       viewer size
# TODO: "=" should reinit view zoom
# TODO: mode where we display all trajectories with links in either red or
#       blue, depending on whether the link is correct or not

###############################################################################
def random_in(min,max):
  return min + random()*(max-min)

def randomColor():
  # Bright pastel colors
  hue = random_in(0, 1.0)
  saturation = random_in(0.8,1.0)
  lightness = random_in(0.5,1.0)
  c = QColor.fromHslF(hue,saturation,lightness)

  return c

###############################################################################
class MyTrajectory(Trajectory):

  def __init__(self, starting_frame, points):
    super(MyTrajectory,self).__init__(starting_frame, points)
    self.pen = QPen(randomColor())

  # Interpolate in holes, so displaying is easier
  def interpolate(self,seq):
    self.normalize()
    if self.length() == 0: return

    px, py = None, None
    for k in range(self.length()):
      pk = self.points[k]

      if pk.type == Point.NONE:
        assert( px != None )
        # Find next point
        np = None
        for r in range(k+1,self.length()):
          q = self.points[r]
          if q.type != Point.NONE:
            np = r
            break
        assert( np != None )
        pq = self.points[np]
        if pq.type == Point.REF:
          nx, ny = seq.frames[self.starting_frame+np][pq.info]
        else:
          nx, ny = pq.info

        x = px + float(nx-px)/float(np-k+1)
        y = py + float(ny-py)/float(np-k+1)
       
        pk.type = Point.INTERP
        pk.info = (x, y)

      if pk.type == Point.REF:
        px, py = seq.frames[self.starting_frame+k][pk.info]
      else:
        px, py = pk.info

###############################################################################
class Cache:
  def __init__(self, seq=None, trajs=None):
    self.setSequence(seq)
    trajs = trajs or []
    self.setTrajectories(trajs)

  def setSequence(self, seq=None):
    self.seq = seq

  def setTrajectories(self, trajs=None):
    trajs = trajs or []
    self.trajs = trajs
    self.trajs_lines = []

    if self.seq == None:
      return

    # Cache the drawing paths of trajectories
    # TODO: does not seem to improve the performances enough (when using
    #       antialiasing)
    frames = self.seq.frames
    for t in trajs:
      sf = t.starting_frame
      points = t.points
      length = len(points)
      prev_point = None

      traj_lines = []

      for k in range(length):
        f = sf+k
        p = points[k]
        if p.type == Point.REF:
          x, y = frames[f][p.info]
        elif p.type == Point.INTERP:
          x, y = p.info
        else:
          print( "Error: trajectory should be interpolated!\n" )
          exit(-1)

        if prev_point != None:
          px, py = prev_point
          traj_lines.append(QLineF(px,py,x,y))
        prev_point = (x, y)

      self.trajs_lines.append(traj_lines)

###############################################################################
class PointViewer(QGraphicsView):

  def __init__(self, seq = None, trajs=None, parent = None):
    super(PointViewer, self).__init__(parent)
    trajs = trajs or []
    self.cur_frame = 0
    self.font = QFont('Mono', 9)

#    self.setRenderHint(QPainter.Antialiasing)
    self.setRenderHint(QPainter.TextAntialiasing)
    self.setDragMode(QGraphicsView.ScrollHandDrag)
    self.setOptimizationFlags( QGraphicsView.DontSavePainterState )
    self.setOptimizationFlags( QGraphicsView.DontAdjustForAntialiasing )

    self.no_brush = QBrush()
    self.no_brush.setStyle( Qt.NoBrush )
    self.back_brush = QBrush(QColor.fromRgb(30,30,30))
    self.point_brush = QBrush(QColor.fromRgb(240,240,240))
    self.interp_brush = QBrush(QColor.fromRgb(240,30,30))
    self.point_pen = QPen(QColor.fromRgb(240,240,240))
    self.interp_pen = QPen(QColor.fromRgb(240,30,30))
    self.point_pen.setWidth(0)

    self.highlight = False

    self.setBackgroundBrush(self.back_brush)

    self.setSequence(seq)
    self.setTrajectories(trajs)

    self.draw_trajs = True
    self.draw_trajs_from = 0

    self.go(0)

  def toggleHighlight(self):
    self.highlight = not self.highlight
  def drawTrajs(self, draw=True):
    self.draw_trajs = draw
  def drawTrajsFrom(self, frame=0):
    self.draw_trajs_from = frame

  def fitScene(self):
    if self.seq == None: return
    self.fitInView(0, 0, self.seq.width, self.seq.height)

  def setTrajectories(self, trajs = []):
    self.trajs = [MyTrajectory(t.starting_frame, t.points) for t in trajs]

    if self.seq != None:
      for t in self.trajs: t.interpolate(self.seq)
      self.trajs = filter( lambda t: t.length() > 0, self.trajs )
      self.cache.setTrajectories(self.trajs)

    # TODO: to improve drawing efficiency, we should cache the
    #       trajectory drawing instructions
    self.update()

  def drawBackground(self, painter, rect):
    painter.setClipRect( rect )
    painter.fillRect( rect, self.back_brush )

    if self.seq == None: return

    seq = self.seq

    # scene border
    painter.setPen( self.point_pen )
    painter.setBrush( self.no_brush )
    painter.drawRect(-1, -1, seq.width+2, seq.height+2)

    if self.draw_trajs:
      frames = self.seq.frames
      trajs_lines = self.cache.trajs_lines

      for i in range(len(self.trajs)):
        t = self.trajs[i]
        sf = t.starting_frame
        if self.cur_frame >= sf+1 and self.cur_frame >= self.draw_trajs_from:
          painter.setPen(t.pen)
          lines = trajs_lines[i]
          length = len(t.points)

          kmin = max(0, self.draw_trajs_from - sf)
          kmax = min(length-1, self.cur_frame - sf)

          for k in range(kmin,kmax):
            painter.drawLine( lines[k] )

          # Interpolated points
          if( self.cur_frame - sf < length ):
            p = t.points[self.cur_frame-sf]
            if p.type == Point.INTERP:
              px, py = p.info
              painter.setPen(self.interp_pen)
              painter.drawRect(px-0.5,py-0.5,1,1)

    # Draw points
    painter.setBrush(self.point_brush)
    painter.setPen(self.point_pen)
    f = self.seq.frames[self.cur_frame]
    for j in range(len(f)):
      p = f[j]
      px, py = p[0], p[1]
      painter.drawRect(px-0.5,py-0.5,1,1)

    if self.highlight:
      painter.setBrush(QBrush(QColor('red')))
      painter.setPen(QPen(QColor('red')))
      for i in range(len(self.trajs)):
        t = self.trajs[i]
        sf = t.starting_frame
        if self.cur_frame >= sf and self.cur_frame <= sf+t.length()-1:
          p = t.points[self.cur_frame-sf]
          if p.type == Point.REF:
            pq = p.info
            pp = f[pq]
            px, py = pp[0], pp[1]
            painter.drawRect(px-0.5, py-0.5, 1, 1)

  def setSequence(self, seq = None):
    self.seq = seq
    self.cache = Cache()
    self.cache.setSequence(seq)

    if seq == None:
      self.setScene(QGraphicsScene())
      return

    scene = QGraphicsScene()
    scene.setSceneRect(0, 0, seq.width, seq.height)
    self.setSceneRect(0,0,seq.width,seq.height)
    self.go(self.cur_frame)

  def go(self, frame):
    if self.seq == None: return

    while frame < 0: frame += self.seq.length()
    while frame >= self.seq.length(): frame -= self.seq.length()

    self.cur_frame = frame
    self.update()
    self.emit(SIGNAL('frameChanged'),frame)

  def step(self, delta):
    self.go(self.cur_frame + delta)

#  def mouseMoveEvent(self, event):
#    TODO: show the point / trajectory infos when hovering

###############################################################################
class Window(QWidget):

  def __init__(self, title=None, parent = None):
    QWidget.__init__(self, parent)

    self.view = PointViewer()
    # Argument of numerical commands
    self.numerical_argument = 0

    self.frameLabel = QLabel(self.tr("Frame #"))
    self.connect(self.view, SIGNAL('frameChanged'), self.updateFrameLabel)

    lblLayout = QHBoxLayout()
    commandsLabel = QLabel(self.tr("<b>f</b>/<b>b</b>: motion"))
    lblLayout.addWidget(self.frameLabel)
    lblLayout.addStretch()
    lblLayout.addWidget(commandsLabel)

    hbtnLayout = QHBoxLayout()
    helpBtn = QPushButton(self.tr("help"))
    hbtnLayout.addStretch()
    hbtnLayout.addWidget(helpBtn)
    helpBtn.clicked.connect(self.showHelp)

    controlsLayout = QVBoxLayout()
    controlsLayout.addWidget(self.view)
    controlsLayout.addLayout(lblLayout)
    controlsLayout.addLayout(hbtnLayout)

    self.setLayout(controlsLayout)

    self.setWindowTitle(self.tr(title or "Point Viewer"))

  def updateFrameLabel(self, frame):
    self.frameLabel.setText(self.tr("Frame #%d" % frame))

  def showHelp(self):
      dialog = QDialog(self)
      layout = QGridLayout()
      layout.setHorizontalSpacing(20)
      labels = [
              ["<b>f</b>/<b>b</b>", "forward / backward"],
              ["<b>q</b>", "quit"],
              ["<b>+</b>/<b>-</b>/<b>=</b>", "zoom in/out/default"],
              ["&lt;n&gt;<b>g</b>", "goto frame (eg. 45g)"],
              ["<b>t</b>", "toggle trajectory drawing"],
              ["<b>m</b>", "set minimal starting frame for trajectories"],
              ["<b>h</b>", "toggle trajectory points highlighting"],
              ["<b>x</b>", "randomize colors"],
      ]
      for i, ls in enumerate(labels):
          layout.addWidget(QLabel(ls[0]), i, 0)
          layout.addWidget(QLabel(ls[1]), i, 1)

      closeBtn = QPushButton("&Close")
      closeBtn.clicked.connect(dialog.accept)
      layout.addWidget(closeBtn, len(labels), 1)

      dialog.setLayout(layout)

      dialog.open()

  def keyPressEvent(self, event):
    repaint = False

    if type(event) == QKeyEvent:
      c = event.key()

      # forward
      if c == Qt.Key_F:
        self.view.step(+1)
        event.accept()
        repaint = True

      # backward
      elif c == Qt.Key_B:
        self.view.step(-1)
        event.accept()
        repaint = True

      # go
      elif c == Qt.Key_G:
        self.view.go(self.numerical_argument)
        event.accept()
        repaint = True

      # quit
      elif c == Qt.Key_Q:
        quit()
        event.accept()

      # zoom
      elif c == Qt.Key_Plus:
        self.view.scale(1.2, 1.2)
        event.accept()
        repaint = True

      elif c == Qt.Key_Minus:
        self.view.scale(1/1.2, 1/1.2)
        event.accept()
        repaint = True

      elif c == Qt.Key_Equal:
        # should do default scale
        event.accept()
        repaint = True

      # Toggle draw trajs
      elif c == Qt.Key_T:
        self.view.drawTrajs(not self.view.draw_trajs)
        event.accept()
        repaint = True
      # Set first frame for drawing the trajectories
      elif c == Qt.Key_M:
        self.view.drawTrajsFrom(self.view.cur_frame)
        event.accept()
        repaint = True

      # Highlight
      elif c == Qt.Key_H:
        self.view.toggleHighlight()
        event.accept()
        repaint = True

      # Randomize colors
      elif c == Qt.Key_X:
        for t in self.view.trajs:
          t.pen = QPen(randomColor())
        event.accept()
        repaint = True

      else:
        numbers = {
            Qt.Key_0: 0, Qt.Key_1: 1, Qt.Key_2: 2, Qt.Key_3: 3, Qt.Key_4: 4,
            Qt.Key_5: 5, Qt.Key_6: 6, Qt.Key_7: 7, Qt.Key_8: 8, Qt.Key_9: 9,
        }
        # Numerical argument
        if c in numbers:
          n = numbers[c]
          self.numerical_argument *= 10
          self.numerical_argument += n
          event.accept()
          return
        else:
          event.ignore()

    else:
      event.ignore()

    # Reset numerical argument
    self.numerical_argument = 0

    if repaint:
      self.view.scene().update()

  def setSequence(self, seq):
    self.view.setSequence(seq)

  def setTrajectories(self, trajs):
    self.view.setTrajectories(trajs)

  def setNumber(self, number):
    self.number = number

  def setText(self, text):
    self.text = QString(text)

###############################################################################
if __name__ == "__main__":
  import sys

  app = QApplication(sys.argv)

  parser = argparse.ArgumentParser(description='Points description file viewer')
  parser.add_argument(
      'file', help='The file containing the points and trajectories'
  )
  parser.add_argument(
      '--trajectory-column', metavar='COL', type=int, default=-1,
      help='The column (0-based) containing the trajectory labels (default: -1, the last column)'
  )
  parser.add_argument(
      '-t', '--show-trajectories', action='store_true',
      help='Show trajectories'
  )

  args = parser.parse_args()
  file = args.file
  trajectory_column = args.trajectory_column
  show_trajectories = args.show_trajectories

  window = Window()

  f = open(file)
  desc = PointsDesc.from_string(f.read())
  seq = desc.getSequence()
  window.setSequence(seq)

  if show_trajectories:
    if len(desc.frames) > 0:
      first_frame = desc.frames[0]
      if len(first_frame) > 0:
          nentries = len(first_frame[0])
          # original number of columns = nentries+1
          if trajectory_column < 0:
              trajectory_column += nentries+1
          if trajectory_column < 3:
              raise Exception('Invalid trajectory column {0} (should be >= 3 since the three first entries are <f> <x> <y>)'.format(trajectory_column))
          trajectory_column -= 1
          trajs = desc.getTrajectories(trajectory_column)
          window.setTrajectories(trajs)

  window.show()

  sys.exit(app.exec_())
