# encoding: utf-8

###############################################################################
class Point(object):
  # REF: reference to a point in the sequence file, corresponds to point
  #      t.info = p in the corresponding frame
  # NONE: no point (the trajectory has a hole)
  # INTERP: the trajectory has been interpolated by point t.info = (x,y)
  REF=0; NONE=1; INTERP=2

  def __init__(self, type=NONE, info=None):
    self.type = type
    self.info = info

###############################################################################
class PointSequence(object):
  # A sequence has a width, height and a sequence of frames containing points
  def __init__(self, frames=None, width=0, height=0):
    self.width = width
    self.height = height
    self.frames = frames or []

  def length(self):
    return len(self.frames)

###############################################################################
class Trajectory(object):
  # A trajectory starts on a given frame, and contains a sequence of [Points]
  def __init__(self, starting_frame, points, data=None):
    self.starting_frame = starting_frame
    self.points = points
    self.data = data
    self.normalize() # Automatic normalization

  def length(self):
    return len(self.points)

  def last_frame(self):
    return self.starting_frame + len(self.points) - 1

  def exists_at_frame(self, k):
    return k >= self.starting_frame and k < self.starting_frame + len(self.points)

  # Remove holes before and after the trajectory (not well defined)
  def normalize(self):
    while( self.length() > 0 and self.points[0].type == Point.NONE ):
      self.starting_frame += 1
      self.points = self.points[1:]

    while( self.length() > 0 and self.points[-1].type == Point.NONE ):
      self.points = self.points[:-1]

  def to_descriptor(self, no_interpolates=False):
    descriptor = ""
    descriptor += "S%d " % self.starting_frame
    for p in self.points:
      if p.type == Point.REF:
        descriptor += "P%d " % p.info
      elif p.type == Point.NONE:
        descriptor += "H1 "
      elif p.type == Point.INTERP:
        if no_interpolates:
          descriptor += "H1 "
        else:
          descriptor += "A %g %g " % p.info
    descriptor += ";"
    return descriptor
