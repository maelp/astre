# encoding: utf-8
import string
import copy
import time

from pymage.format.descfile import DescFile
from pymage.trajs.structs import Point, PointSequence, Trajectory
from pymage.utils.odict import OrderedDict
import copy

###############################################################################
class PointsDesc(object):
  def __init__(self, width=0, height=0, uid=None, first_frame=0, frames=None):
    self.width = width
    self.height = height
    if uid is None: uid = int(time.time())
    self.uid = uid
    self.first_frame = first_frame
    self.frames = frames or []
    self.nfields = 0

    self.headers = None
    self.tags = None

  def copy(self, add_n_cols=0):
    assert add_n_cols >= 0

    c = PointsDesc()
    c.width = self.width
    c.height = self.height
    c.uid = self.uid
    c.first_frame = self.first_frame
    c.nfields = self.nfields + add_n_cols
    c.headers = OrderedDict(self.headers)
    c.tags = copy.copy(self.tags)
    c.tags.extend([None]*add_n_cols)
    c.frames = []
    for f in self.frames:
      c.frames.append([p+[0.0]*add_n_cols for p in f])

    return c

  def save(self, filename):
    df = self.to_descfile()
    df.save(filename)

  @staticmethod
  def from_string(points_desc_str=""):
    self = PointsDesc()
    df = DescFile.from_string(points_desc_str)

    assert( df.headers["type"] == "PointsFile v.1.0" )
    assert( "width" in df.headers and "height" in df.headers )

    self.headers = OrderedDict(df.headers)
    self.tags = copy.copy(df.tags)[1:]

    self.width = int(df.headers["width"])
    self.height = int(df.headers["height"])
    self.uid = int(df.headers["uid"])

    first_frame = None
    last_frame = None
    frames = []

    if len(df.data) > 0:
      nfields = len(df.data[0])
      assert( nfields >= 3 )
      self.nfields = nfields

      for entry in df.data:
        f = entry[0]
        if abs( f - int(f)) > 0.01:
          raise Exception("Error: first entry should be a frame number!")
        f = int(f)
        if first_frame == None or f < first_frame:
          first_frame = f
        if last_frame == None or f > last_frame:
          last_frame = f

      frames = [ [] for k in range(last_frame-first_frame+1) ]

      for entry in df.data:
        f = int(entry[0])-first_frame
        frames[f].append(entry[1:])

    self.first_frame = first_frame
    self.frames = frames

    return self

  def to_descfile(self):
    df = DescFile()

    df.data = []
    for i, f in enumerate(self.frames):
      lines = [[i+self.first_frame]+p for p in f]
      df.data += lines

    self.headers['type'] = 'PointsFile v.1.0'
    self.headers['width'] = str(self.width)
    self.headers['height'] = str(self.height)
    self.headers['uid'] = str(self.uid)

    df.headers = OrderedDict(self.headers)
    df.tags = ['f']+copy.copy(self.tags)

    return df

  def getSequence(self, other_fields=None):
    other_fields = other_fields or []
    fields = [0,1] + other_fields
    for i in range(len(fields)):
      if fields[i] < 0: fields[i] += self.nfields
      assert(fields[i] >= 0 and fields[i] < self.nfields)

    frames = map( lambda f: [ [t[i] for i in fields] for t in f ], self.frames )
    return PointSequence( frames, self.width, self.height )

  def getTrajectories(self, column_index=-1):
    trajs = {}

    if len(self.frames) > 0:
      first_frame = self.frames[0]
      assert(len(first_frame) > 0)
      nentries = len(first_frame[0])
      assert(nentries >= 3)
      if column_index < 0: column_index += nentries
      assert(column_index >= 2 and column_index < nentries)
      for k in range(len(self.frames)):
        f = self.frames[k]
        for j in range(len(f)):
          d = f[j]

          pf = d[column_index]
          if abs(pf-int(pf)) > 1e-5:
            raise Exception("Error, trajectory column contains non-integer values!")
          p = int(pf)
          if p < 0: continue

          if not p in trajs:
            trajs[p] = Trajectory( k, [Point(Point.REF, j)])

          else:
            t = trajs[p]
            last_frame = t.starting_frame + len(t.points)-1
            cur_frame = k
            if last_frame == cur_frame:
                raise Exception("Error, in image {0}, several points have the same trajectory id {1}".format(k, p))
            for p in range(last_frame+1,cur_frame):
              t.points.append( Point(Point.NONE) )
            t.points.append( Point(Point.REF,j) )

    return trajs.values()
