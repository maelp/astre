# encoding: utf-8
import string
from pymage.utils.odict import OrderedDict

###############################################################################
class DescFile(object):
  def __init__(self):
    self.headers = OrderedDict()
    self.tags = []
    self.data = []

  @staticmethod
  def from_filename(filename):
      desc_file_str = open(filename).read()
      return DescFile.from_string(desc_file_str)

  @staticmethod
  def from_string(desc_file_str):
    self = DescFile()
    self.headers = OrderedDict()
    self.tags = []
    self.data = []

    is_data = False

    n_entries = None

    for line in desc_file_str.splitlines():
      line = line.strip()

      if len(line) == 0 or line.startswith('#'):
        continue

      if not(is_data) and line == 'DATA':
        is_data = True

      elif not(is_data):
        # Headers
        header, value = line.split('=')
        self.headers[header.strip()] = value.strip()

      else:
        # Data
        entries = line.split()
        if n_entries == None:
          n_entries = len(entries)
          self.tags = [None]*n_entries

        elif len(entries) != n_entries:
          print "[ERROR] line %s" % line
          print " > Invalid number of entries (%d instead of %d)" % (len(entries),n_entries)
          sys.exit(-1)

        data_line = []
        for k in range(len(entries)):
          entry = entries[k]
          if entry.find(':') >= 0:
            tag, value = entry.split(':')
            value = float(value)
          else:
            tag = None
            value = float(entry)

          if( self.tags[k] and tag and self.tags[k] != tag ):
            print "[ERROR] entry %s" % entry
            print " > tag is %s instead of %s" % (tag,self.tags[k])
            sys.exit(-1)
          else:
            if self.tags[k] == None and tag != None:
              self.tags[k] = "%s" % tag # FIXME: copy
            data_line.append(value)

        self.data.append(data_line)
    return self

  def save(self, filename):
    file = open(filename, 'w')
    for key, value in self.headers.iteritems():
      file.write("{0} = {1}\n".format(key, value))
    file.write("DATA\n")
    for l in range(len(self.data)):
      for i in range(len(self.data[l])):
        if self.tags[i]:
          try:
            file.write("{0}:{1:g} ".format(self.tags[i], self.data[l][i]))
          except ValueError:
            print "l=",l,"i=",i,"data:",self.data[l][i]
            raise ValueError("error")
        else:
          file.write("{0:g} ".format(self.data[l][i]))
      file.write("\n")
    file.close()

