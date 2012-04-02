from PIL import Image, ImageDraw
from pymage.trajs.pointsdesc import PointsDesc
from pymage.trajs.structs import Point, PointSequence, Trajectory

file='snow.30Hz.desc'
f = open(file)
desc = PointsDesc.from_string(f.read())
seq = desc.getSequence()

frames = [35,36,37]
def frame_filename(f):
    return 'public/images/frame'+str(f-35+1)+'.jpg'

def process_frame(f):
    im = Image.open(frame_filename(f))
    frame = seq.frames[f]
    draw = ImageDraw.Draw(im)
    for p in frame:
        x, y = p[0], p[1]
        w = 5
        color = '#FF0000'
        draw.ellipse((x-w/2,y-w/2,x+w/2,y+w/2), fill=color, outline=color)
    draw.im.save_ppm('out'+str(f)+'.ppm')

for f in frames:
  process_frame(f)
