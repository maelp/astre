# encoding: utf-8

import numpy as np

class _Pixel(object):
  def __init__(self,x,y,rad):
    self.x = x
    self.y = y
    self.rad = rad

  def __cmp__(self, b):
    t = self.rad - b.rad
    if t < -0.001: return -1
    elif t > 0.001: return 1
    else: return 0

class DiscreteArea(object):
  def __init__(self, max_r):
    assert max_r > 0

    self.max_r_sq = max_r_sq = max_r*max_r
    self.width = width = max_r+1
    self.data = data = np.zeros((width, width))
    pixels = sorted([_Pixel(x,y,np.sqrt(x*x+y*y)) for x in range(width) for y in range(x+1)])

    cur_area = 0
    # We are only looking for the areas of balls with euclidian radii less than r,
    # so we determine how many we have to consider
    num_pix = len(pixels)
    for i in range(len(pixels)-1):
      if pixels[i].rad > max_r:
        num_pix = i
        break
    # Several pixels are at an equal distance from the center, so we need to
    # compute all their contributions to the current area. This is why we
    # consider successive "layers" of pixels
    fst_pix_layer = 0
    while fst_pix_layer < num_pix:
      cur_radius = pixels[fst_pix_layer].rad
      lst_pix_layer = fst_pix_layer
      while lst_pix_layer < num_pix-1 and pixels[lst_pix_layer+1].rad - cur_radius < 0.001:
        lst_pix_layer += 1

      # Add the contribution of all pixels to the ball area
      for q in range(fst_pix_layer, lst_pix_layer+1):
        p = pixels[q]
        px, py = p.x, p.y

        # Increment the area depending on the pixel's position in the octant
        # (when we replicate the octant 8 times, the central pixel is only
        # replicated once, the border pixels are replicated 4 times, and the
        # inside pixels are replicated 8 times)
        if px == 0 and py == 0: cur_area += 1
        elif py == 0 or py == px: cur_area += 4
        else: cur_area += 8

      # Set the ball area of the corresponding pixels
      for q in range(fst_pix_layer, lst_pix_layer+1):
        p = pixels[q]
        px, py = p.x, p.y

        # data is the first quadrant, so we fill it by symmetry
        data[px, py] = float(cur_area)
        data[py, px] = float(cur_area)

      fst_pix_layer = lst_pix_layer + 1 ;

    del pixels

  def area(self, x, y):
    d_sq = x*x + y*y
    if d_sq > self.max_r_sq: return np.pi*float(d_sq)
    else:
      if x < 0: x = -x
      if y < 0: y = -y
      return self.data[x,y]
