#!D:\Progs\Python25\python
""" FILE:  pngs2agif.py
    DATE:  29 OCT 2010
    AUTH:  Gary E. Deschaines (gedesch@hotmail.com)
    
    Converts series of png "####.png" files to a single
    animated GIF "frames_0000_####.gif" file using the Python 
    Imaging Library (PIL) and the gifmaker.py script obtained
    from the source distribution of PIL.
"""

from PIL    import Image
from locale import format_string

import glob
import gifmaker

if __name__ == "__main__":
    
    seq  = []         # sequence of converted images
    num  = -1         # counter of converted images minus 1
    idir = "./pngs/"  # input file directory
    odir = "./gifs/"  # output file directory
    
    # Assemble sequence of converted images
    for ifile in glob.glob(idir + "*.png"):  
        im  = Image.open(ifile, "r")
        imc = im.convert('P')
        seq.append(imc)
        print "Loaded %s and converted from %s to %s" % \
              (ifile, im.mode, imc.mode)
        num += 1
  
    # Create compressed animated GIF file
    ofile = odir + "frames_0000_" + format_string("%04d", num) + ".gif"
    fp    = open(ofile, "wb")
    gifmaker.makedelta(fp, seq)
    fp.close()
    
