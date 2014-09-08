'''
pgm_utils.py : Python utilties for PGM files
             
Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.

Change log:

20-APR-2014 - Simon D. Levy - Get params from command line
'''

def pgm_load(filename):

    print('Loading image from file %s...' % filename)
    
    fd = open(filename, 'rt')
    
    # Skip constant header
    fd.readline()
    
    # Grab image size (assume square)
    imgsize = [int(tok) for tok in fd.readline().split()]
        
    # Start with empty list
    imglist = []
    
    # Read lines and append them to list until done
    while True:  
        
        line = fd.readline()
        
        if len(line) == 0:
            break       
            
        imglist.extend([int(tok) for tok in line.split()])   

    fd.close()
    
    # Convert list into bytes
    imgbytes = bytearray(imglist)     

    return imgbytes, imgsize
    
def pgm_save(filename, imgbytes, imgsize):    
    
    print('\nSaving image to file %s' % filename)
        
    output = open(filename, 'wt')
    
    output.write('P2\n%d %d 255\n' % imgsize)
    
    wid, hgt = imgsize
    
    for y in range(hgt):
        for x in range(wid):
            output.write('%d ' % imgbytes[y * wid + x])
        output.write('\n')

    output.close()

     
                                        
