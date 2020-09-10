#!/usr/bin/python

import sys
import os
import xml.etree.ElementTree as ET

prefix="{http://www.w3.org/2000/svg}"

def getTranslation(transform):
    if transform==None:
        return 0.0, 0.0
    tx,ty = transform[transform.index("(") + 1:transform.rindex(")")].split(',')
    return float(tx), float(ty)

def getPoints(d):
    if d==None:
        return None
    ptsStr = d.split(' ')
    count = 0
    pts = []
    i = 0
    isRelative = False
    dispX = 0
    dispY = 0
    for s in ptsStr:
        print s
        if s == 'm':
            count = 0
            isRelative = True
            pts.append([])
        elif s == 'M':
            count = 0
            isRelative = False
            pts.append([])   
        elif s == 'l':
            isRelative = True
        elif s == 'L':
            isRelative = False
        elif s == 'z' or s == 'Z':
            i += 1
#            print count
        else:
            p = s.split(',')
            if isRelative and count>0:
                pts[i].append([float(p[0])+pts[i][count-1][0],float(p[1])+pts[i][count-1][1]])
            elif isRelative and count == 0 and i>0:
                pts[i].append([float(p[0])+pts[i-1][0][0],float(p[1])+pts[i-1][0][1]])
#                pts[i].append([float(p[0])+pts[i-1][len(pts[i-1])-1][0],float(p[1])+pts[i-1][len(pts[i-1])-1][1]])
            else:
                pts[i].append([float(p[0]),float(p[1])])

            count += 1
    return pts

def translatePoints(points,tx,ty):
    if tx == 0.0 and ty == 0.0:
        return points
    trans_pts = points
    for l in trans_pts:
        for p in l:
            p[0] += tx
            p[1] += ty
            # inverting y signal
            p[1] = -p[1]
    return trans_pts

def points2lines(points,lines):
    for out in points:
#        print xrange(len(out))
        for p in xrange(len(out)-1):
#            print "{"+str(p)+"-"+str(p+1)+"}",out[p],out[p+1]
            lines.append( [out[p][0], out[p][1], out[p+1][0], out[p+1][1]] ) 
#        print "{"+str(len(out)-1)+"-0}",out[len(out)-1], out[0]
        lines.append( [out[len(out)-1][0], out[len(out)-1][1], out[0][0], out[0][1]] )     
    return lines

def getLimits(lines):
    # find mins and maxs
    minX = min([min([x[0] for x in lines]),min([x[2] for x in lines])])
    maxX = max([max([x[0] for x in lines]),max([x[2] for x in lines])])
    minY = min([min([x[1] for x in lines]),min([x[3] for x in lines])])
    maxY = max([max([x[1] for x in lines]),max([x[3] for x in lines])])
    return minX,maxX,minY,maxY

def scaleTranslateLines(lines,tx,ty,scale):
    sumList = [tx,ty,tx,ty]
    output = []
    for l in lines:
        l = [x+y for x,y in zip(l,sumList)]
        l = [i*scale for i in l]
        l = [int(i) for i in l]
        output.append(l)
    return output

def generateDotMapFile(lines,outputName):

    minX,maxX,minY,maxY = getLimits(lines)
    print minX,maxX,minY,maxY

    scale = 50
    lines = scaleTranslateLines(lines,-minX,-minY,scale)

    minX,maxX,minY,maxY = getLimits(lines)
    print minX,maxX,minY,maxY

    lines.sort(key=lambda x: x[0])

    print outputName
    f = open(outputName, 'w')
    f.write("2D-Map\n")
    f.write("Resolution: 0\n")
    f.write("LineMinPos: %d %d\n" % (minX, minY) )
    f.write("LineMaxPos: %d %d\n" % (maxX, maxY) )
    f.write("NumLines: %d\n" % len(lines) )
    f.write("LinesAreSorted: false\n")
    f.write("Cairn: RobotHome %d %d 0.000000 \"\" ICON \"\"\n" % (minX+10, minY+10) )
    f.write("LINES\n")
    for l in lines:
        f.write("%d %d %d %d\n" % (l[0],l[1],l[2],l[3]) )
    f.write("DATA\n")

def main(inputFile):
    print 'InputFile:', inputFile
    tree = ET.parse(inputFile)
    root = tree.getroot()
#    print "TAG:",root.tag, root.tag.strip(prefix)
#    print "ATTRIB",root.attrib

    lines = []

    for g in root:
        if g.tag == prefix+'g': # Inner g
            for p in g:
                if p.tag == prefix+'path':
                    points = getPoints(p.attrib.get('d'))
                    ntx,nty = getTranslation(p.attrib.get('transform'))
                    print "TRANS:",ntx,nty,p.attrib.get('transform')
                    pts = translatePoints(points,ntx,nty)
#                    print pts
                    lines = points2lines(pts,lines)

#    for l in lines:   
#        print "LINHA",l  
    print "# Lines",len(lines)

    filename, file_extension = os.path.splitext(inputFile)
    generateDotMapFile(lines,filename+".map")


#    for c1 in root:
#        if c1.tag == prefix+'g':    # Outer g, get transform
##            print "ACHOU"
#            tx,ty = getTranslation(c1.attrib.get('transform'))
##            print "TRANS:",tx,ty
#            for g in c1:
#                if g.tag == prefix+'g': # Inner g
#                    for p in g:
#                        if p.tag == prefix+'path':
#                            points = getPoints(p.attrib.get('d'))
#                            ntx,nty = getTranslation(p.attrib.get('transform'))
##                            print "TRANS:",ntx,nty
#                            print points

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "USAGE: python svg2mobilesim.py INPUTFILE.svg"
    else:
        main(sys.argv[1])
