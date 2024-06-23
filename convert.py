import xml.etree.ElementTree as ET
import numpy as np
import math
import sys

def limit(vetor, limite):
    magnitude = np.linalg.norm(vetor)
    if magnitude > limite:
        return vetor * (limite / magnitude)
    return vetor

tree = ET.parse(sys.argv[1])
root = tree.getroot()
coordinates = [coord.text[coord.text.find('-'):coord.text.find('\n', coord.text.find('-'))-1].split() for coord in root.iter('{http://www.opengis.net/kml/2.2}coordinates')][0]
altitude = float([alt.text for alt in root.iter('{http://www.opengis.net/kml/2.2}altitude')][0])

# Mavic 3 (Resolution: 5280x3956, Sensor Size: 17.3 x 13)
GSD = ((altitude * 100) * 17.3) / (5280 * 24)
GSD = math.ceil(GSD / 0.01) / 100
print("GSD: {} px/cm".format(GSD))
verDis30 = 1186.8 * GSD / 100
velo_1f_5s = verDis30 / 5.0 // 0.1 / 10
print("Velocidade 1foto/5s: {} m/s".format(velo_1f_5s))
lateral70 = math.floor(3696 * GSD / 100)
# print("Lateral 70%: ", lateral70)

vs = []
for i in range(4):
    xyz = [float(n) for n in coordinates[i].split(',')]
    # vs.append(p5.Vector(xyz[0], xyz[1])) ### p5 para numpy abaixo:
    vs.append(np.array([xyz[0], xyz[1]]))
xs = [v[0] for v in vs]
ys = [v[1] for v in vs]
xsSort = xs.copy()
xsSort.sort()
no = xs.index(xsSort[0]) if ys[xs.index(xsSort[0])] > ys[xs.index(xsSort[1])] else xs.index(xsSort[1])
so = xs.index(xsSort[0]) if ys[xs.index(xsSort[0])] < ys[xs.index(xsSort[1])] else xs.index(xsSort[1])
ne = xs.index(xsSort[2]) if ys[xs.index(xsSort[2])] > ys[xs.index(xsSort[3])] else xs.index(xsSort[3])
se = xs.index(xsSort[2]) if ys[xs.index(xsSort[2])] < ys[xs.index(xsSort[3])] else xs.index(xsSort[3])
# print(vs[no], vs[so], vs[ne], vs[se])

hor = vs[no]-vs[ne]
# hor = hor.magnitude * 111100 * math.cos(math.radians(vs[no].y)) ### p5 para numpy abaixo:
hor = np.linalg.norm(hor) * 111100 * math.cos(math.radians(vs[no][1]))
# print(hor)
ver = vs[no]-vs[so]
# ver = ver.magnitude * 111100 ### p5 para numpy abaixo:
ver = np.linalg.norm(ver) * 111100
# print(ver)

if hor > ver:
    ori1 = vs[no]
    des2 = vs[se]
else:
    ori1 = vs[se]
    des2 = vs[no]

ori_des_1 = ori1-vs[so]
ori_des_2 = vs[ne]-des2
# nSeg = math.ceil(max(ori_des_1.magnitude, ori_des_2.magnitude) / (lateral70/111100)) ### p5 para numpy abaixo:
nSeg = math.ceil(max(np.linalg.norm(ori_des_1), np.linalg.norm(ori_des_2)) / (lateral70/111100)) 
segO = ori_des_1
# print("Lateral1: ", ori_des_1.magnitude / nSeg * 111100)
# print("Lateral2: ", ori_des_2.magnitude / nSeg * 111100)
# segO.limit(ori_des_1.magnitude / nSeg) ### p5 para numpy abaixo:
segO = limit(segO, np.linalg.norm(ori_des_1) / nSeg)
segD = ori_des_2
# segD.limit(ori_des_2.magnitude / nSeg) ### p5 para numpy abaixo:
segD = limit(segD, np.linalg.norm(ori_des_2) / nSeg)

o = []
o.append(ori1)
for i in range(nSeg-1):
    o.append(o[-1]-segO)
o.append(vs[so])

d = []
d.append(vs[ne])
for i in range(nSeg-1):
    d.append(d[-1]-segD)
d.append(des2)

wp = []
distancia = 0
for i in range(0, nSeg+1, 2):
    wp.append(o[i])
    wp.append(d[i])
    # distancia += ((o[i]-d[i]).magnitude * 111100) ### p5 para numpy abaixo:
    distancia += (np.linalg.norm(o[i]-d[i]) * 111100)
    if i+1 < len(d):
        # distancia += ((d[i]-d[i+1]).magnitude * 111100) ### p5 para numpy abaixo:
        distancia += (np.linalg.norm(d[i]-d[i+1]) * 111100)
        wp.append(d[i+1])
        # distancia += ((d[i+1]-o[i+1]).magnitude * 111100) ### p5 para numpy abaixo:
        distancia += (np.linalg.norm(d[i+1]-o[i+1]) * 111100)
        wp.append(o[i+1])
distancia = math.ceil(distancia)
print("Distancia: {} metros".format(distancia))
tempo = distancia / velo_1f_5s
minutos = int(tempo // 60)
segundos = math.ceil(tempo%60)
print("Tempo: {} min. {:02} seg.".format(minutos, segundos))
fotos = math.ceil(tempo/5)
print("Fotos: {}".format(fotos))

# Arredonda os cantos
wp3 = []
wp3.append(wp[0])
for i in range(1, len(wp)-1):
    dir = wp[i-1]-wp[i]
    # dir.limit(0.0001) ### p5 para numpy abaixo:
    dir = limit(dir, 0.0001)
    wp3.append(wp[i]+dir)
    wp3.append(wp[i])
    dir = wp[i]-wp[i+1]
    # dir.limit(0.0001) ### p5 para numpy abaixo:
    dir = limit(dir, 0.0001)
    wp3.append(wp[i]-dir)
wp3.append(wp[-1])

# for p in wp3:
#     print("{:.14f},{:.14f},{} ".format(p.x, p.y, altitude), end='')
# print()

with open('head.xml', 'r') as file:
    data = file.read().replace('###velocidade###', '{}'.format(velo_1f_5s))
with open('waylines.wpml', 'w') as file:
    print(data, file=file)

with open('first.xml', 'r') as file:
    data = file.read().replace('###coordenada###', '{},{}'.format(wp3[0][0], wp3[0][1]))
    data = data.replace('###altitude###', '{}'.format(altitude))
    data = data.replace('###velocidade###', '{}'.format(velo_1f_5s))
with open('waylines.wpml', 'a') as file:
    print(data, file=file)

for i in range(1, len(wp3)-1):
    with open('placemark.xml', 'r') as file:
        data = file.read().replace('###coordenada###', '{},{}'.format(wp3[i][0], wp3[i][1]))
        data = data.replace('###altitude###', '{}'.format(altitude))
        data = data.replace('###velocidade###', '{}'.format(velo_1f_5s))
        data = data.replace('###index###', '{}'.format(i))
        data = data.replace('###nextindex###', '{}'.format(i+1))
    with open('waylines.wpml', 'a') as file:
        print(data, file=file)

with open('last.xml', 'r') as file:
    data = file.read().replace('###coordenada###', '{},{}'.format(wp3[-1][0], wp3[-1][1]))
    data = data.replace('###altitude###', '{}'.format(altitude))
    data = data.replace('###velocidade###', '{}'.format(velo_1f_5s))
    data = data.replace('###index###', '{}'.format(len(wp3)-1))
with open('waylines.wpml', 'a') as file:
    print(data, file=file)

with open('tail.xml', 'r') as file:
    data = file.read()
with open('waylines.wpml', 'a') as file:
    print(data, file=file)
