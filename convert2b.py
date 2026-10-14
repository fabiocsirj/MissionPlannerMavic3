import math
import sys
import missao_wpml as wpml

# Uso: python3 convert2b.py area.kml [altura_da_decolagem_sobre_o_solo]
# O KML precisa ter a area (poligono) e um ponto chamado "Decolagem".
# A altitude do KML passa a ser a altura sobre o solo (AGL) em cada foto.

FOLGA_MINIMA = 0.8      # fracao da altura AGL mantida sobre o terreno entre waypoints
AMOSTRA_TRECHO = 10.0   # m, espacamento das amostras de terreno entre waypoints
SUBIDA_MAX = 5.0        # m/s, velocidade vertical maxima de subida
DESCIDA_MAX = 4.0       # m/s, velocidade vertical maxima de descida

placemarks = wpml.ler_placemarks(sys.argv[1])
decolagem = next((p for p in placemarks if 'decolagem' in p['nome'].lower() and p['pontos']), None)
if decolagem is None:
    sys.exit('Adicione ao KML um ponto chamado "Decolagem" no local de decolagem.')
area = next((p for p in placemarks if p is not decolagem and len(p['pontos']) >= 3), None)
if area is None:
    sys.exit("KML sem a area (poligono com pelo menos 3 vertices).")
altitude = area['altitude']
if altitude is None:
    altitude = wpml.ler_kml(sys.argv[1])[1]
alturaDecolagem = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0   # ex.: decolagem de laje/predio
print("Altura AGL: {:g} m".format(altitude))

pontos = wpml.gerar_grade(area['pontos'], altitude)

# Amostras intermediarias de cada trecho, para checar a folga sobre o terreno entre waypoints
ds = wpml.distancias(pontos)
amostras = []
for i in range(len(pontos) - 1):
    (lon1, lat1), (lon2, lat2) = pontos[i], pontos[i+1]
    n = max(1, math.ceil(ds[i] / AMOSTRA_TRECHO))
    amostras.append([(lon1 + (lon2 - lon1) * j / n, lat1 + (lat2 - lat1) * j / n) for j in range(1, n)])

terreno = wpml.altitudes_terreno(decolagem['pontos'][:1] + pontos + [p for a in amostras for p in a])
base = terreno[0] + alturaDecolagem
zWp = terreno[1:len(pontos) + 1]
zAmostras = []
k = len(pontos) + 1
for a in amostras:
    zAmostras.append(terreno[k:k + len(a)])
    k += len(a)

# Altura relativa a decolagem para manter a altura AGL em cada foto
alturas = [altitude + z - base for z in zWp]

# Entre waypoints o drone voa em linha reta: se o terreno subir no meio, eleva o trecho
folga = FOLGA_MINIMA * altitude
elevados = 0
for i, zs in enumerate(zAmostras):
    n = len(zs) + 1
    falta = 0
    for j, z in enumerate(zs, 1):
        h = alturas[i] + (alturas[i+1] - alturas[i]) * j / n
        falta = max(falta, folga - (base + h - z))
    if falta > 0:
        alturas[i] += falta
        alturas[i+1] += falta
        elevados += 1
alturas = [math.ceil(h * 10) / 10 for h in alturas]

# Velocidade: intervalo entre fotos e limite de subida/descida nos trechos adjacentes a cada waypoint
velocidadeBase = wpml.calcular_velocidade(pontos)
limites = []
for i in range(len(pontos) - 1):
    dh = alturas[i+1] - alturas[i]
    vertical = SUBIDA_MAX if dh > 0 else DESCIDA_MAX
    limites.append(vertical * ds[i] / abs(dh) if dh else math.inf)
velocidades = []
for i in range(len(pontos)):
    v = min([velocidadeBase] + limites[max(0, i-1):i+1])
    velocidades.append(max(wpml.VELOCIDADE_MIN, v // 0.1 / 10))

print("Terreno na decolagem: {:g} m (+{:g} m sobre o solo)".format(terreno[0], alturaDecolagem))
print("Terreno nas fotos: {:g} a {:g} m".format(min(zWp), max(zWp)))
print("Altura relativa a decolagem: {:g} a {:g} m".format(min(alturas), max(alturas)))
if elevados:
    print("Trechos elevados para manter {:.0f} m sobre o terreno: {}".format(folga, elevados))
if min(alturas) < 0:
    print("ATENCAO: ha waypoints abaixo do ponto de decolagem (altura negativa).")
wpml.resumo(pontos, velocidades)
wpml.escrever_missao(pontos, alturas, velocidades)
