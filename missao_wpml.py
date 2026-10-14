import xml.etree.ElementTree as ET
import urllib.request
import urllib.error
import json
import math
import os
import ssl
import sys
import time

KML = '{http://www.opengis.net/kml/2.2}'
METROS_GRAU = 111100

VELOCIDADE_MAX = 9.0        # m/s (mesma da missão de referência)
VELOCIDADE_MIN = 1.0        # m/s
INTERVALO_MIN_FOTO = 3.0    # s, tempo mínimo entre fotos para a câmera gravar
DRONE_ENUM = 68             # mesmo valor do head.xml

OPENTOPODATA = 'https://api.opentopodata.org/v1/srtm30m'
CACHE_TERRENO = 'cache_srtm30m.json'


def ler_coordenadas(coord):
    pontos = []
    for item in coord.text.split():
        lon, lat = item.split(',')[:2]
        pontos.append((float(lon), float(lat)))
    return pontos


def ler_kml(arquivo):
    """Retorna ([[(lon, lat), ...] para cada <coordinates>], altitude)."""
    root = ET.parse(arquivo).getroot()
    trechos = [p for p in (ler_coordenadas(c) for c in root.iter(KML + 'coordinates')) if p]
    alt = root.find('.//' + KML + 'altitude')
    if not trechos or alt is None:
        sys.exit("KML sem <coordinates> ou sem <altitude>.")
    return trechos, float(alt.text)


def ler_placemarks(arquivo):
    """Retorna [{'nome', 'pontos', 'altitude'}] para cada Placemark do KML."""
    root = ET.parse(arquivo).getroot()
    placemarks = []
    for pm in root.iter(KML + 'Placemark'):
        pontos = []
        for coord in pm.iter(KML + 'coordinates'):
            pontos += ler_coordenadas(coord)
        alt = pm.find('.//' + KML + 'altitude')
        placemarks.append({'nome': pm.findtext(KML + 'name', default=''),
                           'pontos': pontos,
                           'altitude': float(alt.text) if alt is not None else None})
    return placemarks


def para_metros(pontos, origem):
    lon0, lat0 = origem
    k = METROS_GRAU * math.cos(math.radians(lat0))
    return [((lon - lon0) * k, (lat - lat0) * METROS_GRAU) for lon, lat in pontos]


def para_graus(pontos, origem):
    lon0, lat0 = origem
    k = METROS_GRAU * math.cos(math.radians(lat0))
    return [(lon0 + x / k, lat0 + y / METROS_GRAU) for x, y in pontos]


def distancias(pontos):
    xy = para_metros(pontos, pontos[0])
    return [math.dist(xy[i], xy[i+1]) for i in range(len(xy)-1)]


def calcular_velocidade(pontos):
    """Maior velocidade que respeita INTERVALO_MIN_FOTO no trecho mais curto."""
    ds = [d for d in distancias(pontos) if d > 0]
    if not ds:
        return VELOCIDADE_MAX
    velo = min(ds) / INTERVALO_MIN_FOTO // 0.1 / 10
    return max(VELOCIDADE_MIN, min(VELOCIDADE_MAX, velo))


def por_waypoint(valor, n):
    """Aceita um valor único ou uma lista com um valor por waypoint."""
    return valor if isinstance(valor, list) else [valor] * n


def resumo(pontos, velocidade):
    velocidades = por_waypoint(velocidade, len(pontos))
    ds = distancias(pontos)
    if min(velocidades) == max(velocidades):
        print("Velocidade: {} m/s".format(velocidades[0]))
    else:
        print("Velocidade: {} a {} m/s".format(min(velocidades), max(velocidades)))
    print("Distancia: {} metros".format(math.ceil(sum(ds))))
    tempo = math.ceil(sum(d / v for d, v in zip(ds, velocidades)))
    print("Tempo: {} min. {:02} seg.".format(tempo // 60, tempo % 60))
    print("Fotos: {}".format(len(pontos)))


def rotaciona(pontos, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pontos]


def corta(poligono, limite, acima):
    # Sutherland-Hodgman: mantém a parte do polígono com y >= limite (acima) ou y <= limite
    dentro = (lambda p: p[1] >= limite) if acima else (lambda p: p[1] <= limite)
    saida = []
    for i in range(len(poligono)):
        a, b = poligono[i-1], poligono[i]
        if dentro(a) != dentro(b):
            t = (limite - a[1]) / (b[1] - a[1])
            saida.append((a[0] + t * (b[0] - a[0]), limite))
        if dentro(b):
            saida.append(b)
    return saida


def gerar_grade(area, altitude):
    """Waypoints de foto em zigue-zague cobrindo a área [(lon, lat), ...] na altitude dada."""
    if len(area) > 1 and area[0] == area[-1]:
        area = area[:-1]
    if len(area) < 3:
        sys.exit("A area precisa ter pelo menos 3 vertices.")

    # Mavic 3 (Resolution: 5280x3956, Sensor Size: 17.3 x 13)
    GSD = ((altitude * 100) * 17.3) / (5280 * 24)
    GSD = math.ceil(GSD / 0.01) / 100
    print("GSD: {} cm/px".format(GSD))
    verDis30 = 1186.8 * GSD / 100               # distancia maxima entre fotos na faixa (m)
    lateral70 = math.floor(3696 * GSD / 100)    # distancia maxima entre faixas (m)
    print("Distancia entre fotos: {:.1f} m".format(verDis30))

    # Coordenadas locais em metros
    origem = area[0]
    xy = para_metros(area, origem)

    # Direcao das faixas: paralela à aresta que deixa a area mais "estreita" (menos faixas)
    ang = 0
    menor = math.inf
    for i in range(len(xy)):
        a, b = xy[i-1], xy[i]
        angAresta = math.atan2(b[1] - a[1], b[0] - a[0])
        ys = [p[1] for p in rotaciona(xy, -angAresta)]
        if max(ys) - min(ys) < menor:
            menor = max(ys) - min(ys)
            ang = angAresta
    rot = rotaciona(xy, -ang)
    print("Direcao das faixas: {:.1f} graus".format(math.degrees(ang)))

    ymin = min(p[1] for p in rot)
    ymax = max(p[1] for p in rot)
    nSeg = max(1, math.ceil((ymax - ymin) / lateral70))
    passo = (ymax - ymin) / nSeg
    print("Faixas: {} (espacamento {:.1f} m)".format(nSeg + 1, passo))

    faixas = []
    for k in range(nSeg + 1):
        y = ymin + k * passo
        # Extensao da area na faixa de cobertura desta linha (meio espacamento para cada lado)
        faixa = corta(corta(rot, y - passo / 2, True), y + passo / 2, False)
        if not faixa:
            continue
        xa = min(p[0] for p in faixa)
        xb = max(p[0] for p in faixa)
        n = math.ceil((xb - xa) / verDis30)
        if n == 0:
            linha = [((xa + xb) / 2, y)]
        else:
            linha = [(xa + (xb - xa) * j / n, y) for j in range(n + 1)]
        faixas.append(linha)

    # Zigue-zague
    pontos = []
    for k, linha in enumerate(faixas):
        pontos += linha if k % 2 == 0 else linha[::-1]

    return para_graus(rotaciona(pontos, ang), origem)


def contexto_ssl():
    """O Python do instalador python.org no macOS vem sem certificados: usa certifi ou os do sistema."""
    contexto = ssl.create_default_context()
    if contexto.cert_store_stats()['x509_ca'] == 0:
        try:
            import certifi
            contexto.load_verify_locations(certifi.where())
        except ImportError:
            if os.path.exists('/etc/ssl/cert.pem'):
                contexto.load_verify_locations('/etc/ssl/cert.pem')
    return contexto


def consultar_opentopodata(locais):
    url = OPENTOPODATA + '?locations=' + '|'.join(locais)
    contexto = contexto_ssl()
    erro = None
    for tentativa in range(5):
        try:
            with urllib.request.urlopen(url, timeout=30, context=contexto) as r:
                dados = json.load(r)
            if dados.get('status') == 'OK':
                return [res['elevation'] for res in dados['results']]
            erro = dados.get('error')
        except urllib.error.URLError as e:
            if isinstance(e.reason, ssl.SSLCertVerificationError):
                sys.exit("Falha de certificado SSL: {}\nRode \"/Applications/Python 3.x/Install Certificates.command\" "
                         "ou 'pip3 install certifi'.".format(e.reason))
            erro = e
        except (OSError, ValueError) as e:
            erro = e
        time.sleep(2 * (tentativa + 1))
    sys.exit("Falha ao consultar o OpenTopoData: {}".format(erro))


def altitudes_terreno(pontos):
    """Elevação do terreno (m) em cada ponto (OpenTopoData SRTM 30 m), com cache local."""
    try:
        with open(CACHE_TERRENO) as file:
            cache = json.load(file)
    except (OSError, ValueError):
        cache = {}
    chaves = ['{:.6f},{:.6f}'.format(lat, lon) for lon, lat in pontos]
    faltam = list(dict.fromkeys(k for k in chaves if k not in cache))
    for i in range(0, len(faltam), 100):
        if i:
            time.sleep(1.1)     # limite do servidor publico: 1 requisicao/s
        print("Consultando terreno: {}/{} pontos".format(min(i + 100, len(faltam)), len(faltam)))
        lote = faltam[i:i+100]
        for k, z in zip(lote, consultar_opentopodata(lote)):
            cache[k] = z
    if faltam:
        with open(CACHE_TERRENO, 'w') as file:
            json.dump(cache, file)
    semDados = sum(1 for k in chaves if cache[k] is None)
    if semDados:
        print("ATENCAO: {} pontos sem dados de terreno (ex.: agua), considerados a 0 m".format(semDados))
    return [cache[k] if cache[k] is not None else 0.0 for k in chaves]


HEAD = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>15</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{drone}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
      </wpml:droneInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>0</wpml:distance>
      <wpml:duration>0</wpml:duration>
      <wpml:autoFlightSpeed>{velocidade}</wpml:autoFlightSpeed>
"""

PLACEMARK = """      <Placemark>
        <Point>
          <coordinates>
            {lon},{lat}
          </coordinates>
        </Point>
        <wpml:index>{index}</wpml:index>
        <wpml:executeHeight>{altitude}</wpml:executeHeight>
        <wpml:waypointSpeed>{velocidade}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingAngleEnable>1</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>{curva}</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>0</wpml:useStraightLine>
{acoes}      </Placemark>
"""

GRUPO = """        <wpml:actionGroup>
          <wpml:actionGroupId>{grupo}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{inicio}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{fim}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>{modo}</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>{gatilho}</wpml:actionTriggerType>
          </wpml:actionTrigger>
{acoes}        </wpml:actionGroup>
"""

GIMBAL_90 = """          <wpml:action>
            <wpml:actionId>{id}</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalHeadingYawBase>aircraft</wpml:gimbalHeadingYawBase>
              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
              <wpml:gimbalPitchRotateAngle>-90.0</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>
              <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
"""

GIMBAL_90_TRECHO = """          <wpml:action>
            <wpml:actionId>{id}</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalEvenlyRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalPitchRotateAngle>-90.0</wpml:gimbalPitchRotateAngle>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
"""

FOTO = """          <wpml:action>
            <wpml:actionId>{id}</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
"""

TAIL = """    </Folder>
  </Document>
</kml>
"""


def escrever_missao(pontos, altitude, velocidade, arquivo='waylines.wpml'):
    """Gera a missão com gimbal a -90 e uma foto em cada waypoint.
    altitude e velocidade podem ser um valor único ou uma lista com um valor por waypoint."""
    alturas = por_waypoint(altitude, len(pontos))
    velocidades = por_waypoint(velocidade, len(pontos))
    grupo = 0
    acao = 0
    ultimo = len(pontos) - 1
    with open(arquivo, 'w') as file:
        file.write(HEAD.format(drone=DRONE_ENUM, velocidade=max(velocidades)))
        for i, (lon, lat) in enumerate(pontos):
            acoes = ''
            if i == 0:
                # Primeiro waypoint: posiciona o gimbal antes da primeira foto
                acoes += GRUPO.format(grupo=grupo, inicio=i, fim=i, modo='sequence', gatilho='reachPoint',
                                      acoes=GIMBAL_90.format(id=acao) + FOTO.format(id=acao+1))
                grupo += 1
                acao += 2
            else:
                acoes += GRUPO.format(grupo=grupo, inicio=i, fim=i, modo='parallel', gatilho='reachPoint',
                                      acoes=FOTO.format(id=acao))
                grupo += 1
                acao += 1
            if i < ultimo:
                # Mantém o gimbal a -90 no trecho até o próximo waypoint
                acoes += GRUPO.format(grupo=grupo, inicio=i, fim=i+1, modo='parallel', gatilho='betweenAdjacentPoints',
                                      acoes=GIMBAL_90_TRECHO.format(id=acao))
                grupo += 1
                acao += 1
            if i == 0 or i == ultimo:
                curva = 'toPointAndStopWithDiscontinuityCurvature'
            else:
                curva = 'toPointAndPassWithContinuityCurvature'
            file.write(PLACEMARK.format(lon=lon, lat=lat, index=i, altitude='{:g}'.format(alturas[i]),
                                        velocidade=velocidades[i], curva=curva, acoes=acoes))
        file.write(TAIL)
    print("Missao gravada em {}".format(arquivo))
