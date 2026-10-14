import argparse
import math
import os
import re
import sys
import warnings

import numpy as np
import rasterio
from rasterio.transform import Affine
from rasterio.errors import NotGeoreferencedWarning
from rasterio.warp import Resampling, reproject, transform as transformar

warnings.filterwarnings('ignore', category=NotGeoreferencedWarning)   # o JPG de entrada nao tem georreferencia

# Uso: python foto_geotiff.py pasta_fotos [pasta_saida] [--agl 120] [--yaw gimbal]
# Gera um GeoTIFF (mesmo nome da foto, extensao .tif) para cada JPG tirado com a camera
# apontada para baixo, usando a posicao, altura e direcao gravadas pelo drone (EXIF/XMP DJI).

DIAGONAL_35MM = math.hypot(36, 24)  # mm, diagonal do quadro 35 mm
INCLINACAO_MAX = 10.0               # graus fora do nadir aceitos (pitch entre -80 e -90)
QUALIDADE_JPEG = 90


def ler_xmp(arquivo):
    """Retorna {'GpsLatitude': '...', ...} com as tags drone-dji do XMP."""
    with open(arquivo, 'rb') as f:
        dados = f.read(512 * 1024).decode('latin-1')
    tags = dict(re.findall(r'drone-dji:(\w+)="([^"]*)"', dados))
    tags.update(re.findall(r'<drone-dji:(\w+)>([^<]*)</drone-dji', dados))
    return tags


def exif_graus(valor, ref):
    """'(22) (53) (6.27)' + 'S' -> -22.885..."""
    g, m, s = (float(x) for x in re.findall(r'[-\d.]+', valor))
    return -(g + m / 60 + s / 3600) if ref.strip() in ('S', 'W') else g + m / 60 + s / 3600


def exif_numero(valor):
    return float(re.findall(r'[-\d.]+', valor)[0])


def zona_utm(lon, lat):
    zona = int((lon + 180) // 6) + 1
    return 'EPSG:{}'.format((32700 if lat < 0 else 32600) + zona)


def parametros(arquivo, src, agl, fonte_yaw):
    """Posicao, altura, GSD (m/pixel) e direcao da camera para uma foto."""
    xmp = ler_xmp(arquivo)
    exif = src.tags()
    if 'GpsLatitude' in xmp:
        lat = float(xmp['GpsLatitude'])
        lon = float(xmp.get('GpsLongitude', xmp.get('GpsLongtitude')))   # alguns modelos gravam "Longtitude"
    elif 'EXIF_GPSLatitude' in exif:
        lat = exif_graus(exif['EXIF_GPSLatitude'], exif['EXIF_GPSLatitudeRef'])
        lon = exif_graus(exif['EXIF_GPSLongitude'], exif['EXIF_GPSLongitudeRef'])
    else:
        raise ValueError("sem coordenadas GPS")
    if agl is None:
        if 'RelativeAltitude' not in xmp:
            raise ValueError("sem RelativeAltitude no XMP (use --agl)")
        agl = float(xmp['RelativeAltitude'])
    if agl <= 0:
        raise ValueError("altura {:g} m invalida".format(agl))

    pitch = float(xmp.get('GimbalPitchDegree', -90))
    inclinacao = 90 + pitch
    if abs(inclinacao) > INCLINACAO_MAX:
        raise ValueError("foto obliqua (pitch {:g})".format(pitch))
    # No Mavic 3 com a camera a -90 o GimbalYaw vem ~46 graus deslocado; o FlightYaw (proa do drone)
    # e o que alinha fotos vizinhas, ja que o gimbal praticamente nao gira em yaw
    yaw = float(xmp['GimbalYawDegree' if fonte_yaw == 'gimbal' else 'FlightYawDegree'])

    # Distancia focal em pixels: calibrada (modelos Enterprise) ou pela equivalente 35 mm
    if 'CalibratedFocalLength' in xmp:
        focal = float(xmp['CalibratedFocalLength'])
    else:
        f35 = exif_numero(exif['EXIF_FocalLengthIn35mmFilm'])
        focal = f35 * math.hypot(src.width, src.height) / DIAGONAL_35MM
    focal *= exif_numero(exif.get('EXIF_DigitalZoomRatio', '1')) or 1
    return lon, lat, agl, agl / focal, yaw, inclinacao


def transformada_foto(x, y, largura, altura, gsd, yaw):
    """Affine pixel -> UTM da foto girada: o topo da imagem aponta para o yaw (horario a partir do norte)."""
    s, c = math.sin(math.radians(yaw)), math.cos(math.radians(yaw))
    a, d = gsd * c, -gsd * s      # coluna +1: para a direita da imagem
    b, e = -gsd * s, -gsd * c     # linha +1: para baixo da imagem
    return Affine(a, b, x - a * largura / 2 - b * altura / 2,
                  d, e, y - d * largura / 2 - e * altura / 2)


def converter(arquivo, saida, agl=None, fonte_yaw='flight'):
    with rasterio.open(arquivo) as src:
        lon, lat, agl, gsd, yaw, inclinacao = parametros(arquivo, src, agl, fonte_yaw)
        imagem = src.read()
        largura, altura = src.width, src.height

    crs = zona_utm(lon, lat)
    (x,), (y,) = transformar('EPSG:4326', crs, [lon], [lat])
    # Camera levemente fora do nadir: o centro da foto fica deslocado para a frente
    deslocamento = agl * math.tan(math.radians(inclinacao))
    x += deslocamento * math.sin(math.radians(yaw))
    y += deslocamento * math.cos(math.radians(yaw))
    origem = transformada_foto(x, y, largura, altura, gsd, yaw)

    # GeoTIFF alinhado ao norte (abre em qualquer SIG), com mascara fora da foto girada
    cantos = [origem * p for p in ((0, 0), (largura, 0), (0, altura), (largura, altura))]
    xmin, xmax = min(p[0] for p in cantos), max(p[0] for p in cantos)
    ymin, ymax = min(p[1] for p in cantos), max(p[1] for p in cantos)
    destino = Affine(gsd, 0, xmin, 0, -gsd, ymax)
    w, h = math.ceil((xmax - xmin) / gsd), math.ceil((ymax - ymin) / gsd)

    rgb = np.zeros((imagem.shape[0], h, w), np.uint8)
    reproject(imagem, rgb, src_transform=origem, src_crs=crs, dst_transform=destino, dst_crs=crs,
              resampling=Resampling.cubic)
    mascara = np.zeros((h, w), np.uint8)
    reproject(np.full((altura, largura), 255, np.uint8), mascara, src_transform=origem, src_crs=crs,
              dst_transform=destino, dst_crs=crs, resampling=Resampling.nearest)

    perfil = dict(driver='GTiff', width=w, height=h, count=rgb.shape[0], dtype='uint8', crs=crs,
                  transform=destino, tiled=True, blockxsize=512, blockysize=512)
    if rgb.shape[0] == 3:
        perfil.update(compress='JPEG', photometric='YCBCR', jpeg_quality=QUALIDADE_JPEG)
    else:
        perfil.update(compress='DEFLATE')
    with rasterio.open(saida, 'w', **perfil) as dst:
        dst.write(rgb)
        dst.write_mask(mascara)
        dst.build_overviews([2, 4, 8, 16], Resampling.average)
    return lon, lat, agl, gsd, yaw


def main():
    parser = argparse.ArgumentParser(description="Gera GeoTIFFs a partir de fotos nadir de drone DJI.")
    parser.add_argument('fotos', help="pasta com os JPG (ou um unico arquivo)")
    parser.add_argument('saida', nargs='?', default='geotiff', help="pasta de saida (padrao: ./geotiff)")
    parser.add_argument('--agl', type=float,
                        help="altura sobre o solo em m (padrao: RelativeAltitude, altura sobre a decolagem)")
    parser.add_argument('--yaw', choices=('flight', 'gimbal'), default='flight',
                        help="direcao usada para girar a foto (padrao: flight)")
    args = parser.parse_args()

    if os.path.isdir(args.fotos):
        fotos = sorted(os.path.join(args.fotos, f) for f in os.listdir(args.fotos)
                       if f.lower().endswith(('.jpg', '.jpeg')) and not f.startswith('.'))
    else:
        fotos = [args.fotos]
    if not fotos:
        sys.exit("Nenhum JPG em {}".format(args.fotos))
    os.makedirs(args.saida, exist_ok=True)

    geradas = 0
    # GDAL_PAM_ENABLED=NO: nao cria .aux.xml ao lado das fotos (ex.: no cartao do drone)
    with rasterio.Env(GDAL_PAM_ENABLED='NO', GDAL_TIFF_INTERNAL_MASK=True):
        for foto in fotos:
            nome = os.path.splitext(os.path.basename(foto))[0] + '.tif'
            try:
                lon, lat, agl, gsd, yaw = converter(foto, os.path.join(args.saida, nome), args.agl, args.yaw)
            except (ValueError, KeyError) as erro:
                print("{}: ignorada ({})".format(os.path.basename(foto), erro))
                continue
            geradas += 1
            print("{}: {:.6f}, {:.6f}  {:.1f} m  GSD {:.1f} cm  yaw {:+.1f}".format(
                nome, lat, lon, agl, gsd * 100, yaw))
    print("{} de {} fotos convertidas em {}".format(geradas, len(fotos), os.path.abspath(args.saida)))


if __name__ == '__main__':
    main()
