import sys
import missao_wpml as wpml

trechos, altitude = wpml.ler_kml(sys.argv[1])

# Todos os pontos do KML, na ordem, ignorando repetidos em sequencia
pontos = []
for trecho in trechos:
    for p in trecho:
        if not pontos or pontos[-1] != p:
            pontos.append(p)
print("Waypoints: {}".format(len(pontos)))
print("Altitude: {:g} m".format(altitude))

velocidade = wpml.calcular_velocidade(pontos)
wpml.resumo(pontos, velocidade)
wpml.escrever_missao(pontos, altitude, velocidade)
