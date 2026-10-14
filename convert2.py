import sys
import missao_wpml as wpml

trechos, altitude = wpml.ler_kml(sys.argv[1])
pontos = wpml.gerar_grade(trechos[0], altitude)

velocidade = wpml.calcular_velocidade(pontos)
wpml.resumo(pontos, velocidade)
wpml.escrever_missao(pontos, altitude, velocidade)
