import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================================
# Gabarito da Maquete Física
# ============================================================================
L1 = 0.20
L2 = 0.15
Z_REPOUSO = 0.02


def calcular_cinematica_inversa(x, y, z_alvo):
    q3 = Z_REPOUSO - z_alvo
    q3 = max(0.0, min(q3, 0.10))

    r_quadrado = x**2 + y**2
    r = math.sqrt(r_quadrado)

    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError(f"Alvo inacessível! Raio da coordenada: {r:.3f}m")

    cos_q2 = (r_quadrado - L1**2 - L2**2) / (2 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))

    alfa = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    q1 = alfa - beta

    return q1, q2, q3


def enviar_pose_imediata(sim, j1, j2, j3, x, y, z):
    q1, q2, q3 = calcular_cinematica_inversa(x, y, z)
    sim.setJointTargetPosition(j1, q1)
    sim.setJointTargetPosition(j2, q2)
    sim.setJointTargetPosition(j3, q3)


def mover_em_linha_reta(
    sim, j1, j2, j3, x_ini, y_ini, z_ini, x_fim, y_fim, z_fim, duracao=2.0
):
    fps = 40
    total_passos = int(duracao * fps)
    for i in range(total_passos + 1):
        fator = i / total_passos
        x_atual = x_ini + (x_fim - x_ini) * fator
        y_atual = y_ini + (y_fim - y_ini) * fator
        z_atual = z_ini + (z_fim - z_ini) * fator
        enviar_pose_imediata(sim, j1, j2, j3, x_atual, y_atual, z_atual)
        time.sleep(1.0 / fps)


# ============================================================================
# CONEXÃO E ROTEIRO DE HONRA
# ============================================================================
print("Iniciando link de fibra ótica com CoppeliaSim...")
client = RemoteAPIClient(host="172.27.128.1")
sim = client.getObject("sim")

j1 = sim.getObject("/Base/j1")
j2 = sim.getObject("/Base/j1/elo1/j2")
j3 = sim.getObject("/Base/j1/elo1/j2/elo2/j3")
tcp = sim.getObject("/Base/j1/elo1/j2/elo2/j3/elo3/tcp")
carga = sim.getObject("/Carga")

sim.startSimulation()
time.sleep(0.5)

caneta_neon = None

try:
    # 1. POSE INICIAL: Braço totalmente esticado para frente (X = 0.20 + 0.15 = 0.35m)
    X_HOME, Y_HOME, Z_HOME = 0.35, 0.00, 0.05
    enviar_pose_imediata(sim, j1, j2, j3, X_HOME, Y_HOME, Z_HOME)
    time.sleep(1.0)

    print("\n" + "=" * 60)
    print(" ATO I: A LEMNISCATA CARTESIANA (Laser Ciano)")
    print("=" * 60)

    # Injeta a fita ciano de alta persistência na GPU
    caneta_neon = sim.addDrawingObject(
        sim.drawing_linestrip, 4, 0.001, -1, 5000, [0.0, 1.0, 1.0]
    )

    X_CENTRO, Y_CENTRO = 0.22, 0.00
    AMP_X, AMP_Y = 0.025, 0.080
    ALTURA_LASER = 0.04

    mover_em_linha_reta(
        sim, j1, j2, j3, X_HOME, Y_HOME, Z_HOME, X_CENTRO, Y_CENTRO, ALTURA_LASER, 1.5
    )

    passos_inf = 200
    duracao_inf = 8.0
    dt_inf = duracao_inf / passos_inf

    for i in range(passos_inf + 1):
        t = (i / passos_inf) * (2 * math.pi)
        x_alvo = X_CENTRO + AMP_X * math.sin(2 * t)
        y_alvo = Y_CENTRO + AMP_Y * math.sin(t)

        enviar_pose_imediata(sim, j1, j2, j3, x_alvo, y_alvo, ALTURA_LASER)

        # Carimba a coordenada real da ponta do dedo na fita Ciano!
        pos_real = sim.getObjectPosition(tcp, sim.handle_world)
        sim.addDrawingObjectItem(caneta_neon, pos_real)

        time.sleep(dt_inf)

    X_ATUAL, Y_ATUAL, Z_ATUAL = X_CENTRO, Y_CENTRO, ALTURA_LASER
    time.sleep(0.5)

    print("\n" + "=" * 60)
    print(" ATO II: MANIPULAÇÃO INDUSTRIAL COM GABARITO IMPRESSO")
    print("=" * 60)
    print("-> O laser foi cortado. A curva ciano servirá de prova de fundo.")

    P_COLETA = (0.20, 0.12)
    P_ENTREGA = (0.20, -0.12)
    Z_VOO = 0.07
    Z_TETO_CUBO = 0.025

    mover_em_linha_reta(
        sim, j1, j2, j3, X_ATUAL, Y_ATUAL, Z_ATUAL, P_COLETA[0], P_COLETA[1], Z_VOO, 2.0
    )
    mover_em_linha_reta(
        sim, j1, j2, j3, P_COLETA[0], P_COLETA[1], Z_VOO, P_COLETA[0], P_COLETA[1], Z_TETO_CUBO, 1.0
    )
    time.sleep(0.2)

    print("   [SUCÇÃO LIGADA] -> Carga capturada.")
    sim.setObjectParent(carga, tcp, True)

    mover_em_linha_reta(
        sim, j1, j2, j3, P_COLETA[0], P_COLETA[1], Z_TETO_CUBO, P_COLETA[0], P_COLETA[1], Z_VOO, 1.0
    )
    mover_em_linha_reta(
        sim, j1, j2, j3, P_COLETA[0], P_COLETA[1], Z_VOO, P_ENTREGA[0], P_ENTREGA[1], Z_VOO, 3.0
    )
    mover_em_linha_reta(
        sim, j1, j2, j3, P_ENTREGA[0], P_ENTREGA[1], Z_VOO, P_ENTREGA[0], P_ENTREGA[1], Z_TETO_CUBO, 1.0
    )
    time.sleep(0.2)

    print("   [VÁCUO CORTADO] -> Peça no alvo.")
    sim.setObjectParent(carga, sim.handle_world, True)

    mover_em_linha_reta(
        sim, j1, j2, j3, P_ENTREGA[0], P_ENTREGA[1], Z_TETO_CUBO, P_ENTREGA[0], P_ENTREGA[1], Z_VOO, 1.0
    )

    X_ATUAL, Y_ATUAL, Z_ATUAL = P_ENTREGA[0], P_ENTREGA[1], Z_VOO

    # ------------------------------------------------------------------------
    # O GRAN FINALE DA BANCA (Retorno para a pose estendida)
    # ------------------------------------------------------------------------
    print("\nRecolhendo para a pose imponente de repouso estendido...")
    mover_em_linha_reta(
        sim, j1, j2, j3, X_ATUAL, Y_ATUAL, Z_ATUAL, X_HOME, Y_HOME, Z_HOME, 2.0
    )

    print("\n[APRESENTAÇÃO ESTACIONADA]: 5 segundos abertos para fotos da maquete...")
    time.sleep(5.0)

except Exception as e:
    print(f"\n[COLAPSO TÉCNICO]: {e}")

finally:
    print("\nFechando as cortinas...")

    # Evapora a fita ciano e reseta a mesa magicamente
    if caneta_neon is not None:
        sim.addDrawingObjectItem(caneta_neon, None)
        time.sleep(0.1)
        sim.removeDrawingObject(caneta_neon)

    if carga is not None:
        sim.setObjectParent(carga, sim.handle_world, True)
        sim.setObjectPosition(carga, sim.handle_world, [0.20, 0.12, 0.0125])

    sim.stopSimulation()
    print("Mesa limpa. Sistema 100% pronto para a defesa.")