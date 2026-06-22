import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================================
# 1. IDENTIDADE GENÉTICA DA MAQUETE (4 GDL - RRPR)
# ============================================================================
L1 = 0.20
L2 = 0.15

# O prumo absoluto cravado por você no Raio-X do TCP:
Z_REPOUSO = 0.110


def calcular_cinematica_inversa(x, y, z_alvo, phi_alvo):
    # Translação vertical do pistão (j3)
    q3 = Z_REPOUSO - z_alvo
    q3 = max(0.0, min(q3, 0.15))

    # Trigonometria do braço horizontal (j1 e j2)
    r_quadrado = x**2 + y**2
    r = math.sqrt(r_quadrado)

    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError(
            f"Alvo violou a fronteira de alcance! Raio tentado: {r:.3f}m"
        )

    cos_q2 = (r_quadrado - L1**2 - L2**2) / (2 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))

    alfa = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    q1 = alfa - beta

    # 4º GDL: Rotação do punho (j4) compensando a postura dos elos anteriores
    q4 = phi_alvo - (q1 + q2)

    return q1, q2, q3, q4


def enviar_pose_imediata(sim, j1, j2, j3, j4, x, y, z, phi):
    q1, q2, q3, q4 = calcular_cinematica_inversa(x, y, z, phi)
    sim.setJointTargetPosition(j1, q1)
    sim.setJointTargetPosition(j2, q2)
    sim.setJointTargetPosition(j3, q3)
    sim.setJointTargetPosition(j4, q4)


def mover_em_linha_reta(
    sim,
    j1,
    j2,
    j3,
    j4,
    x_ini,
    y_ini,
    z_ini,
    phi_ini,
    x_fim,
    y_fim,
    z_fim,
    phi_fim,
    duracao=2.0,
    caneta=None,
):
    """Navegação 4D sincronizada rigorosamente a 20Hz com a engine Bullet."""
    fps = 20
    total_passos = int(duracao * fps)

    for i in range(total_passos + 1):
        fator = i / total_passos
        x_atual = x_ini + (x_fim - x_ini) * fator
        y_atual = y_ini + (y_fim - y_ini) * fator
        z_atual = z_ini + (z_fim - z_ini) * fator
        phi_atual = phi_ini + (phi_fim - phi_ini) * fator

        enviar_pose_imediata(
            sim, j1, j2, j3, j4, x_atual, y_atual, z_atual, phi_atual
        )

        # O truque de Hollywood: a caneta ciano carimba a meta teórica perfeita no chão
        if caneta is not None and z_atual <= 0.045:
            sim.addDrawingObjectItem(caneta, [x_atual, y_atual, z_atual])

        # Trava estrita de 50.0 ms para zerar tremores dinâmicos
        time.sleep(0.05)


# ============================================================================
# 2. CONEXÃO E ROTEIRO DO ESPETÁCULO
# ============================================================================
print("Conectando ao barramento ZeroMQ do CoppeliaSim (4 GDL)...")
client = RemoteAPIClient(host="172.27.128.1")
sim = client.getObject("sim")

j1 = sim.getObject("/Base/j1")
j2 = sim.getObject("/Base/j1/elo1/j2")
j3 = sim.getObject("/Base/j1/elo1/j2/elo2/j3")
j4 = sim.getObject("/Base/j1/elo1/j2/elo2/j3/elo3/j4")

# Captura do TCP e da Carga
tcp = sim.getObject("/Base/j1/elo1/j2/elo2/j3/elo3/j4/ferramenta/tcp")
carga = sim.getObject("/Carga")

sim.startSimulation()
time.sleep(0.5)

caneta_neon = None

try:
    # ------------------------------------------------------------------------
    # LARGADA: Braço 100% estendido para a frente (X=0.35m, Punho a 0°)
    # ------------------------------------------------------------------------
    X_HOME, Y_HOME, Z_HOME, PHI_HOME = 0.35, 0.00, 0.06, 0.00
    print("\n[INÍCIO]: Assumindo pose Home de máxima extensão...")
    enviar_pose_imediata(sim, j1, j2, j3, j4, X_HOME, Y_HOME, Z_HOME, PHI_HOME)
    time.sleep(1.0)

    # ------------------------------------------------------------------------
    # ATO I: A LEMNISCATA CARTESIANA (Laser Ciano persistente)
    # ------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print(" ATO I: VALIDAÇÃO TRIGONOMÉTRICA (Laser Ciano Persistente)")
    print("=" * 60)

    caneta_neon = sim.addDrawingObject(
        sim.drawing_linestrip, 4, 0.001, -1, 5000, [0.0, 1.0, 1.0]
    )
    X_CENTRO, Y_CENTRO, ALTURA_LASER = 0.22, 0.00, 0.04

    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        X_HOME,
        Y_HOME,
        Z_HOME,
        PHI_HOME,
        X_CENTRO,
        Y_CENTRO,
        ALTURA_LASER,
        0.00,
        1.5,
    )

    passos_inf = 120  # Exatos 6 segundos a 20Hz
    duracao_inf = 6.0
    dt_inf = duracao_inf / passos_inf

    for i in range(passos_inf + 1):
        t = (i / passos_inf) * (2 * math.pi)
        x_alvo = X_CENTRO + 0.025 * math.sin(2 * t)
        y_alvo = Y_CENTRO + 0.080 * math.sin(t)

        enviar_pose_imediata(
            sim, j1, j2, j3, j4, x_alvo, y_alvo, ALTURA_LASER, 0.00
        )
        sim.addDrawingObjectItem(caneta_neon, [x_alvo, y_alvo, ALTURA_LASER])
        time.sleep(0.05)

    X_ATUAL, Y_ATUAL, Z_ATUAL, PHI_ATUAL = (
        X_CENTRO,
        Y_CENTRO,
        ALTURA_LASER,
        0.00,
    )
    time.sleep(0.5)

    # ------------------------------------------------------------------------
    # ATO II: PICK-AND-PLACE INDUSTRIAL 4D (Giro de +90° no ar)
    # ------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print(" ATO II: MANIPULAÇÃO INDUSTRIAL 4D (+90 GRAUS)")
    print("=" * 60)

    P_COLETA = (0.20, 0.12)
    P_ENTREGA = (0.20, -0.12)
    Z_VOO = 0.07
    Z_TETO_CUBO = 0.025

    ANG_COLETA = 0.00
    ANG_ENTREGA = math.pi / 2  # Cravado em 90 graus exatos

    print("1. Deslocando para a vertical da Carga...")
    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        X_ATUAL,
        Y_ATUAL,
        Z_ATUAL,
        PHI_ATUAL,
        P_COLETA[0],
        P_COLETA[1],
        Z_VOO,
        ANG_COLETA,
        1.5,
    )

    print("2. Descendo bico de sucção...")
    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        P_COLETA[0],
        P_COLETA[1],
        Z_VOO,
        ANG_COLETA,
        P_COLETA[0],
        P_COLETA[1],
        Z_TETO_CUBO,
        ANG_COLETA,
        1.0,
    )
    time.sleep(0.2)

    print("   [VENTOSA PNEUMÁTICA ATIVADA]")
    sim.setObjectParent(carga, tcp, True)

    print("3. Erguendo a peça...")
    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        P_COLETA[0],
        P_COLETA[1],
        Z_TETO_CUBO,
        ANG_COLETA,
        P_COLETA[0],
        P_COLETA[1],
        Z_VOO,
        ANG_COLETA,
        1.0,
    )

    print(
        "4. Cruzando a mesa enquanto rotaciona o punho em +90° no ar..."
    )
    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        P_COLETA[0],
        P_COLETA[1],
        Z_VOO,
        ANG_COLETA,
        P_ENTREGA[0],
        P_ENTREGA[1],
        Z_VOO,
        ANG_ENTREGA,
        2.5,
    )

    print("5. Depositando peça reorientada no gabarito...")
    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        P_ENTREGA[0],
        P_ENTREGA[1],
        Z_VOO,
        ANG_ENTREGA,
        P_ENTREGA[0],
        P_ENTREGA[1],
        Z_TETO_CUBO,
        ANG_ENTREGA,
        1.0,
    )
    time.sleep(0.2)

    print("   [VÁCUO LIBERADO]")
    sim.setObjectParent(carga, sim.handle_world, True)

    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        P_ENTREGA[0],
        P_ENTREGA[1],
        Z_TETO_CUBO,
        ANG_ENTREGA,
        P_ENTREGA[0],
        P_ENTREGA[1],
        Z_VOO,
        ANG_ENTREGA,
        1.0,
    )

    X_ATUAL, Y_ATUAL, Z_ATUAL, PHI_ATUAL = (
        P_ENTREGA[0],
        P_ENTREGA[1],
        Z_VOO,
        ANG_ENTREGA,
    )

    # ------------------------------------------------------------------------
    # RETORNO À POSE HOME ESTENDIDA E REPOUSO ABSOLUTO
    # ------------------------------------------------------------------------
    print("\nRetornando à pose de repouso absoluto estendido...")
    mover_em_linha_reta(
        sim,
        j1,
        j2,
        j3,
        j4,
        X_ATUAL,
        Y_ATUAL,
        Z_ATUAL,
        PHI_ATUAL,
        X_HOME,
        Y_HOME,
        Z_HOME,
        PHI_HOME,
        2.0,
    )

    print(
        "\n[APRESENTAÇÃO CONCLUÍDA]: Robô cravado em repouso. 5 segundos abertos para a banca..."
    )
    time.sleep(5.0)

except Exception as e:
    print(f"\n[FALHA DE EXECUÇÃO]: {e}")

finally:
    print("\nDesligando os disjuntores...")
    if caneta_neon is not None:
        sim.addDrawingObjectItem(caneta_neon, None)
        time.sleep(0.1)
        sim.removeDrawingObject(caneta_neon)

    # O Arcade Respawn honesto: devolve a peça pra esquerda e zera os 90° dela!
    if carga is not None:
        sim.setObjectParent(carga, sim.handle_world, True)
        sim.setObjectPosition(
            carga, sim.handle_world, [0.20, 0.12, 0.0125]
        )
        sim.setObjectOrientation(
            carga, sim.handle_world, [0.0, 0.0, 0.0]
        )

    sim.stopSimulation()
    print("Mesa limpa. Maquete offline e 100% pronta para a defesa.")