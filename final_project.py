import math

# ==========================================================
# PARÂMETROS DO ROBÔ
# ==========================================================

L1 = 0.20          # elo 1
L2 = 0.15          # elo 2
Z_HOME = 0.11      # altura TCP com j3=0
CURSO = 0.30       # curso do prismático

# ==========================================================
# CINEMÁTICA DIRETA
# ==========================================================

def FK(q1,q2,q3,q4):

    x = L1*math.cos(q1) + L2*math.cos(q1+q2)

    y = L1*math.sin(q1) + L2*math.sin(q1+q2)

    z = Z_HOME - q3

    phi = q1 + q2 + q4

    return [x,y,z,phi]

# ==========================================================
# CINEMÁTICA INVERSA
# ==========================================================

def IK(x,y,z,phi=0):

    r2 = x*x + y*y

    c2 = (r2-L1*L1-L2*L2)/(2*L1*L2)

    c2 = max(-1,min(1,c2))

    s2 = math.sqrt(1-c2*c2)

    q2 = math.atan2(s2,c2)

    k1 = L1 + L2*c2
    k2 = L2*s2

    q1 = math.atan2(y,x)-math.atan2(k2,k1)

    q3 = max(0,min(CURSO,Z_HOME-z))

    q4 = phi-q1-q2

    return q1,q2,q3,q4

# ==========================================================
# MOVE O ROBÔ
# ==========================================================

def moveTo(x,y,z,phi=0):

    q1,q2,q3,q4 = IK(x,y,z,phi)

    sim.setJointTargetPosition(self.j1,q1)
    sim.setJointTargetPosition(self.j2,q2)
    sim.setJointTargetPosition(self.j3,q3)
    sim.setJointTargetPosition(self.j4,q4)
    
def chegou(x,y,z):

    tcp = sim.getObjectPosition(self.tcp,sim.handle_world)

    d = math.sqrt(
        (tcp[0]-x)**2 +
        (tcp[1]-y)**2 +
        (tcp[2]-z)**2
    )

    return d < self.tol

# ==========================================================
# INIT
# ==========================================================

def sysCall_init():

    global sim

    sim = require('sim')

    self.j1 = sim.getObject('/Base/j1')
    self.j2 = sim.getObject('/Base/j1/elo1/j2')
    self.j3 = sim.getObject('/Base/j1/elo1/j2/elo2/j3')
    self.j4 = sim.getObject('/Base/j1/elo1/j2/elo2/j3/elo3/j4')

    self.tcp = sim.getObject('/Base/j1/elo1/j2/elo2/j3/elo3/j4/ferramenta/tcp')

    self.carga = sim.getObject('/Carga')

    print("SCARA iniciado")
    
    # ponto de entrega
    self.destino = [0.20, -0.12, 0.02]

    # alturas
    self.zSegura = 0.08
    self.zColeta = 0.02

    # estados
    self.estado = 0

    # ferramenta
    self.ferramenta = sim.getObject('/Base/j1/elo1/j2/elo2/j3/elo3/j4/ferramenta')

    # velocidade de aproximação
    self.tol = 0.003
    
# ==========================================================
# LOOP
# ==========================================================

def sysCall_actuation():

    carga = sim.getObjectPosition(self.carga,sim.handle_world)

    # -------------------------
    # Estado 0
    # Vai acima da carga
    # -------------------------

    if self.estado == 0:

        moveTo(carga[0],carga[1],self.zSegura)

        if chegou(carga[0],carga[1],self.zSegura):

            self.estado = 1

    # -------------------------
    # Estado 1
    # Desce
    # -------------------------

    elif self.estado == 1:

        moveTo(carga[0],carga[1],self.zColeta)

        if chegou(carga[0],carga[1],self.zColeta):

            self.estado = 2

    # -------------------------
    # Estado 2
    # Pega
    # -------------------------

    elif self.estado == 2:

        sim.setObjectParent(
            self.carga,
            self.ferramenta,
            True
        )

        self.estado = 3

    # -------------------------
    # Estado 3
    # Sobe
    # -------------------------

    elif self.estado == 3:

        moveTo(carga[0],carga[1],self.zSegura)

        if chegou(carga[0],carga[1],self.zSegura):

            self.estado = 4

    # -------------------------
    # Estado 4
    # Vai para entrega
    # -------------------------

    elif self.estado == 4:

        moveTo(
            self.destino[0],
            self.destino[1],
            self.zSegura
        )

        if chegou(
            self.destino[0],
            self.destino[1],
            self.zSegura
        ):

            self.estado = 5

    # -------------------------
    # Estado 5
    # Desce
    # -------------------------

    elif self.estado == 5:

        moveTo(
            self.destino[0],
            self.destino[1],
            self.zColeta
        )

        if chegou(
            self.destino[0],
            self.destino[1],
            self.zColeta
        ):

            self.estado = 6

    # -------------------------
    # Estado 6
    # Solta
    # -------------------------

    elif self.estado == 6:

        sim.setObjectParent(
            self.carga,
            sim.handle_world,
            True
        )

        self.estado = 7

    # -------------------------
    # Estado 7
    # Retorna
    # -------------------------

    elif self.estado == 7:

        moveTo(
            self.destino[0],
            self.destino[1],
            self.zSegura
        )

        if chegou(
            self.destino[0],
            self.destino[1],
            self.zSegura
        ):

            self.estado = 8
    # -------------------------
    # Estado 8
    # Fim ate que a caixa nao esteja na posiçao final
    # -------------------------
    elif self.estado == 8:
        
        carga = sim.getObjectPosition(self.carga,sim.handle_world)
        
        if abs(carga[0] - self.destino[0]) > 0.001 or abs(carga[1] - self.destino[1]) > 0.001:
            self.estado = 0


# ==========================================================
# SENSING
# ==========================================================

def sysCall_sensing():

    tcp = sim.getObjectPosition(
        self.tcp,
        sim.handle_world
    )

    # distância até a carga

    carga = sim.getObjectPosition(
        self.carga,
        sim.handle_world
    )

    dx = tcp[0]-carga[0]
    dy = tcp[1]-carga[1]
    dz = tcp[2]-carga[2]

    d = math.sqrt(dx*dx+dy*dy+dz*dz)

    if d < 0.005:
        print("Contato")

# ==========================================================
# CLEANUP
# ==========================================================

def sysCall_cleanup():
    pass
