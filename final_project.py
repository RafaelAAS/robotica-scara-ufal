import math
import numpy as np

# -----------------------------
# Parâmetros do robô
# -----------------------------

L1 = 0.20          # elo 1
L2 = 0.15          # elo 2

Z_HOME = 0.110     # TCP com j3=0
CURSO_J3 = 0.300

VEL = 0.15         # m/s
DT = 0.05          # passo de integração

# ============================================================
# Classe DH
# ============================================================

class Link:

    def __init__(self,theta,alpha,a,d):

        self.theta=theta
        self.alpha=alpha
        self.a=a
        self.d=d


def T(link):

    ct=math.cos(link.theta)
    st=math.sin(link.theta)

    ca=math.cos(link.alpha)
    sa=math.sin(link.alpha)

    return np.array([

        [ct,-st*ca, st*sa, link.a*ct],

        [st, ct*ca,-ct*sa, link.a*st],

        [0,     sa,    ca, link.d],

        [0,      0,     0,      1]

    ])


# ============================================================
# Cinemática Direta
# ============================================================

def FK(q1,q2,q3):

    A1=Link(q1,0,L1,0)
    A2=Link(q2,0,L2,0)

    T03=T(A1)@T(A2)

    T03[2,3]=Z_HOME-q3

    return T03


# ============================================================
# Cinemática Inversa
# ============================================================

def IK(x,y,z,phi):

    r2=x*x+y*y

    c2=(r2-L1*L1-L2*L2)/(2*L1*L2)

    c2=max(-1,min(1,c2))

    q2=math.acos(c2)

    beta=math.atan2(

        L2*math.sin(q2),

        L1+L2*math.cos(q2)

    )

    q1=math.atan2(y,x)-beta

    q3=max(

        0,

        min(

            CURSO_J3,

            Z_HOME-z

        )

    )

    q4=phi-q1-q2

    return q1,q2,q3,q4


# ============================================================
# Interpolação
# ============================================================

def lerp(a,b,t):

    return a+(b-a)*t


def interpPose(A,B,t):

    return (

        lerp(A[0],B[0],t),

        lerp(A[1],B[1],t),

        lerp(A[2],B[2],t),

        lerp(A[3],B[3],t)

    )


# ============================================================
# Envia posição ao robô
# ============================================================

def enviarPose(p):

    q1,q2,q3,q4=IK(*p)

    sim.setJointTargetPosition(self.j1,q1)
    sim.setJointTargetPosition(self.j2,q2)
    sim.setJointTargetPosition(self.j3,q3)
    sim.setJointTargetPosition(self.j4,q4)

    self.q=(q1,q2,q3,q4)


# ============================================================
# Lê pose do TCP
# ============================================================

def getTCP():

    pos=sim.getObjectPosition(

        self.tcp,

        sim.handle_world

    )

    ang=sim.getObjectOrientation(

        self.tcp,

        sim.handle_world

    )

    return (

        pos[0],

        pos[1],

        pos[2],

        ang[2]

    )


# ============================================================
# Inicialização
# ============================================================

def sysCall_init():

    global sim
    global self

    sim=require('sim')

    class Robot:
        pass

    self=Robot()

    # -----------------------------
    # Objetos
    # -----------------------------

    self.j1=sim.getObject('/Base/j1')

    self.j2=sim.getObject('/Base/j1/elo1/j2')

    self.j3=sim.getObject('/Base/j1/elo1/j2/elo2/j3')

    self.j4=sim.getObject('/Base/j1/elo1/j2/elo2/j3/elo3/j4')

    self.tcp=sim.getObject(
        '/Base/j1/elo1/j2/elo2/j3/elo3/j4/ferramenta/tcp'
    )

    self.carga=sim.getObject('/Carga')

    # -----------------------------
    # Home
    # -----------------------------

    self.home=(

        0.35,

        0.00,

        0.11,

        0.0

    )

    # posição de entrega

    self.destino=(

        0.20,

        -0.12,

        0.03,

        math.pi/2

    )

    # -----------------------------
    # Variáveis do controlador
    # -----------------------------

    self.estado=0

    self.traj=[]

    self.passo=0

    self.t=0.0

    self.q=(0,0,0,0)

    self.poseAtual=self.home

    self.poseDestino=self.home

    # posição real da carga

    p=sim.getObjectPosition(

        self.carga,

        sim.handle_world

    )

    self.pick=(

        p[0],

        p[1],

        p[2]+0.010,

        0.0

    )

    self.pickAlto=(

        p[0],

        p[1],

        p[2]+0.060,

        0.0

    )

    self.drop=(

        self.destino[0],

        self.destino[1],

        self.destino[2]+0.010,

        math.pi/2

    )

    self.dropAlto=(

        self.destino[0],

        self.destino[1],

        self.destino[2]+0.060,

        math.pi/2

    )

    print('SCARA inicializado.')
    
    iniciarControlador()

# -----------------------------
# Estados
# -----------------------------

HOME            = 0
IR_PICK         = 1
DESCER_PICK     = 2
PEGAR           = 3
SUBIR_PICK      = 4
IR_DROP         = 5
DESCER_DROP     = 6
SOLTAR          = 7
SUBIR_DROP      = 8
RETORNAR_HOME   = 9


# ============================================================
# Planejamento de trajetória
# ============================================================

def criarTrajetoria(origem,destino):

    dx=destino[0]-origem[0]
    dy=destino[1]-origem[1]
    dz=destino[2]-origem[2]

    dist=math.sqrt(dx*dx+dy*dy+dz*dz)

    n=max(2,int(dist/(VEL*DT)))

    self.traj=[]

    for i in range(n+1):

        t=i/n

        self.traj.append(

            interpPose(

                origem,

                destino,

                t

            )

        )

    self.passo=0


# ============================================================
# Executa um ponto da trajetória
# ============================================================

def executarTrajetoria():

    if self.passo>=len(self.traj):

        return True

    pose=self.traj[self.passo]

    enviarPose(pose)

    self.poseAtual=pose

    self.passo+=1

    return False


# ============================================================
# Atualiza posição da carga
# ============================================================

def atualizarCarga():

    p=sim.getObjectPosition(

        self.carga,

        sim.handle_world

    )

    self.pick=(

        p[0],

        p[1],

        p[2]+0.010,

        0.0

    )

    self.pickAlto=(

        p[0],

        p[1],

        p[2]+0.060,

        0.0

    )


# ============================================================
# Ventosa
# ============================================================

def pegarObjeto():

    sim.setObjectParent(

        self.carga,

        self.tcp,

        True

    )


def soltarObjeto():

    sim.setObjectParent(

        self.carga,

        sim.handle_world,

        True

    )


# ============================================================
# Inicia movimento
# ============================================================

def moverPara(destino):

    criarTrajetoria(

        getTCP(),

        destino

    )


# ============================================================
# Avança estado
# ============================================================

def proximoEstado():

    self.estado+=1


# ============================================================
# Volta para HOME
# ============================================================

def reiniciarCiclo():

    atualizarCarga()

    self.estado=HOME


# ============================================================
# Verificação FK
# ============================================================

def verificarFK():

    q1,q2,q3,q4=self.q

    T=FK(

        q1,

        q2,

        q3

    )

    p=T[0:3,3]

    print(

        "FK:",

        round(p[0],3),

        round(p[1],3),

        round(p[2],3)

    )


# ============================================================
# Inicialização da sequência
# (chamar no final do sysCall_init)
# ============================================================

def iniciarControlador():

    atualizarCarga()

    self.estado=HOME

    self.poseAtual=self.home

    self.poseDestino=self.home

    self.traj=[]

    self.passo=0

    moverPara(self.home)

    print("Máquina de estados pronta.")
    

def sysCall_actuation():

    # --------------------------------------------------------
    # HOME
    # --------------------------------------------------------

    if self.estado==HOME:

        if executarTrajetoria():

            moverPara(

                self.pickAlto

            )

            proximoEstado()


    # --------------------------------------------------------
    # IR ACIMA DA CARGA
    # --------------------------------------------------------

    elif self.estado==IR_PICK:

        if executarTrajetoria():

            moverPara(

                self.pick

            )

            proximoEstado()


    # --------------------------------------------------------
    # DESCER
    # --------------------------------------------------------

    elif self.estado==DESCER_PICK:

        if executarTrajetoria():

            proximoEstado()


    # --------------------------------------------------------
    # PEGAR
    # --------------------------------------------------------

    elif self.estado==PEGAR:

        pegarObjeto()

        moverPara(

            self.pickAlto

        )

        proximoEstado()


    # --------------------------------------------------------
    # SUBIR
    # --------------------------------------------------------

    elif self.estado==SUBIR_PICK:

        if executarTrajetoria():

            moverPara(

                self.dropAlto

            )

            proximoEstado()


    # --------------------------------------------------------
    # IR PARA ENTREGA
    # --------------------------------------------------------

    elif self.estado==IR_DROP:

        if executarTrajetoria():

            moverPara(

                self.drop

            )

            proximoEstado()


    # --------------------------------------------------------
    # DESCER PARA ENTREGA
    # --------------------------------------------------------

    elif self.estado==DESCER_DROP:

        if executarTrajetoria():

            proximoEstado()


    # --------------------------------------------------------
    # SOLTAR
    # --------------------------------------------------------

    elif self.estado==SOLTAR:

        soltarObjeto()

        moverPara(

            self.dropAlto

        )

        proximoEstado()


    # --------------------------------------------------------
    # SUBIR
    # --------------------------------------------------------

    elif self.estado==SUBIR_DROP:

        if executarTrajetoria():

            moverPara(

                self.home

            )

            proximoEstado()


    # --------------------------------------------------------
    # RETORNO HOME
    # --------------------------------------------------------

    elif self.estado==RETORNAR_HOME:

        if executarTrajetoria():

            reiniciarCiclo()

            moverPara(

                self.home

            )



    # --------------------------------------------------------
    # Atualização da FK (opcional)
    # --------------------------------------------------------

    # comentar se não quiser mensagens no console

    # verificarFK()
    
def resetCarga():

    sim.setObjectParent(
        self.carga,
        sim.handle_world,
        True
    )

    sim.setObjectPosition(
        self.carga,
        sim.handle_world,
        [0.20, 0.12, 0.0125]
    )

    sim.setObjectOrientation(
        self.carga,
        sim.handle_world,
        [0.0, 0.0, 0.0]
    )


# ------------------------------------------------------------
# Reinicia o controlador
# ------------------------------------------------------------

def resetControlador():

    atualizarCarga()

    self.estado = HOME

    self.traj = []

    self.passo = 0

    self.poseAtual = self.home

    self.poseDestino = self.home

    moverPara(self.home)


# ------------------------------------------------------------
# Monitoramento
# ------------------------------------------------------------

def sysCall_sensing():

    # Atualiza continuamente a posição da carga
    # enquanto ela ainda não foi capturada

    if self.estado < PEGAR:

        atualizarCarga()

    # Descomente para depuração

    # verificarFK()


# ------------------------------------------------------------
# Limpeza da simulação
# ------------------------------------------------------------

def sysCall_cleanup():

    try:

        resetCarga()

        resetControlador()

    except Exception as e:

        print(e)

    print("-------------------------------------")
    print(" Controlador SCARA encerrado")
    print("-------------------------------------")
