"""Suporte em chapa - Aco 1020 galvanizado, t=2 mm (500 pecas).
Gera: modelo 3D (STEP/STL), desenho 2D (DXF + PDF) e planificacao.
Uso:  python3 gerar.py
"""
import math, os
import cadquery as cq
import ezdxf
from ezdxf.addons.drawing import matplotlib as ezmpl
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS

OUT = os.path.dirname(os.path.abspath(__file__))

# ---------------- parametros ----------------
L, W, H = 160.0, 54.0, 15.0      # externo
T = 2.0                          # espessura
R_EXT = 2.74                     # raio externo das dobras
R_INT = R_EXT - T                # 0.74
GAP_X0, GAP_X1 = 60.0, 100.0     # abertura na face frontal (40 mm)
HOLE_D, HOLE_Z = 5.0, 7.5
HOLES_X = [30.0, 45.0, 115.0, 130.0]
K = 0.33                         # fator K (ajustar ao ferramental)

# ---------------- 3D ----------------
# Bandeja dobrada: fundo + 4 abas dobradas a 90 graus, cantos abertos (alivio).
def flange(length):
    """Aba em coordenadas locais: ao longo de +X, para fora em +Y, tangente do fundo em y=0."""
    c = (cq.Workplane("YZ").center(0, R_EXT).circle(R_EXT).circle(R_INT).extrude(length)
         .intersect(cq.Workplane("XY").box(length, R_EXT, R_EXT, centered=False)))
    wall = cq.Workplane("XY").box(length, T, H - R_EXT, centered=False).translate((0, R_INT, R_EXT))
    return c.union(wall)
BL_, BW_ = L - 2*R_EXT, W - 2*R_EXT
part = cq.Workplane("XY").box(BL_, BW_, T, centered=False).translate((R_EXT, R_EXT, 0))
part = part.union(flange(BL_).translate((R_EXT, W - R_EXT, 0)))                               # fundo (traseira)
part = part.union(flange(BL_).rotate((0,0,0), (0,0,1), 180).translate((L - R_EXT, R_EXT, 0)))  # frente
part = part.union(flange(BW_).rotate((0,0,0), (0,0,1), 90).translate((R_EXT, R_EXT, 0)))       # lateral esq.
part = part.union(flange(BW_).rotate((0,0,0), (0,0,1), -90).translate((L - R_EXT, W - R_EXT, 0)))  # lateral dir.
# abertura de 40 mm so na parede frontal (o fundo continua inteiro)
part = part.cut(cq.Workplane("XY").box(GAP_X1-GAP_X0, T+1, H, centered=False)
                .translate((GAP_X0, -1, R_EXT)))
for x in HOLES_X:
    part = part.cut(cq.Workplane("XZ").center(x, HOLE_Z).circle(HOLE_D/2).extrude(-(T+2)).translate((0, -1, 0)))

cq.exporters.export(part, f"{OUT}/suporte.step")
cq.exporters.export(part, f"{OUT}/suporte.stl", tolerance=0.02, angularTolerance=0.1)
print("volume mm3:", round(part.val().Volume(), 1),
      " massa g:", round(part.val().Volume()*7.87e-3, 1))

# ---------------- projecoes HLR ----------------
def hlr(shape, direction, xdir):
    algo = HLRBRep_Algo(); algo.Add(shape)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*direction), gp_Dir(*xdir))))
    algo.Update(); algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    def lines(comp):
        res = []
        if comp is None or comp.IsNull(): return res
        ex = TopExp_Explorer(comp, TopAbs_EDGE)
        while ex.More():
            c = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
            d = GCPnts_QuasiUniformDeflection(c, 0.01)
            res.append([(d.Value(i).X(), d.Value(i).Y()) for i in range(1, d.NbPoints()+1)])
            ex.Next()
        return res
    vis = lines(h.VCompound()) + lines(h.OutLineVCompound()) + lines(h.Rg1LineVCompound())
    hid = lines(h.HCompound()) + lines(h.OutLineHCompound())
    return vis, hid

shp = part.val().wrapped
# vista frontal (olhando de -Y): x->X, z->Y
front = hlr(shp, (0, -1, 0), (1, 0, 0))
# vista superior (olhando de +Z): x->X, y->Y
top = hlr(shp, (0, 0, 1), (1, 0, 0))
# vista lateral esquerda (1o diedro: projetada a direita da frontal; olhando de -X)
side = hlr(shp, (-1, 0, 0), (0, -1, 0))
iso = hlr(shp, (1, -1, 1), (1, 1, 0))

# ---------------- DXF ----------------
doc = ezdxf.new("R2010", setup=True); doc.units = ezdxf.units.MM
doc.layers.add("VISIVEL", color=7, lineweight=50)
doc.layers.add("OCULTA", color=8, linetype="DASHED", lineweight=25)
doc.layers.add("CENTRO", color=1, linetype="CENTER", lineweight=18)
doc.layers.add("COTA", color=5, lineweight=18)
doc.layers.add("DOBRA", color=3, linetype="DASHED", lineweight=18)
doc.layers.add("TEXTO", color=7, lineweight=25)
doc.layers.add("MOLDURA", color=7, lineweight=70)
msp = doc.modelspace()
ds = doc.dimstyles.get("EZDXF")
ds.dxf.dimtxt = 2.5; ds.dxf.dimasz = 2.2; ds.dxf.dimdec = 2; ds.dxf.dimdsep = ord(",")
ds.dxf.dimexo = 1.0; ds.dxf.dimexe = 1.5; ds.dxf.dimgap = 0.8; ds.dxf.dimclrd = 5; ds.dxf.dimclre = 5; ds.dxf.dimclrt = 5
DIM = dict(dimstyle="EZDXF", dxfattribs={"layer": "COTA"})

def bbox(view):
    pts = [p for l in view[0] for p in l]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)

def place(view, ox, oy, flipy=False, center=False):
    """Coloca a vista com o canto inf. esq. do bbox em (ox,oy) (ou centro, se center=True)."""
    x0, y0, x1, y1 = bbox(view)
    if center: ox -= (x1-x0)/2; oy -= (y1-y0)/2
    for lay, ls in (("VISIVEL", view[0]), ("OCULTA", view[1])):
        for l in ls:
            pts = [(ox+x-x0, oy+((y1-y) if flipy else (y-y0))) for x, y in l]
            msp.add_lwpolyline(pts, dxfattribs={"layer": lay})

def fmt(v): return f"{v:.2f}".replace(".", ",")

def hdim(p1, p2, y, text=None):
    msp.add_linear_dim(base=(p1[0], y), p1=p1, p2=p2, text=text or fmt(abs(p2[0]-p1[0])), **DIM).render()
def vdim(p1, p2, x, text=None):
    msp.add_linear_dim(base=(x, p1[1]), p1=p1, p2=p2, angle=90, text=text or fmt(abs(p2[1]-p1[1])), **DIM).render()
def text(s, x, y, h=3.5, align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER, layer="TEXTO"):
    msp.add_text(s, height=h, dxfattribs={"layer": layer, "style": "OpenSans"}).set_placement((x, y), align=align)
def cline(a, b): msp.add_line(a, b, dxfattribs={"layer": "CENTRO"})

# Folha A3 (420x297), escala 1:1
msp.add_lwpolyline([(0,0),(420,0),(420,297),(0,297)], close=True, dxfattribs={"layer":"MOLDURA"})
msp.add_lwpolyline([(10,10),(410,10),(410,287),(10,287)], close=True, dxfattribs={"layer":"MOLDURA"})

# --- Vista frontal: origem (FX,FY) = canto inf. esq. da peca
FX, FY = 45, 225
place(front, FX, FY)
for x in HOLES_X:
    cline((FX+x-4, FY+HOLE_Z), (FX+x+4, FY+HOLE_Z)); cline((FX+x, FY+HOLE_Z-4), (FX+x, FY+HOLE_Z+4))
YD = FY+H+9
hdim((FX, FY+H), (FX+30, FY+HOLE_Z), YD)
hdim((FX+30, FY+HOLE_Z), (FX+45, FY+HOLE_Z), YD)
hdim((FX+115, FY+HOLE_Z), (FX+130, FY+HOLE_Z), YD)
hdim((FX+130, FY+HOLE_Z), (FX+L, FY+H), YD)
vdim((FX, FY), (FX, FY+HOLE_Z), FX-8)
vdim((FX, FY), (FX, FY+H), FX-17)
a = math.radians(45)
msp.add_diameter_dim_2p(p1=(FX+45-2.5*math.cos(a), FY+HOLE_Z-2.5*math.sin(a)),
                        p2=(FX+45+2.5*math.cos(a), FY+HOLE_Z+2.5*math.sin(a)),
                        text="4x %%c5,00", **DIM).render()
text("VISTA FRONTAL", FX+L/2, FY-7, 3.5)

# --- Vista lateral esquerda (1o diedro: a direita da frontal)
SX = FX + L + 35
place(side, SX, FY)
vdim((SX+W, FY), (SX+W, FY+H), SX+W+8)
hdim((SX, FY+H), (SX+W, FY+H), YD)
msp.add_radius_dim(center=(FX+L-R_EXT, FY+R_EXT), radius=R_EXT, angle=315, text="4x R2,74 (dobras)", **DIM).render()
text("VISTA LATERAL ESQ.", SX+W/2, FY-7, 3.5)

# --- Vista superior (abaixo da frontal; frente da peca voltada para cima)
TX, TY = FX, 135            # TY = linha do fundo (y=54); frente em TY+W
place(top, TX, TY, flipy=True)
YF = TY + W                 # frente
hdim((TX, YF), (TX+GAP_X0, YF), YF+9)
hdim((TX+GAP_X0, YF), (TX+GAP_X1, YF), YF+9)
hdim((TX+GAP_X1, YF), (TX+L, YF), YF+9)
hdim((TX, TY), (TX+L, TY), TY-10)
vdim((TX+L, TY), (TX+L, YF), TX+L+10)
vdim((TX+GAP_X1+12, YF-T), (TX+GAP_X1+12, YF), TX+GAP_X1+12, text="2,00 ±0,30")
text("VISTA SUPERIOR", TX+L/2, TY-18, 3.5)

# --- Isometrica
place(iso, 325, 166, center=True)
text("ISOMETRICA (sem escala)", 325, 116, 3.0)

# --- Planificacao (blank em cruz: fundo + 4 abas, cantos com alivio)
ba = math.pi/2*(R_INT + K*T)                 # bend allowance por dobra
F = ba + (H - R_EXT)                         # aba desenvolvida (dobra + trecho reto)
BL, BW = L - 2*R_EXT, W - 2*R_EXT            # fundo entre tangentes
Lb, Wb = BL + 2*F, BW + 2*F
PX, PY = 30, 22
fx = lambda x: PX + F + (x - R_EXT)          # x da peca -> x do blank
g0, g1 = fx(GAP_X0), fx(GAP_X1)
yb = PY + F + BW                             # tangente fundo/aba frontal
outline = [(PX+F,PY),(PX+F+BL,PY),(PX+F+BL,PY+F),(PX+Lb,PY+F),(PX+Lb,yb),(PX+F+BL,yb),
           (PX+F+BL,PY+Wb),(g1,PY+Wb),(g1,yb+ba),(g0,yb+ba),(g0,PY+Wb),(PX+F,PY+Wb),
           (PX+F,yb),(PX,yb),(PX,PY+F),(PX+F,PY+F)]
msp.add_lwpolyline(outline, close=True, dxfattribs={"layer":"VISIVEL"})
def bl(p, q): msp.add_line(p, q, dxfattribs={"layer":"DOBRA"})
bl((PX+F, PY+F-ba/2), (PX+F+BL, PY+F-ba/2))            # fundo (traseira)
bl((PX+F, yb+ba/2), (g0, yb+ba/2)); bl((g1, yb+ba/2), (PX+F+BL, yb+ba/2))   # frente
bl((PX+F-ba/2, PY+F), (PX+F-ba/2, yb))                 # lateral esq.
bl((PX+F+BL+ba/2, PY+F), (PX+F+BL+ba/2, yb))           # lateral dir.
hy = PY + Wb - HOLE_Z
for x in HOLES_X:
    msp.add_circle((fx(x), hy), HOLE_D/2, dxfattribs={"layer":"VISIVEL"})
    cline((fx(x)-4, hy), (fx(x)+4, hy)); cline((fx(x), hy-4), (fx(x), hy+4))
YT = PY + Wb
hdim((PX, PY+F), (PX+Lb, PY+F), PY-8 if PY > 14 else PY-6)
vdim((PX+Lb, PY+F), (PX+Lb, yb), PX+Lb+7)
vdim((PX+F, PY), (PX+F, PY+Wb), PX-8)
vdim((PX+F+BL, PY), (PX+F+BL, PY+F), PX+F+BL+6)
vdim((PX+F+BL, hy), (PX+F+BL, YT), PX+F+BL+6)
hdim((PX+F, YT), (fx(30), hy), YT+7)
hdim((fx(30), hy), (fx(45), hy), YT+7)
hdim((fx(45), hy), (g0, YT), YT+7)
hdim((g0, YT), (g1, YT), YT+7)
hdim((g1, YT), (fx(115), hy), YT+7)
hdim((fx(115), hy), (fx(130), hy), YT+7)
hdim((fx(130), hy), (PX+F+BL, YT), YT+7)
text(f"PLANIFICACAO - blank {fmt(Lb)} x {fmt(Wb)} | 4 dobras 90° p/ cima | Ri {fmt(R_INT)} | K {K} (ajustar)",
     PX+Lb+12, PY+Wb/2+14, 2.6, align=ezdxf.enums.TextEntityAlignment.MIDDLE_LEFT)
text(f"Cantos abertos - alivio {fmt(F)} x {fmt(F)}", PX+Lb+12, PY+Wb/2+7, 2.6,
     align=ezdxf.enums.TextEntityAlignment.MIDDLE_LEFT)
flat_len = Lb
# --- Legenda
BX, BY = 290, 10
msp.add_lwpolyline([(BX,BY),(410,BY),(410,BY+50),(BX,BY+50)], close=True, dxfattribs={"layer":"MOLDURA"})
for yy in (BY+10, BY+20, BY+30, BY+40):
    msp.add_line((BX, yy), (410, yy), dxfattribs={"layer":"MOLDURA"})
rows = ["SUPORTE EM CHAPA - BANDEJA", "Material: Aco SAE 1020 Galvanizado  t = 2,00 mm",
        "Quantidade: 500 pecas", "Escala 1:1   Folha A3   Unid.: mm   1o diedro",
        "Tol. ISO 2768-m   Ri dobra 0,74   Massa ~205 g"]
for i, r in enumerate(rows):
    text(r, BX+3, BY+45-10*i, 4.0 if i == 0 else 2.8, align=ezdxf.enums.TextEntityAlignment.MIDDLE_LEFT)

doc.saveas(f"{OUT}/suporte_desenho.dxf")

# ---------------- PDF / PNG a partir do DXF ----------------
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from ezdxf.addons.drawing import RenderContext, Frontend
from ezdxf.addons.drawing.config import Configuration, ColorPolicy, BackgroundPolicy
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
fig = plt.figure(figsize=(420/25.4, 297/25.4))
ax = fig.add_axes([0, 0, 1, 1]); ax.set_axis_off()
cfg = Configuration(color_policy=ColorPolicy.BLACK, background_policy=BackgroundPolicy.WHITE,
                    lineweight_scaling=1.0, min_lineweight=0.1)
Frontend(RenderContext(doc), MatplotlibBackend(ax), config=cfg).draw_layout(msp, finalize=True)
ax.set_xlim(0, 420); ax.set_ylim(0, 297)
fig.savefig(f"{OUT}/suporte_desenho.pdf")
fig.savefig(f"{OUT}/suporte_desenho.png", dpi=150)
print(f"blank: {Lb:.2f} x {Wb:.2f} mm  (BA por dobra {ba:.3f})")

# ---------------- FreeCAD (.FCStd) ----------------
# .FCStd = zip com Document.xml + forma em BREP; abre direto no FreeCAD (Part::Feature).
import zipfile
from OCP.BRepTools import BRepTools
brep = f"{OUT}/_tmp.brp"
BRepTools.Write_s(part.val().wrapped, brep)
doc_xml = """<?xml version='1.0' encoding='utf-8'?>
<Document SchemaVersion="4" ProgramVersion="0.21R" FileVersion="1">
    <Properties Count="1">
        <Property name="Label" type="App::PropertyString"><String value="suporte"/></Property>
    </Properties>
    <Objects Count="1">
        <Object type="Part::Feature" name="Suporte" id="1"/>
    </Objects>
    <ObjectData Count="1">
        <Object name="Suporte">
            <Properties Count="2">
                <Property name="Label" type="App::PropertyString"><String value="Suporte_chapa_t2"/></Property>
                <Property name="Shape" type="Part::PropertyPartShape"><Part file="Suporte.Shape.brp"/></Property>
            </Properties>
        </Object>
    </ObjectData>
</Document>
"""
with zipfile.ZipFile(f"{OUT}/suporte.FCStd", "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("Document.xml", doc_xml)
    z.write(brep, "Suporte.Shape.brp")
os.remove(brep)
