"""Tessellate a downloaded SUCHAI-II STEP assembly for the Blender asset builder.

Requires cadquery-ocp==8.0.1.1.0 in a separate Python environment.
Usage: python convert_suchai_step.py INPUT.stp OUTPUT.json
"""

import argparse
import json
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.collections import Sequence_TDF_Label
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source")
parser.add_argument("output")
args = parser.parse_args()
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)

if reader.ReadFile(args.source) != IFSelect_RetDone:
    raise RuntimeError("Could not read STEP assembly")
doc = TDocStd_Document(TCollection_ExtendedString("XCAF"))
if not reader.Transfer(doc):
    raise RuntimeError("Could not transfer STEP assembly")
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = Sequence_TDF_Label()
tool.GetFreeShapes(roots)
parts = []


def visit(label, location, path):
    if tool.IsReference_s(label):
        location = location.Multiplied(tool.GetLocation_s(label))
        ref = TDF_Label()
        tool.GetReferredShape_s(label, ref)
        label = ref
    name = TDataStd_Name()
    n = (
        name.Get().ToExtString()
        if label.FindAttribute(TDataStd_Name.GetID_s(), name)
        else "Part"
    )
    path = path + "/" + n
    if tool.IsAssembly_s(label):
        children = Sequence_TDF_Label()
        tool.GetComponents_s(label, children)
        for i in range(1, children.Length() + 1):
            visit(children.Value(i), location, path)
        return
    # Omit deployed antennas and exterior solar panels for the internal cutaway.
    if "nanocom-ant430" in path or "/P110U" in path:
        return
    shape = tool.GetShape_s(label).Moved(location)
    bounds = Bnd_Box()
    BRepBndLib.Add_s(shape, bounds)
    if bounds.IsVoid():
        return
    lo = bounds.CornerMin()
    hi = bounds.CornerMax()
    lohi = [lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()]
    BRepMesh_IncrementalMesh(shape, 0.25, False, 0.35, True)
    vertices = []
    triangles = []
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    while faces.More():
        face = TopoDS.Face(faces.Current())
        loc = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, loc)
        if mesh:
            offset = len(vertices)
            for i in range(1, mesh.NbNodes() + 1):
                p = mesh.Node(i).Transformed(loc.Transformation())
                vertices.append([p.X(), p.Y(), p.Z()])
            for i in range(1, mesh.NbTriangles() + 1):
                a, b, c = mesh.Triangle(i).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                triangles.append([offset + a - 1, offset + b - 1, offset + c - 1])
        faces.Next()
    parts.append(
        {"name": path, "vertices": vertices, "triangles": triangles, "bounds": lohi}
    )
    print(len(parts), path, len(triangles), lohi, flush=True)


for i in range(1, roots.Length() + 1):
    visit(roots.Value(i), TopLoc_Location(), "")
with open(args.output, "w") as f:
    json.dump(parts, f, separators=(",", ":"))
print("DONE", len(parts), sum(len(p["triangles"]) for p in parts), flush=True)
