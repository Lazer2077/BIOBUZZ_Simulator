"""Extract a tessellation of the official FIRST STEP HIVE assemblies."""
import json
import sys
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location

source, destination = map(Path, sys.argv[1:3])
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
assert reader.ReadFile(str(source)).name.endswith('RetDone')
doc = TDocStd_Document(TCollection_ExtendedString('BIOBUZZ'))
reader.Transfer(doc)
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = TDF_LabelSequence()
tool.GetFreeShapes(roots)

def name(label):
    att = TDataStd_Name()
    return att.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), att) else ''

def find_hives(label):
    ref = TDF_Label()
    is_ref = XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
    title = name(label) or (name(ref) if is_ref else '')
    if title.startswith('am-5853-') and 'Hive' in title:
        yield title, label
        return
    seq = TDF_LabelSequence()
    if XCAFDoc_ShapeTool.GetComponents_s(label, seq, False):
        for i in range(1, seq.Length()+1): yield from find_hives(seq.Value(i))
    elif is_ref and XCAFDoc_ShapeTool.GetComponents_s(ref, seq, False):
        for i in range(1, seq.Length()+1): yield from find_hives(seq.Value(i))

def find_cell_parts(label):
    ref = TDF_Label()
    is_ref = XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
    title = name(label) or (name(ref) if is_ref else '')
    if any(word in title for word in ('Goal Rib', 'Hive Goal Top Skin', 'Hive Goal Bottom Skin', 'Hive Goal Back Skin')):
        yield title, XCAFDoc_ShapeTool.GetShape_s(label)
        return
    seq = TDF_LabelSequence()
    if XCAFDoc_ShapeTool.GetComponents_s(label, seq, False):
        for i in range(1, seq.Length()+1): yield from find_cell_parts(seq.Value(i))
    elif is_ref and XCAFDoc_ShapeTool.GetComponents_s(ref, seq, False):
        for i in range(1, seq.Length()+1): yield from find_cell_parts(seq.Value(i))

def find_cells(label):
    ref = TDF_Label()
    is_ref = XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
    title = name(label) or (name(ref) if is_ref else '')
    if ' Cell (' in title:
        yield title, label
        return
    seq = TDF_LabelSequence()
    if XCAFDoc_ShapeTool.GetComponents_s(label, seq, False):
        for i in range(1, seq.Length()+1): yield from find_cells(seq.Value(i))
    elif is_ref and XCAFDoc_ShapeTool.GetComponents_s(ref, seq, False):
        for i in range(1, seq.Length()+1): yield from find_cells(seq.Value(i))

meshes = {}
for i in range(1, roots.Length()+1):
    for title, label in find_hives(roots.Value(i)):
        name_id = 'red' if 'Red' in title else 'blue'
        meshes[name_id] = {}
        for cell_title, cell_label in find_cells(label):
          side = 'plus' if 'Audience' in cell_title else 'minus'
          groups = {'rib': [], 'shell': [], 'back': []}
          for part_title, shape in find_cell_parts(cell_label):
            part_id = 'rib' if 'Goal Rib' in part_title else 'back' if 'Back Skin' in part_title else 'shell'
            positions = groups[part_id]
            before = len(positions)
            BRepMesh_IncrementalMesh(shape, 8.0, False, .75, True)
            explorer = TopExp_Explorer(shape, TopAbs_FACE)
            while explorer.More():
                face = TopoDS.Face_s(explorer.Current())
                loc = TopLoc_Location()
                tri = BRep_Tool.Triangulation_s(face, loc)
                if tri:
                    tx = loc.Transformation()
                    for j in range(1, tri.NbTriangles()+1):
                        triangle = tri.Triangle(j)
                        for index in triangle.Get():
                            p = tri.Node(index).Transformed(tx)
                            positions.extend((round(p.X()/1000, 4), round(p.Z()/1000, 4), round(p.Y()/1000, 4)))
                explorer.Next()
            print(name_id,side,part_title, 'triangles', (len(positions)-before)//9, flush=True)
          meshes[name_id][side] = groups
        print(title, 'triangles', sum(len(v)//9 for cell in meshes[name_id].values() for v in cell.values()), flush=True)

destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text('window.HIVE_CAD_MESH=' + json.dumps(meshes, separators=(',', ':')) + ';', encoding='utf-8')
print('Wrote', destination, destination.stat().st_size, 'bytes')
