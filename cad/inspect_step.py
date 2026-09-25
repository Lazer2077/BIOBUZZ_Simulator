import sys
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
status = reader.ReadFile(sys.argv[1])
print('STEP read status:', status, flush=True)
doc = TDocStd_Document(TCollection_ExtendedString('BIOBUZZ'))
reader.Transfer(doc)
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = TDF_LabelSequence()
tool.GetFreeShapes(roots)
print('Roots:', roots.Length(), flush=True)

def name(label):
    att = TDataStd_Name()
    return att.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), att) else ''

def walk(label, depth=0):
    ref = TDF_Label()
    is_ref = XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
    label_name = name(label) or (name(ref) if is_ref else '')
    if 'Cell' in label_name or 'Hive' in label_name or depth <= 1:
        shape = XCAFDoc_ShapeTool.GetShape_s(label)
        box = Bnd_Box()
        BRepBndLib.Add_s(shape, box)
        print('  '*depth, label_name[:90], 'bbox', tuple(round(x,1) for x in box.Get()) if not box.IsVoid() else None, flush=True)
    seq = TDF_LabelSequence()
    if XCAFDoc_ShapeTool.GetComponents_s(label, seq, False):
        for i in range(1,seq.Length()+1): walk(seq.Value(i), depth+1)
    elif is_ref and XCAFDoc_ShapeTool.GetComponents_s(ref, seq, False):
        for i in range(1,seq.Length()+1): walk(seq.Value(i), depth+1)

for i in range(1,roots.Length()+1): walk(roots.Value(i))
