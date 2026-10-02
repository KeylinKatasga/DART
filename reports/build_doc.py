from pathlib import Path
import re
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.opc.constants import RELATIONSHIP_TYPE as RT

root=Path(__file__).resolve().parents[1]
doc=Document()
sec=doc.sections[0]
sec.page_width=Inches(8.5); sec.page_height=Inches(11)
sec.top_margin=sec.bottom_margin=Inches(.7)
sec.left_margin=sec.right_margin=Inches(.75)
for name in ['Normal','Title','Heading 1','Heading 2','Heading 3']:
    s=doc.styles[name]; s.font.name='Noto Sans CJK SC'; s.font.color.rgb=RGBColor(0,0,0)
    s.element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),'Noto Sans CJK SC')
doc.styles['Normal'].font.size=Pt(10.5)
doc.styles['Normal'].paragraph_format.line_spacing=1.15
doc.styles['Normal'].paragraph_format.space_after=Pt(6)
doc.styles['Title'].font.size=Pt(22)
doc.styles['Heading 1'].font.size=Pt(16)
doc.styles['Heading 2'].font.size=Pt(12)

def content(p,text):
    # Preserve actual clickable source links in the Word document.
    cursor=0
    for m in re.finditer(r'\[([^\]]+)\]\((https?://[^)]+)\)',text):
        p.add_run(text[cursor:m.start()])
        hl=OxmlElement('w:hyperlink'); hl.set(qn('r:id'),p.part.relate_to(m[2],RT.HYPERLINK,is_external=True))
        r=OxmlElement('w:r'); pr=OxmlElement('w:rPr'); color=OxmlElement('w:color'); color.set(qn('w:val'),'245B78'); pr.append(color); r.append(pr)
        t=OxmlElement('w:t'); t.text=m[1]; r.append(t); hl.append(r); p._p.append(hl); cursor=m.end()
    p.add_run(text[cursor:])

lines=(root/'reports/教学仿真与硬件选型.md').read_text().splitlines()
i=0
while i<len(lines):
    line=lines[i]
    if not line.strip(): i+=1; continue
    if line.startswith('|'):
        rows=[]
        while i<len(lines) and lines[i].startswith('|'):
            cells=[x.strip() for x in lines[i].strip('|').split('|')]
            if not all(re.fullmatch(r'[: -]+',c) for c in cells): rows.append(cells)
            i+=1
        table=doc.add_table(rows=0,cols=len(rows[0])); table.alignment=WD_TABLE_ALIGNMENT.CENTER
        table.autofit=False
        widths=[1.35,3.95,1.7] if len(rows[0])==3 else [2.8,4.2]
        if rows[0][0]=='对比维度': widths=[1.35,2.8,2.85]
        for col,w in zip(table.columns,widths): col.width=Inches(w)
        borders=OxmlElement('w:tblBorders')
        for edge in ['top','left','bottom','right','insideH','insideV']:
            e=OxmlElement('w:'+edge); e.set(qn('w:val'),'single'); e.set(qn('w:sz'),'4'); e.set(qn('w:color'),'D9D9D9'); borders.append(e)
        table._tbl.tblPr.append(borders)
        for ri,row in enumerate(rows):
            cells=table.add_row().cells
            trpr=table.rows[-1]._tr.get_or_add_trPr()
            ns=OxmlElement('w:cantSplit'); trpr.append(ns)
            if ri==0:
                repeat=OxmlElement('w:tblHeader'); trpr.append(repeat)
            for ci,(cell,txt) in enumerate(zip(cells,row)):
                cell.width=Inches(widths[ci]); cell.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
                tcpr=cell._tc.get_or_add_tcPr(); shade=OxmlElement('w:shd'); shade.set(qn('w:fill'),'E3EDF3' if ri==0 else 'FFFFFF'); tcpr.append(shade)
                margins=OxmlElement('w:tcMar')
                for edge in ['top','left','bottom','right']:
                    e=OxmlElement('w:'+edge); e.set(qn('w:w'),'90'); e.set(qn('w:type'),'dxa'); margins.append(e)
                tcpr.append(margins)
                p=cell.paragraphs[0]; p.paragraph_format.space_after=Pt(2); p.paragraph_format.space_before=Pt(2)
                content(p,txt)
                for run in p.runs: run.font.size=Pt(9); run.bold=ri==0
        doc.add_paragraph().paragraph_format.space_after=Pt(0)
        continue
    if line.startswith('# '): doc.add_paragraph(line[2:],'Title')
    elif line.startswith('## '): doc.add_heading(line[3:],level=1)
    elif line.startswith('### '): doc.add_heading(line[4:],level=2)
    else:
        p=doc.add_paragraph(); content(p,line)
        if re.match(r'^\d+\. \[',line):
            p.paragraph_format.space_after=Pt(0)
            p.paragraph_format.space_before=Pt(0)
            p.paragraph_format.line_spacing=Pt(15)
            snap=OxmlElement('w:snapToGrid'); snap.set(qn('w:val'),'0'); p._p.get_or_add_pPr().append(snap)
    i+=1
doc.core_properties.title='被动运动教学仿真与视觉硬件选型'
doc.core_properties.subject='MATLAB 安装核查 教学实验 视觉平台比较'
for element in [doc.styles.element,doc.element]:
    for border in list(element.iter(qn('w:pBdr'))):
        border.getparent().remove(border)
doc.save(root/'reports/教学仿真与硬件选型.docx')
print('DOCX written')
