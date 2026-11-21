"""A4 registration patterns at actual size; no drill or cutting template."""
from pathlib import Path
import math
import os
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
import damiao_drive as dm

out = Path(__file__).with_name("DM4340P_fit_sheet.pdf")
c = Canvas(str(out), pagesize=A4)
c.setTitle("DM4340P 24V | 고정판과 풀리 맞춤 확인")
for family, file in [('KR','malgun.ttf'), ('KR-B','malgunbd.ttf')]:
    pdfmetrics.registerFont(TTFont(family, str(Path(os.environ['WINDIR'])/'Fonts'/file)))
ink, teal, muted = '#142F38','#006B62','#516670'

def label(x,y,s,size=10,bold=False,color=ink):
    c.setFillColor(color);c.setFont('KR-B' if bold else 'KR',size)
    c.drawString(x*mm,y*mm,s)
def line(x,y,xx,yy,color=muted,width=0.5):
    c.setStrokeColor(color);c.setLineWidth(width);c.line(x*mm,y*mm,xx*mm,yy*mm)
def circle(x,y,r,color=ink,width=0.7):
    c.setStrokeColor(color);c.setLineWidth(width);c.circle(x*mm,y*mm,r*mm)

label(15,274,'모터가 맞는지 확인하는 종이 도면',23,True)
label(15,264,'Damiao DM4340P 24V   /   무동력 맞춤용   /   단위 mm',11,color=teal)
label(15,249,'A4 용지에 실제 크기(100%)로 인쇄한 뒤, 먼저 아래 눈금이 50mm인지 재세요.')
line(15,240,65,240,ink,1)
for x in (15,65): line(x,238,x,242,ink,1)
label(69,239,'50 mm',9)
label(15,222,'고정판 쪽  ·  뒷면',14,True)
label(115,222,'풀리 쪽  ·  회전면',14,True)
for cx, dia, count, phase, hole in [(55,dm.REAR_PCD,4,45,dm.REAR_CLEARANCE),(155,dm.OUTPUT_PCD,6,90,3.4)]:
    cy=184
    circle(cx,cy,28.5)
    c.setDash([2,2]);circle(cx,cy,dia/2,muted);c.setDash([])
    line(cx-5,cy,cx+5,cy);line(cx,cy-5,cx,cy+5)
    for x,y in dm.bolt_centres(count,dia,phase):
        circle(cx+x,cy+y-dm.CENTRE,hole/2,teal,1)
label(15,147,'구멍 4개 · 지름 38mm인 원 위',10,True)
label(115,147,'구멍 6개 · 지름 27mm인 원 위',10,True)
label(15,138,f'고정판 구멍 Ø{dm.REAR_CLEARANCE:g}mm · 두 나사 공용.',9)
label(115,138,'M3 나사 · 모터 안쪽 나사 깊이 6mm.',9)
label(15,131,'볼트와 평와셔는 받은 모터 규격에 맞춰 선택.',9)
label(115,131,'모터 바깥 고정 링의 구멍과 구별하세요.',9)
line(15,120,195,120)
label(15,108,'벨트가 지나는 높이',14,True)
label(15,99,'팔꿈치 축과 모터 축 사이 147.5mm. 두 풀리의 벨트 중심면을 Z132에 배치했습니다.')
label(15,90,'모터 회전면 Z122에서 벨트 중심까지 10mm. 치형 폭 17mm, 벨트 폭 15mm를 가정했습니다.')
label(15,78,'공개 3D 모델로 정한 명목 치수입니다. 실제 풀리의 높이와 쏠림은 조립 후 확인하세요.',9,color=muted)
line(15,67,195,67)
label(15,56,'모터 도착 후',13,True)
label(15,46,'① 실물의 네 구멍 위치를 대조하고, 맞는 볼트와 평와셔를 고르세요. 억지로 조이지 마세요.')
label(15,36,'② 판 구멍에는 여유가 있습니다. 실물에서 모터 중심과 벨트 정렬, 체결 깊이를 확인하세요.')
label(15,22,'참고용 외곽은 모터 지름 57mm입니다. 이 종이는 가공이나 드릴 작업에 사용하지 마세요.',9,color=teal)
c.showPage();c.save()
